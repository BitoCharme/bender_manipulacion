#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, JointState
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import numpy as np
import time
import cv2, tf2_ros, tf2_geometry_msgs, math


# =========================================================
# ✅ Kalman mejorado (x, y, z, vx, vy, vz)
# =========================================================
class SimpleKalman3D:
    def __init__(self, q_pos=1e-3, q_vel=1e-2, r_meas=2e-3, gate_thresh=9.0):
        self.x = None
        self.P = None
        self.Q_base = np.diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel]).astype(np.float64)
        self.R = (r_meas * np.eye(3)).astype(np.float64)
        self.H = np.zeros((3, 6), dtype=np.float64)
        self.H[0,0] = self.H[1,1] = self.H[2,2] = 1.0
        self.gate_thresh = gate_thresh
        self.initialized = False
        self.last_t = None

        # Parámetros adicionales de robustez
        self.switch_distance_thresh = 0.10  # máxima distancia 3D para cambiar target
        self.stickiness_frames = 6          # histéresis de seguimiento
        self.missed_frames = 0

    def init(self, z, t):
        self.x = np.zeros((6, 1), dtype=np.float64)
        self.x[0,0], self.x[1,0], self.x[2,0] = z
        self.P = np.diag([0.01,0.01,0.01, 0.1,0.1,0.1]).astype(np.float64)
        self.initialized = True
        self.last_t = t
        self.missed_frames = 0

    def _F_and_Q(self, dt):
        F = np.eye(6)
        F[0,3] = dt; F[1,4] = dt; F[2,5] = dt
        Q = self.Q_base.copy()
        Q[0,0]*=dt*dt; Q[1,1]*=dt*dt; Q[2,2]*=dt*dt
        Q[3,3]*=dt; Q[4,4]*=dt; Q[5,5]*=dt
        return F, Q

    def predict(self, t):
        if not self.initialized:
            return
        dt = max(1e-3, float(t - self.last_t))
        F, Q = self._F_and_Q(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.last_t = t

    def innovation(self, z):
        y = (z.reshape(3,1) - self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        return y, S

    def gate_distance2(self, z):
        y, S = self.innovation(z)
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return np.inf
        return float(y.T @ S_inv @ y)

    def update(self, z):
        y, S = self.innovation(z)
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P

    def get_pos(self):
        if not self.initialized:
            return None
        return float(self.x[0,0]), float(self.x[1,0]), float(self.x[2,0])


# =========================================================
# 🧠 Nodo YOLO + Depth + Kalman + heurística mejorada
# =========================================================
class YoloDepthNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_node')
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")

        # Suscripciones y publicaciones
        self.sub_image = self.create_subscription(Image, '/camera/color/image_raw', self.image_cb, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)
        self.sub_cam_info = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.cam_cb, 10)
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.pub_point = self.create_publisher(PointStamped, '/object/point', 10)
        self.pub_image = self.create_publisher(Image, '/detections/image', 10)
        self.pub_marker = self.create_publisher(Marker, '/detections/marker', 10)
        self.pub_trigger = self.create_publisher(Bool, '/auto_grasp_trigger', 10)

        # Parámetros de compensación
        self.declare_parameter("enable_z_comp", True)
        self.declare_parameter("z_comp_gain", 1.0)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Estado interno
        self.fx = self.fy = self.cx = self.cy = None
        self.latest_depth = None
        self.ee_links = ["l6r_1", "l6l_1"]
        self.ee_projections = []
        self.exclude_radius_px = 75
        self.margin_px = 10  # borde a ignorar

        # Estabilidad
        self.last_positions = []
        self.required_stable_frames = 5
        self.trigger_sent = False

        # Filtro Kalman
        self.kf = SimpleKalman3D(q_pos=1e-3, q_vel=5e-3, r_meas=4e-4, gate_thresh=9.0)

        self.get_logger().info("    YOLO")

    # ------------------------------
    def cam_cb(self, msg):
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]

    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def joint_cb(self, _):
        self.ee_projections.clear()
        if None in (self.fx, self.fy, self.cx, self.cy):
            return
        now = rclpy.time.Time()
        for link in self.ee_links:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'camera_depth_optical_frame', link, now,
                    timeout=rclpy.duration.Duration(seconds=0.2)
                )
                x, y, z = tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z
                if 0.05 < z < 2.5:
                    u = int(self.fx * (x / z) + self.cx)
                    v = int(self.fy * (y / z) + self.cy)
                    if 0 <= u < 424 and 0 <= v < 240:
                        self.ee_projections.append((u, v))
            except Exception:
                continue

    # ------------------------------
    def image_cb(self, msg):
        now_t = self.get_clock().now().nanoseconds * 1e-9
        if self.latest_depth is None or None in (self.fx, self.fy, self.cx, self.cy):
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # === Máscara del gripper ===
        mask = np.ones(frame.shape[:2], dtype=np.uint8) * 255
        for gx, gy in self.ee_projections:
            cv2.circle(mask, (gx, gy), int(self.exclude_radius_px * 0.6), 0, -1)
        frame_masked = cv2.bitwise_and(frame, frame, mask=mask)

        # === YOLO detección ===
        results = self.model(frame_masked, conf=0.001)
        candidates = []

        if results and len(results[0].boxes) > 0:
            h, w = frame.shape[:2]
            for box in results[0].boxes:
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                area = max(1, (x2 - x1) * (y2 - y1))
                # Ignorar objetos muy grandes o fuera del área útil
                if (x2 - x1) > 200 or (y2 - y1) > 200:
                    continue
                if x1 < self.margin_px or y1 < self.margin_px or x2 > (w - self.margin_px) or y2 > (h - self.margin_px):
                    continue
                x_cam, y_cam, z_cam = self.compute_3d(self.latest_depth, x1, y1, x2, y2)
                if np.isfinite(z_cam) and 0.05 < z_cam < 2.5:
                    candidates.append(((x1, y1, x2, y2, area), (x_cam, y_cam, z_cam)))

        # === Kalman predict/update ===
        if self.kf.initialized:
            self.kf.predict(now_t)

        matched_bbox, meas_used = None, None

        # --- SELECCIÓN DE OBJETO PRIORITARIO ---
        if not self.kf.initialized:
            if candidates:
                # Elegir el bounding box más pequeño (prioridad)
                (x1, y1, x2, y2, _), meas = min(candidates, key=lambda c: c[0][4])
                self.kf.init(np.array(meas, dtype=np.float64), now_t)
                matched_bbox, meas_used = (x1, y1, x2, y2), meas
        else:
            if candidates:
                pred = np.array(self.kf.get_pos(), dtype=np.float64)
                best, best_d2, best_area, best_dist3d = None, np.inf, np.inf, np.inf
                for bbox, meas in candidates:
                    x1, y1, x2, y2, area = bbox
                    z = np.array(meas, dtype=np.float64)
                    d2 = self.kf.gate_distance2(z)
                    dist3d = np.linalg.norm(z - pred)
                    # prioridad: dentro del gate y menor área
                    if d2 < self.kf.gate_thresh and (area < best_area or dist3d < best_dist3d):
                        best, best_d2, best_area, best_dist3d = (bbox, meas), d2, area, dist3d

                if best:
                    (x1, y1, x2, y2, _), meas_used = best
                    matched_bbox = (x1, y1, x2, y2)
                    self.kf.update(np.array(meas_used, dtype=np.float64))
                    self.kf.missed_frames = 0
                else:
                    self.kf.missed_frames += 1
                    if self.kf.missed_frames > self.kf.stickiness_frames:
                        self.kf.initialized = False
                        self.kf.missed_frames = 0

        # === Publicar ===
        filt_pos = self.kf.get_pos()
        if filt_pos:
            self.publish_point_and_marker(frame, matched_bbox if matched_bbox else (0,0,0,0), filt_pos)

        self._draw_and_publish_image(frame, [matched_bbox] if matched_bbox else [], None)

    # ------------------------------
    def compute_3d(self, depth, x1, y1, x2, y2, step=2, min_samples=8):
        cx, cy = int((x1+x2)/2), int((y1+y2)/2)
        radius = max(4, int(min(x2-x1, y2-y1)/3))
        h, w = depth.shape[:2]
        u0, u1 = max(0, cx-radius), min(w-1, cx+radius)
        v0, v1 = max(0, cy-radius), min(h-1, cy+radius)
        zs=[]
        for v in range(v0, v1, step):
            for u in range(u0, u1, step):
                if (u-cx)**2+(v-cy)**2>radius**2: continue
                z=depth[v,u]
                if np.isnan(z) or z<=0: continue
                z=z/1000.0 if z>10 else float(z)
                if 0.08<z<3.0: zs.append(z)
        if len(zs)<min_samples: return float('nan'),float('nan'),float('nan')
        z_med=float(np.median(zs))
        x=(cx-self.cx)*z_med/self.fx
        y=(cy-self.cy)*z_med/self.fy

        if self.get_parameter("enable_z_comp").value:
            gain = self.get_parameter("z_comp_gain").value
            angle = math.atan2(math.sqrt(x**2 + y**2), z_med)
            z_corr = z_med * math.cos(angle) * gain
            return x, y, z_corr
        else:
            return x, y, z_med

    # ------------------------------
    def _draw_and_publish_image(self, frame, bboxes, _unused):
        for (gx, gy) in self.ee_projections:
            cv2.circle(frame, (gx, gy), self.exclude_radius_px, (0,0,255), 2)
        for bb in bboxes:
            if bb:
                x1,y1,x2,y2 = bb
                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def publish_point_and_marker(self, frame, bbox, point_3d):
        x_cam, y_cam, z_cam = point_3d
        pt_cam = PointStamped()
        pt_cam.header.frame_id = "camera_depth_optical_frame"
        pt_cam.header.stamp = self.get_clock().now().to_msg()
        pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = x_cam, y_cam, z_cam

        try:
            tf = self.tf_buffer.lookup_transform('base_link', 'camera_depth_optical_frame', rclpy.time.Time())
            pt_base = tf2_geometry_msgs.do_transform_point(pt_cam, tf)
            bx, by, bz = pt_base.point.x, pt_base.point.y, pt_base.point.z
        except Exception:
            bx, by, bz = x_cam, y_cam, z_cam

        pm = PointStamped()
        pm.header.frame_id = 'base_link'
        pm.point.x, pm.point.y, pm.point.z = bx, by, bz
        self.pub_point.publish(pm)

        m = Marker()
        m.header.frame_id = 'base_link'
        m.type = Marker.SPHERE
        m.pose.position.x, m.pose.position.y, m.pose.position.z = bx, by, bz
        m.scale.x = m.scale.y = m.scale.z = 0.05
        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        self.pub_marker.publish(m)


# =========================================================
# MAIN
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
