#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import tf2_ros, tf2_geometry_msgs, cv2, math, random, time


# =========================================================
# ✅ Kalman 3D (x, y, z, vx, vy, vz)
# =========================================================
class SimpleKalman3D:
    def __init__(self):
        self.x = np.zeros((6, 1))
        self.P = np.eye(6) * 0.02
        self.Q = np.diag([5e-4,5e-4,5e-4, 5e-3,5e-3,5e-3])   # suavizado
        self.R = np.eye(3) * 1e-3
        self.H = np.zeros((3, 6))
        self.H[0,0] = self.H[1,1] = self.H[2,2] = 1.0
        self.initialized = False
        self.last_t = None
        self.missed = 0

    def init(self, z, t):
        self.x[:3, 0] = z
        self.P = np.eye(6) * 0.01
        self.initialized = True
        self.last_t = t
        self.missed = 0

    def predict(self, t):
        if not self.initialized: return
        dt = max(1e-3, t - self.last_t)
        F = np.eye(6)
        for i in range(3):
            F[i, i+3] = dt
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
        self.last_t = t

    def update(self, z):
        y = z.reshape(3,1) - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P

    def get_pos(self):
        return tuple(self.x[:3,0])


# =========================================================
# 🧠 Nodo YOLO + Depth + Multi-Kalman + Filtro mesa refinado
# =========================================================
class MultiObjectYoloNode(Node):
    def __init__(self):
        super().__init__('yolo_multi_marker_node')
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")

        # Suscripciones
        self.sub_image = self.create_subscription(Image, '/camera/color/image_raw', self.image_cb, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)
        self.sub_cam_info = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.cam_cb, 10)

        # Publicadores
        self.pub_markers = self.create_publisher(MarkerArray, '/detections/marker_array', 10)
        self.pub_debug_image = self.create_publisher(Image, '/detections/debug_image', 10)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Estado
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_img = None
        self.kalmans = {}       # id -> (kalman, color)
        self.max_missed = 40    # ✅ más tolerante ante frames sin detección
        self.margin_px = 10

        self.get_logger().info("🎯 YOLO + Depth + Multi-Kalman (suavizado) + Mesa filtrada ✅")

    # ------------------------------
    def cam_cb(self, msg):
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]

    def depth_cb(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # ------------------------------
    def image_cb(self, msg):
        if self.depth_img is None or None in (self.fx, self.fy, self.cx, self.cy):
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        now_t = time.time()
        results = self.model(frame, conf=0.01, verbose=False)

        detections = []
        if results and len(results[0].boxes) > 0:
            h, w = frame.shape[:2]
            for box in results[0].boxes:
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
                area = (x2 - x1) * (y2 - y1)
                # descartar bordes y mesas
                if (x2 - x1) > 100 or (y2 - y1) > 100 or area > 40000:
                    continue
                if x1 < self.margin_px or y1 < self.margin_px or x2 > w - self.margin_px or y2 > h - self.margin_px:
                    continue

                x3d, y3d, z3d, std_z, bbox_w_m = self.compute_3d(self.depth_img, x1, y1, x2, y2)
                # descartar planos (mesa) o sin profundidad válida
                if not np.isfinite(z3d) or z3d < 0.05 or z3d > 3.0 or std_z < 0.002:
                    continue

                detections.append(((x1, y1, x2, y2, bbox_w_m), np.array([x3d, y3d, z3d])))

        # Actualizar/crear Kalman
        new_kalmans = {}
        for i, (bbox, z) in enumerate(detections):
            kalman_id = i
            if kalman_id not in self.kalmans:
                self.kalmans[kalman_id] = (SimpleKalman3D(), self.random_color())
                self.kalmans[kalman_id][0].init(z, now_t)
            else:
                kf, _ = self.kalmans[kalman_id]
                kf.predict(now_t)
                kf.update(z)
                kf.missed = 0
            new_kalmans[kalman_id] = self.kalmans[kalman_id]

        # Mantener los Kalmans antiguos si no hay detección puntual
        for k in list(self.kalmans.keys()):
            if k not in new_kalmans:
                kf, color = self.kalmans[k]
                kf.predict(now_t)
                kf.missed += 1
                if kf.missed <= self.max_missed:
                    new_kalmans[k] = (kf, color)

        self.kalmans = new_kalmans

        # Publicar MarkerArray e imagen debug
        self.publish_markers(detections)
        self.publish_debug_image(frame, detections)

    # ------------------------------
    def compute_3d(self, depth, x1, y1, x2, y2, step=2):
        cx, cy = int((x1+x2)/2), int((y1+y2)/2)
        h, w = depth.shape[:2]
        radius = max(3, int(min(x2-x1, y2-y1)/4))
        u0, u1 = max(0, cx-radius), min(w-1, cx+radius)
        v0, v1 = max(0, cy-radius), min(h-1, cy+radius)
        zs = []
        for v in range(v0, v1, step):
            for u in range(u0, u1, step):
                z = depth[v,u]
                if np.isnan(z) or z <= 0: continue
                z = z/1000.0 if z > 10 else float(z)
                if 0.05 < z < 3.0: zs.append(z)
        if len(zs) < 5:
            return float('nan'), float('nan'), float('nan'), 0.0, 0.0
        z_med = np.median(zs)
        std_z = np.std(zs)
        x = (cx - self.cx) * z_med / self.fx
        y = (cy - self.cy) * z_med / self.fy

        # Ancho proyectado en metros
        bbox_w_px = x2 - x1
        bbox_w_m = (bbox_w_px / self.fx) * z_med
        return x, y, z_med, std_z, bbox_w_m

    # ------------------------------
    def random_color(self):
        return (random.random(), random.random(), random.random())

    def publish_markers(self, detections):
        marker_array = MarkerArray()
        for obj_id, (kf, color) in self.kalmans.items():
            pos = kf.get_pos()
            if pos is None:
                continue
            x, y, z = pos

            pt_cam = PointStamped()
            pt_cam.header.frame_id = "camera_depth_optical_frame"
            pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = x, y, z

            try:
                tf = self.tf_buffer.lookup_transform('base_link', 'camera_depth_optical_frame', rclpy.time.Time())
                pt_base = tf2_geometry_msgs.do_transform_point(pt_cam, tf)
                bx, by, bz = pt_base.point.x, pt_base.point.y, pt_base.point.z
            except Exception:
                bx, by, bz = x, y, z

            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "objects"
            m.id = obj_id
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = bx, by, bz
            m.pose.orientation.x = m.pose.orientation.y = m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0

            # Escala basada en el ancho del bbox proyectado
            bbox = detections[obj_id][0] if obj_id < len(detections) else (0,0,10,10,0.05)
            _, _, _, _, bbox_w_m = bbox
            m.scale.x = m.scale.y = max(0.03, bbox_w_m)
            m.scale.z = 0.1 + 0.002 * (bbox[3] - bbox[1])

            m.color.a = 0.85
            m.color.r, m.color.g, m.color.b = color
            marker_array.markers.append(m)

        self.pub_markers.publish(marker_array)

    # ------------------------------
    def publish_debug_image(self, frame, detections):
        for i, (bbox, _) in enumerate(detections):
            x1, y1, x2, y2, _ = bbox
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, f"Obj{i}", (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        debug_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.pub_debug_image.publish(debug_msg)


# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = MultiObjectYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
