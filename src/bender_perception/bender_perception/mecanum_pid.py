#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped, PointStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from control_msgs.action import GripperCommand
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    PlanningScene, CollisionObject, AttachedCollisionObject,
    RobotState, PlanningSceneWorld
)
from moveit_msgs.srv import ApplyPlanningScene
import math


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class VisualPickPID(Node):
    def __init__(self):
        super().__init__('pick_and_place_visual_pid')

        # === Parámetros ===
        self.declare_parameter("target_y_left",  0.27)
        self.declare_parameter("target_y_right", -0.27)
        self.declare_parameter("grasp_dist",     0.25)
        self.declare_parameter("close_dist",     0.22)
        self.declare_parameter("min_z",          0.45)
        self.declare_parameter("max_z",          0.95)
        self.declare_parameter("vx_limit",       0.12)
        self.declare_parameter("wz_limit",       0.18)
        self.declare_parameter("kp_x",           0.45)
        self.declare_parameter("kp_y",           1.1)
        self.declare_parameter("gripper_effort", 10.0)
        self.declare_parameter("gripper_open",   0.8)
        self.declare_parameter("gripper_close",  0.0)

        # Timings
        self.declare_parameter("grasp_settle_time",   0.8)
        self.declare_parameter("pre_close_move_time", 0.6)
        self.declare_parameter("lift_duration",       1.0)

        # === MoveIt Planning Scene ===
        self.declare_parameter("obj_size_x", 0.06)
        self.declare_parameter("obj_size_y", 0.06)
        self.declare_parameter("obj_size_z", 0.06)
        self.declare_parameter("obj_z_offset", -0.06)   # desplaza la caja respecto al punto 3D
        self.declare_parameter("world_frame", "base_link")
        self.declare_parameter("object_id", "target_object")
        self.declare_parameter("gripper_link_left",  "g1l_1")
        self.declare_parameter("gripper_link_right", "g1r_1")
        self.declare_parameter("touch_links_left",  "g1l_1,g2la_1,g2lb_1")
        self.declare_parameter("touch_links_right", "g1r_1,g2ra_1,g2rb_1")

        # === Estado ===
        # idle / opening_gripper / approaching / waiting_grasp / moving_to_grasp /
        # closing_gripper / attaching / lifting / done
        self.state = "idle"
        self.object_point = None
        self.closest_arm = None
        self.current_x = 0.0
        self.current_yaw = 0.0
        self.start_x = None
        self.last_seen = None
        self.pending_timer = None

        # === ROS Interfaces ===
        self.pub_ref = self.create_publisher(TwistStamped, '/mecanum_base_controller/reference', 10)
        self.pub_pose_left = self.create_publisher(PoseStamped, '/target_pose_left', 10)
        self.pub_pose_right = self.create_publisher(PoseStamped, '/target_pose_right', 10)

        self.sub_obj = self.create_subscription(PointStamped, '/object/point', self.obj_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/mecanum_base_controller/odometry', self.odom_cb, 10)
        self.sub_trigger = self.create_subscription(Bool, '/auto_grasp_trigger', self.trigger_cb, 10)

        self.gripper_left = ActionClient(self, GripperCommand, '/left_gripper_controller/gripper_cmd')
        self.gripper_right = ActionClient(self, GripperCommand, '/right_gripper_controller/gripper_cmd')

        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not self.scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("⚠️ Servicio /apply_planning_scene no disponible aún.")

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("🤖 VisualPickPID con offset de colisión y orden de grasp FIX ✅")

    # ------------------------------
    def odom_cb(self, msg):
        self.current_yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.current_x = msg.pose.pose.position.x

    def obj_cb(self, msg):
        if not all(map(math.isfinite, [msg.point.x, msg.point.y, msg.point.z])):
            return
        self.object_point = msg.point
        self.last_seen = self.get_clock().now().seconds_nanoseconds()[0]

        if self.state == "idle":
            self.closest_arm = "left" if msg.point.y > 0 else "right"
            self.state = "opening_gripper"
            self.get_logger().info(f"🎯 Objeto detectado — abriendo gripper {self.closest_arm}")
            self.send_gripper(self.closest_arm, "open", next_state="approaching")

    def trigger_cb(self, msg):
        if msg.data and self.state == "approaching":
            self.publish_stop()
            self.get_logger().info("✋ Contacto detectado — preparando grasp controlado")
            self.state = "waiting_grasp"
            self._start_one_shot(self.get_parameter("grasp_settle_time").value, self._transition_to_grasp)

    # ------------------------------
    def _start_one_shot(self, duration, cb):
        """Timer one-shot seguro."""
        if self.pending_timer:
            try:
                self.pending_timer.cancel()
            except Exception:
                pass
            self.pending_timer = None

        def _wrapper():
            # cancelar este mismo timer para que sea one-shot
            if self.pending_timer:
                try:
                    self.pending_timer.cancel()
                except Exception:
                    pass
                self.pending_timer = None
            cb()

        self.pending_timer = self.create_timer(float(duration), _wrapper)

    # ------------------------------
    def _transition_to_grasp(self):
        if self.object_point is None:
            return
        self.state = "moving_to_grasp"

        # 1) Añadir el objeto al mundo (ligeramente desplazado en Z para evitar colisiones antes de tiempo)
        self._scene_add_or_update_world_object(self.object_point)

        # 2) Mover a pose de grasp
        self.execute_grasp(self.object_point)
        self.get_logger().info("🤏 Moviendo brazo a posición final de grasp...")
        # 3) Esperar que llegue a la pose y cerrar
        self._start_one_shot(self.get_parameter("pre_close_move_time").value, self._close_then_attach)

    def _close_then_attach(self):
        """Cerrar primero, luego adjuntar, y entonces levantar."""
        self.state = "closing_gripper"
        self.get_logger().info("🤏 Cerrando gripper...")
        # Cuando llegue el RESULT, pasamos a 'attaching'
        self.send_gripper(self.closest_arm, "close", next_state="attaching")
        # Además, disparamos un pequeño margen para adjuntar y levantar (por si el controlador no bloquea bien)
        self._start_one_shot(0.8, self._scene_attach_and_lift)

    def _scene_attach_and_lift(self):
        if self.state != "attaching":
            # si ya estamos en attaching por el RESULT, igual hacemos attach
            self.state = "attaching"
        self._scene_attach_object(self.closest_arm)
        self._start_lift()

    def _start_lift(self):
        self.lift_object()
        self.state = "lifting"
        self.get_logger().info("⬆️ Levantando objeto...")
        self._start_one_shot(self.get_parameter("lift_duration").value, self._finish_lift)

    def _finish_lift(self):
        self.state = "done"
        self.start_x = self.current_x
        self.get_logger().info("✅ Levantado — iniciando retroceso")

    # ------------------------------
    def control_loop(self):
        if self.state in ["idle", "opening_gripper", "waiting_grasp", "moving_to_grasp",
                          "closing_gripper", "attaching", "lifting"]:
            return

        if self.object_point is None:
            self.publish_stop()
            return

        if self.state not in ["done"] and self.last_seen and \
           self.get_clock().now().seconds_nanoseconds()[0] - self.last_seen > 2.0:
            self.get_logger().warn("⚠️ Objeto perdido — reiniciando rutina.")
            self._scene_detach_and_remove()
            self.reset_routine()
            return

        if self.state == "approaching":
            self._approach_behavior()
        elif self.state == "done":
            self._backoff_behavior()

    # ------------------------------
    def _approach_behavior(self):
        p = self.object_point
        if p is None:
            return
        # Safety fallback: if object is already within grasp distance (and in
        # front of the robot), stop and proceed to the controlled grasp
        # (fallback to trigger sensor). Ignore negative p.x (behind robot).
        grasp_dist = self.get_parameter("grasp_dist").value + 0.3
        try:
            px_val = float(p.x)
            py_val = float(p.y)
            pz_val = float(p.z)
        except Exception:
            px_val = p.x
            py_val = p.y
            pz_val = p.z

        # Log coordinates for debugging (helpful to diagnose wrong TFs/frames)
        self.get_logger().debug(f"object_point (x,y,z) = {px_val:.3f},{py_val:.3f},{pz_val:.3f}")
        print(f"px_val: {px_val}, grasp_dist: {grasp_dist}")

        # Only trigger fallback if object is in front (positive x) and within grasp distance
        # NOTE: do NOT close the gripper here. Instead stop the base and start the
        # arm movement to the grasp pose (moving_to_grasp) so the arm moves first
        # and only after the arm reaches the pose we close the gripper.
        if px_val > 0.0 and px_val <= grasp_dist:
            self.publish_stop()
            self.get_logger().info("✋ Distancia de grasp alcanzada — deteniendo base y moviendo brazo al grasp")
            # transition immediately to moving_to_grasp (this will call execute_grasp
            # and schedule the pre_close_move_time before closing the gripper).
            self._transition_to_grasp()
            return
        target_y = self.get_parameter("target_y_left").value if self.closest_arm == "left" else self.get_parameter("target_y_right").value
        close_dist = self.get_parameter("close_dist").value
        kp_x = self.get_parameter("kp_x").value
        kp_y = self.get_parameter("kp_y").value
        vx_limit = self.get_parameter("vx_limit").value
        wz_limit = self.get_parameter("wz_limit").value

        vx = clamp(kp_x * (p.x - close_dist), -vx_limit, vx_limit)
        wz = clamp(kp_y * (target_y - p.y), -wz_limit, wz_limit)

        cmd = TwistStamped()
        cmd.twist.linear.x = vx
        cmd.twist.angular.z = -wz
        self.pub_ref.publish(cmd)

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = clamp(p.x, 0.3, 0.45)
        pose.pose.position.y = target_y
        pose.pose.position.z = clamp(p.z, self.get_parameter("min_z").value, self.get_parameter("max_z").value)
        pose.pose.orientation.y = 0.707
        pose.pose.orientation.w = -0.707
        pub = self.pub_pose_left if self.closest_arm == "left" else self.pub_pose_right
        pub.publish(pose)

    def _backoff_behavior(self):
        if self.start_x is None:
            self.start_x = self.current_x
        if abs(self.current_x - self.start_x) < 1.0:
            cmd = TwistStamped()
            cmd.twist.linear.x = -0.1
            self.pub_ref.publish(cmd)
        else:
            self.publish_stop()
            self._scene_detach_and_remove()
            self.get_logger().info("🏁 Retroceso completado — rutina finalizada.")
            self.reset_routine()

    # ------------------------------
    def execute_grasp(self, p):
        x_final = clamp(p.x, 0.3, 0.45)
        y_final = self.get_parameter("target_y_left").value if self.closest_arm == "left" else self.get_parameter("target_y_right").value
        z_final = clamp(p.z - 0.05, 0.40, 0.90)  # baja 5 cm para contactar

        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose.position.x = x_final
        ps.pose.position.y = y_final
        ps.pose.position.z = z_final
        ps.pose.orientation.y = 0.707
        ps.pose.orientation.w = -0.707

        pub = self.pub_pose_left if self.closest_arm == "left" else self.pub_pose_right
        pub.publish(ps)

    def lift_object(self):
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose.position.x = 0.3
        ps.pose.position.y = self.get_parameter("target_y_left").value if self.closest_arm == "left" else self.get_parameter("target_y_right").value
        ps.pose.position.z = 1.0
        ps.pose.orientation.y = 0.707
        ps.pose.orientation.w = -0.707
        pub = self.pub_pose_left if self.closest_arm == "left" else self.pub_pose_right
        pub.publish(ps)

    # ------------------------------
    # === MOVEIT HELPERS ===
    def _scene_add_or_update_world_object(self, point):
        """Añade/actualiza una caja en el mundo (frame world_frame) cerca del punto 3D."""
        if not self.scene_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("⚠️ apply_planning_scene no disponible")
            return

        obj_id = self.get_parameter("object_id").value
        world_frame = self.get_parameter("world_frame").value
        sx = float(self.get_parameter("obj_size_x").value)
        sy = float(self.get_parameter("obj_size_y").value)
        sz = float(self.get_parameter("obj_size_z").value)
        z_off = float(self.get_parameter("obj_z_offset").value)

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [sx, sy, sz]

        pose = Pose()
        pose.position.x = float(point.x)
        pose.position.y = float(point.y)
        # centro de la caja con un pequeño offset para evitar colisión inmediata
        pose.position.z = float(point.z + z_off)
        pose.orientation.w = 1.0

        co = CollisionObject()
        co.id = obj_id
        co.header.frame_id = world_frame
        co.primitives = [box]
        co.primitive_poses = [pose]
        co.operation = CollisionObject.ADD

        world = PlanningSceneWorld()
        world.collision_objects = [co]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world = world

        req = ApplyPlanningScene.Request(scene=scene)
        self.scene_client.call_async(req)
        self.get_logger().info(f"📦 Objeto '{obj_id}' añadido/actualizado en escena en "
                               f"{pose.position.x:.2f},{pose.position.y:.2f},{pose.position.z:.2f}")

    def _scene_attach_object(self, side):
        if not self.scene_client.wait_for_service(timeout_sec=1.0):
            return
        obj_id = self.get_parameter("object_id").value
        link = self.get_parameter(f"gripper_link_{side}").value
        touch_links = [t.strip() for t in self.get_parameter(f"touch_links_{side}").value.split(",")]

        # quitar del mundo
        remove_world = CollisionObject()
        remove_world.id = obj_id
        remove_world.operation = CollisionObject.REMOVE

        # adjuntar
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object = CollisionObject()
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = touch_links

        rs = RobotState()
        rs.attached_collision_objects = [aco]

        world = PlanningSceneWorld()
        world.collision_objects = [remove_world]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world = world
        scene.robot_state = rs

        req = ApplyPlanningScene.Request(scene=scene)
        self.scene_client.call_async(req)
        self.get_logger().info(f"🔗 Objeto '{obj_id}' ATTACHED a '{link}' con touch_links={touch_links}")

    def _scene_detach_and_remove(self):
        if not self.scene_client.wait_for_service(timeout_sec=1.0):
            return
        obj_id = self.get_parameter("object_id").value

        aco = AttachedCollisionObject()
        aco.object = CollisionObject()
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.REMOVE

        remove_world = CollisionObject()
        remove_world.id = obj_id
        remove_world.operation = CollisionObject.REMOVE

        rs = RobotState()
        rs.attached_collision_objects = [aco]

        world = PlanningSceneWorld()
        world.collision_objects = [remove_world]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world = world
        scene.robot_state = rs

        req = ApplyPlanningScene.Request(scene=scene)
        self.scene_client.call_async(req)

    # ------------------------------
    def send_gripper(self, side, action, next_state=None):
        client = self.gripper_left if side == "left" else self.gripper_right
        goal = GripperCommand.Goal()
        goal.command.max_effort = float(self.get_parameter("gripper_effort").value)
        goal.command.position = (
            float(self.get_parameter("gripper_open").value) if action == "open"
            else float(self.get_parameter("gripper_close").value)
        )

        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"{side} gripper: servidor no disponible")
            self.reset_routine()
            return

        self.get_logger().info(f"🤖 Gripper {side}: {action}")
        send_future = client.send_goal_async(goal)

        def _on_goal_response(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn("❌ Gripper: goal rechazado")
                self.reset_routine()
                return

            result_future = goal_handle.get_result_async()

            def _on_result(_):
                self.get_logger().info("✅ Gripper: RESULT recibido")
                if next_state:
                    self.state = next_state
                    self.get_logger().info(f"➡️ Estado cambiado a '{next_state}'")

            result_future.add_done_callback(_on_result)

        send_future.add_done_callback(_on_goal_response)

    # ------------------------------
    def publish_stop(self):
        stop = TwistStamped()
        stop.twist.linear.x = 0.0
        stop.twist.angular.z = 0.0
        self.pub_ref.publish(stop)

    def reset_routine(self):
        if self.pending_timer:
            try:
                self.pending_timer.cancel()
            except Exception:
                pass
            self.pending_timer = None
        self.publish_stop()
        self._scene_detach_and_remove()
        self.state = "idle"
        self.closest_arm = None
        self.object_point = None
        self.start_x = None
        self.get_logger().info("🔄 Rutina reseteada y lista para siguiente objeto.")


def main(args=None):
    rclpy.init(args=args)
    node = VisualPickPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
