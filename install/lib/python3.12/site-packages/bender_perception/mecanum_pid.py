#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped, PointStamped, PoseStamped, Pose, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from control_msgs.action import GripperCommand
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    PlanningScene, CollisionObject, AttachedCollisionObject,
    RobotState, PlanningSceneWorld
)
from moveit_msgs.msg import AllowedCollisionEntry
from moveit_msgs.srv import ApplyPlanningScene
import math
import tf2_ros
import time

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def euler_to_quaternion(yaw, pitch, roll):
    """Convert Euler angles (yaw, pitch, roll) to quaternion (x,y,z,w).

    Angles are in radians. Follows convention: roll around X, pitch around Y, yaw around Z.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


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
        self.declare_parameter("lift_duration",       2.0)
        # Parámetros para pre-lift (estado que eleva el brazo y espera)
        self.declare_parameter("pre_lift_hold_time", 15.0)
        self.declare_parameter("pre_lift_send_count", 150)
        self.declare_parameter("pre_lift_send_interval", 0.1)

        # === MoveIt Planning Scene ===
        self.declare_parameter("obj_size_x", 0.06)
        self.declare_parameter("obj_size_y", 0.06)
        self.declare_parameter("obj_size_z", 0.06)
        self.declare_parameter("obj_z_offset", -0.06)   # desplaza la caja respecto al punto 3D
        # desplaza en X la caja respecto al punto detectado (negativo = hacia atrás)
        self.declare_parameter("obj_x_offset", 0.08)
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
        self.object_point_scene = None
        self.closest_arm = None
        self.current_x = 0.0
        self.current_yaw = 0.0
        self.start_x = None
        self.last_seen = None
        self.pending_timer = None
        # monitor para transición automática desde 'approaching' a 'waiting_grasp'
        self._approach_publish_count = 0
        self._approach_timeout = False
        self._approach_timeout_timer = None
        self._approach_monitor_active = False
        # flag que indica si se recibió el trigger de confirmación/override
        # cuando se quiere autorizar continuar tras el cierre del gripper
        self._auto_grasp_triggered = False
        # flag que indica que estamos esperando el trigger para proceder al attach
        self._waiting_for_attach_trigger = False
        # timer one-shot usado mientras esperamos el /auto_grasp_trigger tras el close
        self._attach_wait_timer = None

        # === ROS Interfaces ===
        self.pub_ref = self.create_publisher(TwistStamped, '/mecanum_base_controller/reference', 10)
        self.pub_pose_left = self.create_publisher(PoseStamped, '/target_pose_left', 10)
        self.pub_pose_right = self.create_publisher(PoseStamped, '/target_pose_right', 10)

        self.sub_obj = self.create_subscription(PointStamped, '/object/point', self.obj_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/mecanum_base_controller/odometry', self.odom_cb, 10)
        self.sub_trigger = self.create_subscription(Bool, '/auto_grasp_trigger', self.trigger_cb, 10)

        self.gripper_left = ActionClient(self, GripperCommand, '/left_gripper_controller/gripper_cmd')
        self.gripper_right = ActionClient(self, GripperCommand, '/right_gripper_controller/gripper_cmd')
        # almacenar último GoalHandle por gripper para permitir cancelación
        self._last_gripper_goal_left = None
        self._last_gripper_goal_right = None

        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not self.scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("⚠️ Servicio /apply_planning_scene no disponible aún.")

        # TF2 listener to read end-effector poses
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("🤖 VisualPickPID con offset de colisión y orden de grasp FIX ✅")

        # Hacer una limpieza inicial de cualquier objeto residual en la PlanningScene
        # programada con un timer corto para evitar race con el arranque de servicios
        def _initial_cleanup():
            try:
                self._scene_detach_and_remove()
            except Exception as e:
                self.get_logger().debug(f"Inicial: no se pudo limpiar planning scene: {e}")
            try:
                # cancelar este timer one-shot
                self._init_timer.cancel()
            except Exception:
                pass
            # marcar que la limpieza inicial ha terminado para aceptar detecciones
            try:
                self._initial_cleanup_done = True
            except Exception:
                pass

        # crear timer one-shot para limpieza inicial
        # flag para evitar aceptar detecciones tempranas mientras se limpia la escena
        self._initial_cleanup_done = False
        self._init_timer = self.create_timer(0.5, _initial_cleanup)

    # ------------------------------
    def odom_cb(self, msg):
        self.current_yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.current_x = msg.pose.pose.position.x

    def obj_cb(self, msg):
        if not all(map(math.isfinite, [msg.point.x, msg.point.y, msg.point.z])):
            return
        # Ignorar detecciones tempranas hasta que la limpieza inicial haya terminado
        if not getattr(self, '_initial_cleanup_done', True):
            self.get_logger().debug("Ignorando detección temprana hasta completar limpieza inicial.")
            return
        self.object_point = msg.point
        self.last_seen = self.get_clock().now().seconds_nanoseconds()[0]

        if self.state == "idle":
            self.closest_arm = "left" if msg.point.y > 0 else "right"
            # pasar a estado pre_lift: levantar brazo a postura segura y mantener un tiempo
            self.state = "pre_lift"
            # Quitar cualquier objeto anterior y añadir el nuevo a la PlanningScene
            try:
                self._scene_detach_and_remove()
            except Exception:
                pass
            try:
                self._scene_add_or_update_world_object(self.object_point)
                # guardar la versión desplazada que representamos en la PlanningScene
                try:
                    x_off = float(self.get_parameter("obj_x_offset").value)
                    z_off = float(self.get_parameter("obj_z_offset").value)
                    p_scene = Point()
                    p_scene.x = float(msg.point.x + x_off)
                    p_scene.y = float(msg.point.y)
                    p_scene.z = float(msg.point.z + z_off)
                    self.object_point_scene = p_scene
                except Exception:
                    self.object_point_scene = None
            except Exception as e:
                self.get_logger().warn(f"⚠️ Error añadiendo objeto a PlanningScene: {e}")

            # Log object position and current end-effector pose (if available)
            ee_pose = self._get_end_effector_pose(self.closest_arm)
            if ee_pose is not None:
                self.get_logger().info(f"🎯 Objeto detectado — punto: ({msg.point.x:.3f},{msg.point.y:.3f},{msg.point.z:.3f}) | ee: ({ee_pose.pose.position.x:.3f},{ee_pose.pose.position.y:.3f},{ee_pose.pose.position.z:.3f}) — abriendo gripper {self.closest_arm}")
            else:
                self.get_logger().info(f"🎯 Objeto detectado — punto: ({msg.point.x:.3f},{msg.point.y:.3f},{msg.point.z:.3f}) — abriendo gripper {self.closest_arm}")
            # programar la rutina de pre-lift inmediatamente (no bloquear el callback)
            self._start_one_shot(0.01, self._pre_lift)

    def trigger_cb(self, msg):
        if msg.data:
            # Si estamos esperando trigger tras el cierre, proceder al attach
            if getattr(self, '_waiting_for_attach_trigger', False):
                self.get_logger().info("✋ auto_grasp_trigger recibido durante wait-for-attach — procediendo a attach")
                try:
                    self._waiting_for_attach_trigger = False
                except Exception:
                    pass
                try:
                    # cancelar timer de espera si estaba activo
                    try:
                        if self._attach_wait_timer:
                            try:
                                self._attach_wait_timer.cancel()
                            except Exception:
                                pass
                            self._attach_wait_timer = None
                    except Exception:
                        pass
                    # realizar attach y lift ahora que tenemos autorización
                    self._scene_attach_and_lift()
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Error al proceder al attach tras trigger: {e}")
                return

            # En otras fases, este trigger actúa como autorización/interrupt: marcar y cancelar cierres
            self.get_logger().info("✋ auto_grasp_trigger recibido — autorizando continuación/cancel")
            try:
                self._auto_grasp_triggered = True
            except Exception:
                pass
            try:
                self.cancel_gripper('left')
            except Exception:
                pass
            try:
                self.cancel_gripper('right')
            except Exception:
                pass
            # Additionally: when an auto-grasp trigger is received, remove the
            # corresponding object from the PlanningScene to avoid stale world
            # collisions. This is desired when the grasping pipeline hands the
            # object to the robot or the perception pose is no longer needed.
            try:
                if getattr(self, 'object_point', None) is not None or getattr(self, 'object_point_scene', None) is not None:
                    self.get_logger().info("🗑️ auto_grasp_trigger: borrando objeto de la PlanningScene")
                    try:
                        self._scene_detach_and_remove()
                    except Exception as e:
                        self.get_logger().warn(f"⚠️ Error al borrar objeto tras trigger: {e}")
                    # clear local references
                    try:
                        self.object_point = None
                        self.object_point_scene = None
                    except Exception:
                        pass
            except Exception:
                pass
            return

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

    # ---- Approach monitor helpers ----
    def _start_approach_monitor(self):
        # reset counters and start a one-shot timeout (20s)
        try:
            if self._approach_monitor_active:
                self._cancel_approach_monitor()
            self._approach_publish_count = 0
            self._approach_timeout = False
            self._approach_monitor_active = True
            # crear timer one-shot de 20s para habilitar la condición temporal
            def _approach_timeout_cb():
                self._approach_timeout = True
                # cancelar este timer
                try:
                    if self._approach_timeout_timer:
                        self._approach_timeout_timer.cancel()
                except Exception:
                    pass

            self._approach_timeout_timer = self.create_timer(20.0, _approach_timeout_cb)
            self.get_logger().info("⏱️ Approach monitor: iniciada (esperando ~20s y >=4 publicaciones para auto-wait)")
        except Exception as e:
            self.get_logger().debug(f"Error iniciando approach monitor: {e}")

    def _cancel_approach_monitor(self):
        try:
            if self._approach_timeout_timer:
                try:
                    self._approach_timeout_timer.cancel()
                except Exception:
                    pass
                self._approach_timeout_timer = None
            self._approach_publish_count = 0
            self._approach_timeout = False
            self._approach_monitor_active = False
        except Exception:
            pass

    def _maybe_transition_to_waiting(self):
        # invoked when both conditions met
        try:
            if self.state != 'approaching':
                return
            self.get_logger().info("✋ Auto: condicion cumplida -> entrando en waiting_grasp")
            self.publish_stop()
            self.state = 'waiting_grasp'
            self._start_one_shot(self.get_parameter('grasp_settle_time').value, self._transition_to_grasp)
            # cancel monitor
            self._cancel_approach_monitor()
        except Exception as e:
            self.get_logger().warn(f"⚠️ Error en auto-transition to waiting: {e}")

    def _get_end_effector_pose(self, side):
        """Try to get the current PoseStamped of the gripper link via TF2.

        Returns PoseStamped or None if not available.
        """
        try:
            link = self.get_parameter(f"gripper_link_{side}").value
            world_frame = self.get_parameter("world_frame").value
            # lookup transform from world_frame -> link
            trans = self.tf_buffer.lookup_transform(world_frame, link, rclpy.time.Time())
            ps = PoseStamped()
            ps.header.stamp = trans.header.stamp
            ps.header.frame_id = world_frame
            ps.pose.position.x = trans.transform.translation.x
            ps.pose.position.y = trans.transform.translation.y
            ps.pose.position.z = trans.transform.translation.z
            ps.pose.orientation = trans.transform.rotation
            return ps
        except Exception as e:
            # Could be LookupException, ExtrapolationException, ConnectivityException
            self.get_logger().debug(f"TF lookup failed for ee '{side}': {e}")
            return None

    def _pre_lift(self):
        """Eleva el brazo a una postura segura y mantiene la pose antes de continuar.

        Publica varias veces la pose (x=0.3, y=pre_target_y, z=1.1) y luego abre el gripper
        para continuar con el flujo ('approaching').
        """
        try:
            if self.closest_arm is None:
                self.get_logger().warn("⚠️ pre_lift: closest_arm desconocido")
                return

            pre_target_y = -0.5 if self.closest_arm == "left" else 0.5
            ps = PoseStamped()
            ps.header.frame_id = self.get_parameter("world_frame").value
            ps.pose.position.x = 0.3
            ps.pose.position.y = pre_target_y
            ps.pose.position.z = 1.1
            # Compute a shoulder-relative yaw so the wrist faces along the arm
            # regardless of whether it's left or right. Use shoulder y offsets
            # (left: -0.3, right: +0.3). For the left arm we invert the yaw so
            # it becomes the negative of the right-arm yaw as requested.
            shoulder_y = -0.3 if self.closest_arm == 'left' else 0.3
            shoulder_x = 0.0
            dx = float(ps.pose.position.x) - shoulder_x
            dy = float(ps.pose.position.y) - shoulder_y
            forced_yaw = -math.atan2(dy, dx)
            if pre_target_y < self.target_y_left and self.closest_arm == "left":
                forced_yaw = -forced_yaw  # invertir para brazo izquierdo
            if pre_target_y > self.target_y_right and self.closest_arm == "right":
                forced_yaw = -forced_yaw  # invertir para brazo derecho

            # Force palm parallel to ground
            forced_pitch = -math.pi / 2.0
            forced_roll = 0.0
            qx, qy, qz, qw = euler_to_quaternion(forced_yaw, forced_pitch, forced_roll)
            ps.pose.orientation.x = float(qx)
            ps.pose.orientation.y = float(qy)
            ps.pose.orientation.z = float(qz)
            ps.pose.orientation.w = float(qw)

            # Log the exact pose that will be published for debugging
            try:
                self.get_logger().info(
                    f"[pre_lift] -> side={self.closest_arm} pose=(x={ps.pose.position.x:.3f}, y={pre_target_y:.3f}, z={ps.pose.position.z:.3f}) "
                    f"euler=(yaw={forced_yaw:.4f} rad/{math.degrees(forced_yaw):.1f}deg, pitch={forced_pitch:.4f} rad) "
                    f"quat=(x={qx:.4f}, y={qy:.4f}, z={qz:.4f}, w={qw:.4f})"
                )
            except Exception:
                pass

            pub = self.pub_pose_left if self.closest_arm == "left" else self.pub_pose_right

            send_count = int(self.get_parameter("pre_lift_send_count").value)
            interval = float(self.get_parameter("pre_lift_send_interval").value)

            self.get_logger().info(f"⤴️ pre_lift: elevando brazo a x=0.3,y={pre_target_y:.2f},z=1.1 durante ~{send_count*interval:.1f}s (trayectoria cartesiana)")

            # Intentar trayectoria cartesiana: interpolar desde la pose actual del efector final
            start_ps = self._get_end_effector_pose(self.closest_arm)
            if start_ps is None:
                # no tenemos pose inicial, publicar objetivo varias veces como fallback
                for _ in range(max(1, send_count)):
                    pub.publish(ps)
                    try:
                        time.sleep(interval)
                    except Exception:
                        pass
            else:
                sx = start_ps.pose.position.x
                sy = start_ps.pose.position.y
                sz = start_ps.pose.position.z
                tx = ps.pose.position.x
                ty = ps.pose.position.y
                tz = ps.pose.position.z
                # mantener la orientación objetivo constante (no hacemos slerp aquí)
                for i in range(1, max(1, send_count) + 1):
                    a = float(i) / float(max(1, send_count))
                    step = PoseStamped()
                    step.header.frame_id = ps.header.frame_id
                    step.pose.position.x = sx * (1.0 - a) + tx * a
                    step.pose.position.y = sy * (1.0 - a) + ty * a
                    step.pose.position.z = sz * (1.0 - a) + tz * a
                    step.pose.orientation = ps.pose.orientation
                    pub.publish(step)
                    try:
                        time.sleep(interval)
                    except Exception:
                        pass

            # Tras mantener la pose, abrir gripper y seguir con approaching
            self.state = "opening_gripper"
            self.send_gripper(self.closest_arm, "open", next_state="approaching")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Error en pre_lift: {e}")

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
        # Indicamos que esperamos el trigger para proceder al attach
        self._waiting_for_attach_trigger = True
        # Enviamos comando de cierre; el attach NO se ejecutará automáticamente
        # hasta recibir /auto_grasp_trigger (ver send_gripper + trigger_cb)
        self.send_gripper(self.closest_arm, "close", next_state="attaching")
        self.get_logger().info("⏳ Gripper cerrado: esperando /auto_grasp_trigger para proceder al attach...")

        # arrancar timer one-shot de 7s: si expira sin recibir el trigger, resetear por seguridad
        try:
            # cancelar timer previo si existe
            if self._attach_wait_timer:
                try:
                    self._attach_wait_timer.cancel()
                except Exception:
                    pass
                self._attach_wait_timer = None

            def _attach_timeout_cb():
                # cancelar este timer (hacerlo one-shot)
                try:
                    if self._attach_wait_timer:
                        self._attach_wait_timer.cancel()
                except Exception:
                    pass
                self._attach_wait_timer = None
                try:
                    self.get_logger().warn("⚠️ Timeout (7s) esperando /auto_grasp_trigger — reseteando rutina por seguridad")
                except Exception:
                    pass
                try:
                    self.reset_routine()
                except Exception:
                    pass

            self._attach_wait_timer = self.create_timer(7.0, _attach_timeout_cb)
        except Exception:
            pass

    def _scene_attach_and_lift(self):
        if self.state != "attaching":
            # si ya estamos en attaching por el RESULT, igual hacemos attach
            self.state = "attaching"
        # asegurar que cualquier timer de espera se cancele al ejecutar attach
        try:
            if self._attach_wait_timer:
                try:
                    self._attach_wait_timer.cancel()
                except Exception:
                    pass
                self._attach_wait_timer = None
        except Exception:
            pass
        self._scene_attach_object(self.closest_arm)
        # dar un pequeño tiempo para que MoveIt procese el attach y actualice la PlanningScene
        try:
            time.sleep(0.25)
        except Exception:
            pass
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
        # preferir la pose corregida en la PlanningScene si existe
        p = self.object_point_scene if getattr(self, 'object_point_scene', None) is not None else self.object_point
        target_y = self.get_parameter("target_y_left").value if self.closest_arm == "left" else self.get_parameter("target_y_right").value
        
        # Estos mueven la base del robot, no nos interesa
        # close_dist = self.get_parameter("close_dist").value
        # kp_x = self.get_parameter("kp_x").value
        # kp_y = self.get_parameter("kp_y").value
        # vx_limit = self.get_parameter("vx_limit").value
        # wz_limit = self.get_parameter("wz_limit").value

        # vx = clamp(kp_x * (p.x - close_dist), -vx_limit, vx_limit)
        # wz = clamp(kp_y * (target_y - p.y), -wz_limit, wz_limit)

        #cmd = TwistStamped()
        #cmd.twist.linear.x = vx
        #cmd.twist.angular.z = -wz
        #self.pub_ref.publish(cmd)

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = clamp(p.x - 0.1 + float(self.get_parameter("obj_x_offset").value), 0.3, 0.5)
        pose.pose.position.y = clamp(p.y, -0.1 + target_y, 0.1 + target_y)
        pose.pose.position.z = clamp(p.z - 0.02, self.get_parameter("min_z").value, self.get_parameter("max_z").value)
        # Use Euler -> quaternion for clarity: yaw=0, pitch=pi/2 (palm parallel), roll=0
        qx, qy, qz, qw = euler_to_quaternion(0.0, -math.pi/2.0, 0.0)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        pub = self.pub_pose_left if self.closest_arm == "left" else self.pub_pose_right
        pub.publish(pose)
        # contar publicaciones de approach para la transición automática
        try:
            if self._approach_monitor_active:
                self._approach_publish_count += 1
                self.get_logger().info(f"Target pose: {pose.pose.position.x},{pose.pose.position.y},{pose.pose.position.z}")
                if self._approach_publish_count >= 4 and self._approach_timeout:
                    self._maybe_transition_to_waiting()
        except Exception:
            pass

    def _backoff_behavior(self):
        if self.start_x is None:
            self.start_x = self.current_x
        if abs(self.current_x - self.start_x) < 1.0:
            pass
            # Estos mueven la base del robot, no nos interesa
            # cmd = TwistStamped()
            # cmd.twist.linear.x = -0.1
            # self.pub_ref.publish(cmd)
        else:
            self.publish_stop()
            self._scene_detach_and_remove()
            self.get_logger().info("🏁 Retroceso completado — rutina finalizada.")
            self.reset_routine()

    # ------------------------------
    def execute_grasp(self, p):
        target_y = self.get_parameter("target_y_left").value if self.closest_arm == "left" else self.get_parameter("target_y_right").value
        x_final = clamp(p.x - 0.06 + float(self.get_parameter("obj_x_offset").value), 0.3, 0.45)
        y_final = clamp(p.y, -0.1 + target_y, 0.1 + target_y)
        z_final = clamp(p.z - 0.1, 0.40, 0.90)  # baja 5 cm para contactar

        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose.position.x = x_final
        ps.pose.position.y = y_final
        ps.pose.position.z = z_final
        # Use Euler -> quaternion for clarity: yaw=0, pitch=pi/2 (palm parallel), roll=0
        qx, qy, qz, qw = euler_to_quaternion(0.0, -math.pi/2.0, 0.0)
        ps.pose.orientation.x = float(qx)
        ps.pose.orientation.y = float(qy)
        ps.pose.orientation.z = float(qz)
        ps.pose.orientation.w = float(qw)

        pub = self.pub_pose_left if self.closest_arm == "left" else self.pub_pose_right
        for _ in range(3):  # publicar varias veces para asegurar recepción
            pub.publish(ps)
            try:
                time.sleep(1)  # pequeño delay para asegurar que llega publicación
            except Exception:
                pass

    def lift_object(self):
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose.position.x = 0.3
        ps.pose.position.y = self.get_parameter("target_y_left").value if self.closest_arm == "left" else self.get_parameter("target_y_right").value
        ps.pose.position.z = 1.2
        # Use Euler -> quaternion for clarity: yaw=0, pitch=pi/2 (palm parallel), roll=0
        qx, qy, qz, qw = euler_to_quaternion(0.0, -math.pi/2.0, 0.0)
        ps.pose.orientation.x = float(qx)
        ps.pose.orientation.y = float(qy)
        ps.pose.orientation.z = float(qz)
        ps.pose.orientation.w = float(qw)
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
        # aplicar offset en X si se ha definido (por ejemplo -0.05 para 5 cm hacia atrás)
        x_off = float(self.get_parameter("obj_x_offset").value)
        pose.position.x = float(point.x + x_off)
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
        future = self.scene_client.call_async(req)
        # esperar respuesta del servicio para evitar condiciones de carrera
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            res = future.result()
            if res is None:
                self.get_logger().warn("⚠️ apply_planning_scene no respondió en tiempo (add/update)")
            else:
                self.get_logger().info(f"📦 Objeto '{obj_id}' añadido/actualizado en escena en "
                                       f"{pose.position.x:.2f},{pose.position.y:.2f},{pose.position.z:.2f}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Error esperando ApplyPlanningScene (add/update): {e}")

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
        future = self.scene_client.call_async(req)
        # esperar respuesta del servicio para evitar condiciones de carrera
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            res = future.result()
            if res is None:
                self.get_logger().warn("⚠️ apply_planning_scene no respondió en tiempo (attach)")
            else:
                self.get_logger().info(f"🔗 Objeto '{obj_id}' ATTACHED a '{link}' con touch_links={touch_links}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Error esperando ApplyPlanningScene (attach): {e}")

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
        future = self.scene_client.call_async(req)
        # esperar respuesta del servicio para evitar condiciones de carrera
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            res = future.result()
            if res is None:
                self.get_logger().warn("⚠️ apply_planning_scene no respondió en tiempo (detach/remove)")
            else:
                # nada específico a loggear aquí
                pass
        except Exception as e:
            self.get_logger().warn(f"⚠️ Error esperando ApplyPlanningScene (detach/remove): {e}")
        
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
            # guardar handle para permitir cancelación posterior
            try:
                if side == 'left':
                    self._last_gripper_goal_left = goal_handle
                else:
                    self._last_gripper_goal_right = goal_handle
            except Exception:
                pass
            if not goal_handle.accepted:
                self.get_logger().warn("❌ Gripper: goal rechazado")
                self.reset_routine()
                return

            result_future = goal_handle.get_result_async()

            def _on_result(_):
                self.get_logger().info("✅ Gripper: RESULT recibido")
                if next_state:
                    # Si acabamos de cerrar el gripper y la intención es adjuntar,
                    # no avanzamos automáticamente hasta que se reciba el /auto_grasp_trigger.
                    try:
                        if action == 'close' and next_state == 'attaching':
                            # Si ya se había recibido el trigger (raro, pero posible), consumirlo
                            if getattr(self, '_auto_grasp_triggered', False):
                                try:
                                    self._auto_grasp_triggered = False
                                except Exception:
                                    pass
                                # continuar normalmente y permitir la transición a attaching
                            else:
                                # Marcar que estamos esperando el trigger y no avanzar
                                try:
                                    self._waiting_for_attach_trigger = True
                                except Exception:
                                    pass
                                self.get_logger().info("⏳ Gripper cerrado — esperando /auto_grasp_trigger para proceder al attach")
                                # limpiar handle
                                try:
                                    if side == 'left':
                                        self._last_gripper_goal_left = None
                                    else:
                                        self._last_gripper_goal_right = None
                                except Exception:
                                    pass
                                return
                    except Exception:
                        pass

                    self.state = next_state
                    self.get_logger().info(f"➡️ Estado cambiado a '{next_state}'")
                    # si entramos en 'approaching', iniciar monitor para transición automática
                    try:
                        if next_state == 'approaching':
                            self._start_approach_monitor()
                        else:
                            # cancelar monitor si cambiamos de estado distinto a approaching
                            self._cancel_approach_monitor()
                    except Exception:
                        pass

                # limpiar handle tras recibir resultado
                try:
                    if side == 'left':
                        self._last_gripper_goal_left = None
                    else:
                        self._last_gripper_goal_right = None
                except Exception:
                    pass

            result_future.add_done_callback(_on_result)

        send_future.add_done_callback(_on_goal_response)

    def cancel_gripper(self, side):
        """Cancel any outstanding gripper goal for the given side ('left'|'right')."""
        try:
            gh = self._last_gripper_goal_left if side == 'left' else self._last_gripper_goal_right
        except Exception:
            gh = None
        if gh is None:
            self.get_logger().debug(f"No hay goal activo para el gripper {side} que cancelar.")
            return

        try:
            cancel_future = gh.cancel_goal_async()

            def _on_cancel(fut):
                try:
                    res = fut.result()
                    self.get_logger().info(f"🛑 Cancel gripper {side}: resultado={res}")
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Error cancelando gripper {side}: {e}")

                # limpiar handle local
                try:
                    if side == 'left':
                        self._last_gripper_goal_left = None
                    else:
                        self._last_gripper_goal_right = None
                except Exception:
                    pass

            cancel_future.add_done_callback(_on_cancel)
        except Exception as e:
            self.get_logger().warn(f"⚠️ No se pudo cancelar gripper {side}: {e}")

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
        # cancelar cualquier comando de gripper en curso
        try:
            self.cancel_gripper('left')
        except Exception:
            pass
        try:
            self.cancel_gripper('right')
        except Exception:
            pass
        self._scene_detach_and_remove()
        self.state = "idle"
        self.closest_arm = None
        self.object_point = None
        self.object_locked = False
        # limpiar flag de trigger para próximas ejecuciones
        try:
            self._auto_grasp_triggered = False
        except Exception:
            pass
        # cancelar timer de espera por trigger si existe
        try:
            if getattr(self, '_attach_wait_timer', None):
                try:
                    self._attach_wait_timer.cancel()
                except Exception:
                    pass
                self._attach_wait_timer = None
        except Exception:
            pass
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
