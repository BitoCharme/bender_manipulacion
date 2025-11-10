#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math, time

def clamp(x, lo, hi): return max(lo, min(hi, x))

class HeadTracker(Node):
    """
    Head tracker asíncrono muy suavizado + búsqueda progresiva.
    Sigue /object/point; si se pierde, panea alrededor de la última
    posición segura con amplitud creciente.
    """
    def __init__(self):
        super().__init__('head_tracker_object_autosearch')

        # ===== Ganancias y límites =====
        self.kp_pan = 0.8             # menor kp => más suave
        self.kp_tilt = 0.55
        self.max_step = 0.03          # rad/orden (límite de velocidad)
        self.max_accel_step = 0.012   # rad/orden (límite de aceleración ≈ jerk)
        self.goal_duration = 0.35     # s por movimiento (un poco más lento)
        self.rate_hz = 10.0
        self.deadband = 0.01          # rad: si el error es menor, no corrijo

        # ===== Pérdida de objetivo / búsqueda =====
        self.lost_timeout = 2.0       # s para considerar “perdido”
        self.pan_limit = 0.7
        self.tilt_limit = 0.6

        # Suavizados (0..1). Más alto = más suave (pero más lento).
        self.alpha_target = 0.35      # filtra objetivo (desired angles)
        self.alpha_output = 0.4       # filtra salida (cmd)

        # Paneo:
        self.base_paneo_amp = 0.25    # amplitud inicial
        self.max_paneo_amp  = 0.65    # amplitud máxima
        self.paneo_speed    = 0.5     # rad/s
        self.amp_growth_per_s = 0.12  # crecimiento de amplitud por segundo perdido

        # ===== Estado =====
        self.cmd_pan = 0.0
        self.cmd_tilt = 0.0
        self.last_point = None
        self.last_seen = 0.0
        self.have_target = False
        self.executing = False
        self.search_mode = False
        self.last_safe_pan = 0.0
        self.last_safe_tilt = 0.0
        self.t_search = 0.0
        self.paneo_amp = self.base_paneo_amp

        # Objetivo suavizado
        self.des_pan_f = 0.0
        self.des_tilt_f = 0.0

        # ===== Acción =====
        self.client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')
        self.joint_names = ['l1h_to_base_link', 'l2h_to_l1h']

        self.get_logger().info("Esperando al head_controller...")
        self.client.wait_for_server()
        self.get_logger().info("✅ Conectado al head_controller")

        # ===== Subs y timer =====
        self.sub = self.create_subscription(PointStamped, '/object/point', self.cb_object, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.loop)

        self.get_logger().info("🎯 Head Tracker suavizado + búsqueda progresiva iniciado")

    # -------------------------------------------------
    # Callback del objeto
    # -------------------------------------------------
    def cb_object(self, msg: PointStamped):
        px, py, pz = msg.point.x, msg.point.y, msg.point.z
        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
            return
        if abs(px) < 1e-3:  # evito tilt infinito si x≈0
            return

        self.last_point = msg.point
        self.last_seen = time.time()
        self.have_target = True

        # Guardar última pose segura (donde “sabemos” que veía el objeto)
        self.last_safe_pan = self.cmd_pan
        self.last_safe_tilt = self.cmd_tilt

        # Reinicio búsqueda si estaba activa
        if self.search_mode:
            self.search_mode = False
            self.paneo_amp = self.base_paneo_amp
            self.get_logger().info("👁️ Objeto re-detectado — saliendo de búsqueda")

    # -------------------------------------------------
    # Control principal
    # -------------------------------------------------
    def loop(self):
        now = time.time()
        dt_lost = now - self.last_seen

        if (not self.have_target) or (dt_lost > self.lost_timeout):
            # === MODO BÚSQUEDA ===
            if not self.search_mode:
                self.search_mode = True
                self.t_search = now
                self.paneo_amp = self.base_paneo_amp
                self.get_logger().warn("🔎 Objeto perdido — paneo progresivo")
            # crecer suavemente la amplitud con el tiempo perdido
            self.paneo_amp = clamp(self.base_paneo_amp + self.amp_growth_per_s * (now - self.t_search),
                                   self.base_paneo_amp, self.max_paneo_amp)
            self._search_pattern(now)
            return

        # === FOLLOW ===
        p = self.last_point
        desired_pan = math.atan2(p.y, p.x)
        desired_tilt = -math.atan2(p.z, p.x)

        # 1) Suavizar el objetivo
        self.des_pan_f  = self.alpha_target * desired_pan  + (1 - self.alpha_target) * self.des_pan_f
        self.des_tilt_f = self.alpha_target * desired_tilt + (1 - self.alpha_target) * self.des_tilt_f

        # 2) Error al objetivo suavizado + deadband
        e_pan  = self.des_pan_f  - self.cmd_pan
        e_tilt = self.des_tilt_f - self.cmd_tilt
        if abs(e_pan)  < self.deadband: e_pan  = 0.0
        if abs(e_tilt) < self.deadband: e_tilt = 0.0

        # 3) Proporcional con límites de velocidad
        step_pan  = clamp(self.kp_pan  * e_pan,  -self.max_step, self.max_step)
        step_tilt = clamp(self.kp_tilt * e_tilt, -self.max_step, self.max_step)

        # 4) Limitador de aceleración (cambio respecto a la orden previa)
        step_pan  = clamp(step_pan,  -self.max_accel_step, self.max_accel_step)
        step_tilt = clamp(step_tilt, -self.max_accel_step, self.max_accel_step)

        # 5) Integrar y filtrar salida (segunda capa)
        cmd_pan_next  = clamp(self.cmd_pan  + step_pan,  -self.pan_limit,  self.pan_limit)
        cmd_tilt_next = clamp(self.cmd_tilt + step_tilt, -self.tilt_limit, self.tilt_limit)
        self.cmd_pan  = self.alpha_output * cmd_pan_next  + (1 - self.alpha_output) * self.cmd_pan
        self.cmd_tilt = self.alpha_output * cmd_tilt_next + (1 - self.alpha_output) * self.cmd_tilt

        # Envío asíncrono
        if not self.executing and self.client.server_is_ready():
            self._send_async(self.cmd_pan, self.cmd_tilt, self.goal_duration)

    # -------------------------------------------------
    # Paneo progresivo alrededor de la última zona segura
    # -------------------------------------------------
    def _search_pattern(self, now):
        t = now - self.t_search
        pan  = clamp(self.last_safe_pan  + self.paneo_amp * math.sin(self.paneo_speed * t),
                     -self.pan_limit, self.pan_limit)
        tilt = clamp(self.last_safe_tilt + 0.12 * math.sin(self.paneo_speed * 0.5 * t),
                     -self.tilt_limit, self.tilt_limit)

        # También suavizamos salida en búsqueda
        self.cmd_pan  = 0.5 * pan  + 0.5 * self.cmd_pan
        self.cmd_tilt = 0.5 * tilt + 0.5 * self.cmd_tilt

        if not self.executing and self.client.server_is_ready():
            self._send_async(self.cmd_pan, self.cmd_tilt, self.goal_duration)

    # -------------------------------------------------
    # Envío asíncrono
    # -------------------------------------------------
    def _send_async(self, pan, tilt, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = [float(pan), float(tilt)]
        pt.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration - int(duration)) * 1e9)
        )
        goal.trajectory.points.append(pt)

        self.executing = True
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._goal_sent_cb)

    def _goal_sent_cb(self, fut):
        handle = fut.result()
        if not handle or not handle.accepted:
            self.executing = False
            return
        res_future = handle.get_result_async()
        res_future.add_done_callback(lambda _: self._reset_exec())

    def _reset_exec(self):
        self.executing = False


def main(args=None):
    rclpy.init(args=args)
    node = HeadTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Cerrando Head Tracker.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
