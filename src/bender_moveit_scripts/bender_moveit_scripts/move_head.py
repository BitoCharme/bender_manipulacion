#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import time
import sys

class HeadController(Node):
    def __init__(self):
        super().__init__('head_control')

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory'
        )

        self.joint_names = ['l1h_to_base_link', 'l2h_to_l1h']

        self.get_logger().info('Esperando al servidor de acción...')
        self.client.wait_for_server()
        self.get_logger().info('Conectado al controlador de cabeza ✅')

    def send_goal(self, pan, tilt, duration=1.0):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal_msg.trajectory.points.append(point)

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('El goal fue rechazado ❌')
            return

        self.get_logger().info('Goal aceptado, esperando resultado...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Movimiento completado ✅')

    # ---- Presets ----
    def look_center(self): self.send_goal(0.0, 0.0)
    def look_left(self): self.send_goal(0.4, 0.0)
    def look_right(self): self.send_goal(-0.4, 0.0)
    def look_down(self): self.send_goal(0.0, -0.4)
    def look_up(self): self.send_goal(0.0, 0.4)

    # ---- Movimiento continuo ----
    def sweep(self, tilt=0.0, amplitude=0.8, speed=0.3):
        """Paneo de izquierda a derecha continuo."""
        self.get_logger().info('Iniciando paneo horizontal (Ctrl+C para detener)')
        t0 = time.time()
        try:
            while rclpy.ok():
                t = time.time() - t0
                pan = amplitude * math.sin(speed * t)
                self.send_goal(pan, tilt, duration=0.5)
        except KeyboardInterrupt:
            self.get_logger().info('Paneo detenido.')

    def sweep_table(self, tilt=0.0, amplitude=0.8, speed=0.6):
        """Paneo de izquierda a derecha continuo."""
        self.get_logger().info('Iniciando paneo horizontal (Ctrl+C para detener)')
        self.look_center()
        t0 = time.time()
        t = 0.0
        try:
            while t <= 3.0:
                t = time.time() - t0
                pan = amplitude * math.sin(speed * t)
                self.send_goal(pan, tilt, duration=0.5)
            t0 = time.time()
            t = 0.0
            while t <= 3.0:
                t = time.time() - t0
                pan = amplitude * math.sin(speed * t)
                self.send_goal(pan, tilt=-0.2, duration=0.5)
            t0 = time.time()
            t = 0.0
            while t <= 3.0:
                t = time.time() - t0
                pan = amplitude * math.sin(speed * t)
                self.send_goal(pan, tilt=-0.2, duration=0.5)
        except KeyboardInterrupt:
            self.get_logger().info('Paneo detenido.')
        

def main(args=None):
    rclpy.init(args=args)
    node = HeadController()

    cmd = sys.argv[1] if len(sys.argv) > 1 else None
    if cmd == 'center': node.look_center()
    elif cmd == 'left': node.look_left()
    elif cmd == 'right': node.look_right()
    elif cmd == 'down': node.look_down()
    elif cmd == 'up': node.look_up()
    elif cmd == 'sweep': node.sweep()
    elif cmd == 'sweep_mid': node.sweep(tilt=-0.2)
    elif cmd == 'sweep_down': node.sweep(tilt=-0.4)
    elif cmd == 'sweep_table': node.sweep_table()
    else:
        node.get_logger().info("Uso: ros2 run tu_paquete head_control.py [center|left|right|up|down|sweep]")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
