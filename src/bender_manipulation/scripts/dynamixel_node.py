#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray
from dynamixel_sdk import *  # noqa: F401
from dynamixel_command import DynamixelCommander
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32


class DynamixelJointPublisher(Node):
    def __init__(self):
        super().__init__('dynamixel_joint_publisher')

        # Publicador y timer (10 Hz)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Subscriptor a posiciones deseadas (para Dynamixel)
        self.position_sub = self.create_subscription(
            Float64MultiArray,
            'goal_pos',
            self.position_callback,
            10
        )

        # Subscripción al ángulo del hombro
        self.shoulder_sub = self.create_subscription(
            Float32,
            '/shoulder/angle',
            self.shoulder_callback,
            10
        )

        # Nombre del joint del hombro en el URDF
        self.joint_name = "l1l_to_base_link"

        # Parámetro: el tópico del hombro viene en grados
        self.declare_parameter('shoulder_in_degrees', True)
        self._shoulder_in_degrees = self.get_parameter('shoulder_in_degrees').get_parameter_value().bool_value

        # Caché del último ángulo del hombro (en radianes)
        self._shoulder_angle_rad = 0.0

        # Resolver params.yaml desde el share del paquete
        pkg_share = get_package_share_directory('bender_manipulation')
        config_path = os.path.join(pkg_share, 'config', 'params.yaml')
        self.get_logger().info(f'Usando config: {config_path}')

        self.dynamixel = DynamixelCommander(config_path)

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Mapear IDs a nombres de joints (Dynamixel)
        joint_name_map = {
            0: "l2l_to_l1l",
            1: "l3l_to_l2l",
            2: "l4l_to_l3l",
            3: "l5l_to_l4l",
            4: "l6l_to_l5l",
            5: "g2la_to_g1l",
            6: "g2lb_to_g1l"
        }

        # 1) Hombro desde /shoulder/angle
        names = [self.joint_name]
        positions = [self._shoulder_angle_rad]
        velocities = [0.0]

        # 2) Resto de articulaciones desde Dynamixel
        pos, vel = self.dynamixel.get_joints_data()

        for i in range(len(pos)):
            # Determinar resolución por tipo
            if i in [0, 2]:  # MX-106
                resolution = 4095
                max_radians = 2 * math.pi
            else:            # RX-28
                resolution = 1023
                max_radians = math.radians(300)

            pos_raw = pos[i]
            vel_raw = vel[i]

            pos_rad = (pos_raw / resolution) * max_radians
            pos_rad = (pos_rad + math.pi) % (2 * math.pi) - math.pi

            # Velocidad cruda a valor "firmado" (nota: si quieres rad/s deberías convertirlo correctamente)
            if vel_raw > 1023:
                velocity = (vel_raw - 1024) * -1
            else:
                velocity = vel_raw

            joint_name = joint_name_map[i]
            names.append(joint_name)
            positions.append(pos_rad)
            velocities.append(velocity)

        joint_state_msg.name = names
        joint_state_msg.position = positions
        joint_state_msg.velocity = velocities
        joint_state_msg.effort = [0.0] * len(names)

        self.joint_pub.publish(joint_state_msg)

    def shoulder_callback(self, msg: Float32):
        angle = float(msg.data)
        if self._shoulder_in_degrees:
            angle = math.radians(angle)
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        self._shoulder_angle_rad = angle
        # Opcional: log liviano
        #self.get_logger().debug(f'Hombro: {self._shoulder_angle_rad:.3f} rad')

    def position_callback(self, msg: Float64MultiArray):
        # Se ignora el primer valor (hombro) y se usan 7 (índices 1..7) para Dynamixel (en radianes).
        if len(msg.data) < 8:
            self.get_logger().warn(
                "Se esperaban al menos 8 valores en /goal_pos. "
                "Se ignora el primero (hombro) y se usan los siguientes 7 (en radianes)."
            )
            return

        desired_joints = list(msg.data[1:8])  # usar índices 1..7 (7 ángulos)
        self.dynamixel.set_joints(desired_joints)

    def destroy_node(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
