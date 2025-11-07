#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray
from geometry_msgs.msg import Twist
import serial
import math


class SerialROSNode(Node):
    def __init__(self):
        super().__init__('serial_shoulder_node')

        # Parámetros
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Serial
        self.ser = serial.Serial(port, baud)

        # Publisher
        self.pub_angle = self.create_publisher(Float32, 'shoulder/angle', 10)

        # Subscriber: posiciones deseadas
        self.sub_goal = self.create_subscription(
            Float64MultiArray,
            'goal_pos',
            self.send_position,
            10
        )

        self.get_logger().info(
            "Listo: sub 'goal_pos' → envía 'p angulo_en_grados' y publica 'shoulder/angle'."
        )

    def send_position(self, msg: Float64MultiArray):
        if not msg.data:
            return

        # Primer valor = hombro (en radianes)
        ang_rads = msg.data[0]
        ang_degs = math.degrees(ang_rads)

        # Comando serial
        cmd_serial = f"p {ang_degs}\n".encode('utf-8')
        self.ser.write(cmd_serial)

        # Leer respuesta desde el micro
        line = self.ser.readline()
        #self.get_logger().info(cmd_serial)
        #self.get_logger().info(line)

        if line:
            try:
                angle_value = float(line)
                self.pub_angle.publish(Float32(data=angle_value))
            except ValueError:
                pass


def main():
    rclpy.init()
    node = SerialROSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
