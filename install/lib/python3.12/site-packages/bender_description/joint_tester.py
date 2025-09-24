#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
import math
import time

class JointTester(Node):
    def __init__(self):
        super().__init__('joint_tester')

        # Publicadores para brazos y cabeza
        self.left_arm_pub = self.create_publisher(Float64MultiArray, '/left_arm/commands', 10)
        self.right_arm_pub = self.create_publisher(Float64MultiArray, '/right_arm/commands', 10)
        self.head_pub = self.create_publisher(Float64MultiArray, '/head/commands', 10)

        # Publicadores para grippers (si es solo 1 DOF cada uno)
        self.left_gripper_pub = self.create_publisher(Float64MultiArray, '/left_gripper/commands', 10)
        self.right_gripper_pub = self.create_publisher(Float64MultiArray, '/right_gripper/commands', 10)

        self.timer = self.create_timer(1.0, self.move_joints)  # publicamos cada 1s
        self.phase = 0

        # Definir posiciones de prueba
        self.left_arm_positions = [
            [0, 0, 0, 0, 0, 0],
            [0.5, -0.5, 0.3, -0.3, 0.2, -0.2]
        ]
        self.right_arm_positions = [
            [0, 0, 0, 0, 0, 0],
            [-0.5, 0.5, -0.3, 0.3, -0.2, 0.2]
        ]
        self.head_positions = [
            [0, 0],
            [0.3, -0.3]
        ]
        self.gripper_right_positions = [
            [0, 0],
            [0.8, 0.8]
        ]

        self.gripper_left_positions = [
            [0, 0],
            [0.8, 0.8]
        ]

    def move_joints(self):
        # Elegir fase
        idx = self.phase % 2

        # Brazos
        left_msg = Float64MultiArray()
        left_msg.data = self.left_arm_positions[idx]
        self.left_arm_pub.publish(left_msg)

        right_msg = Float64MultiArray()
        right_msg.data = self.right_arm_positions[idx]
        self.right_arm_pub.publish(right_msg)

        # Cabeza
        head_msg = Float64MultiArray()
        head_msg.data = self.head_positions[idx]
        self.head_pub.publish(head_msg)

        # Grippers
        left_gripper_msg = Float64MultiArray()
        left_gripper_msg.data = self.gripper_left_positions[idx]
        self.left_gripper_pub.publish(left_gripper_msg)

        right_gripper_msg = Float64MultiArray()
        right_gripper_msg.data = self.gripper_right_positions[idx]
        self.right_gripper_pub.publish(right_gripper_msg)

        self.get_logger().info(f'Moviendo fase {idx}')
        self.phase += 1

def main(args=None):
    rclpy.init(args=args)
    node = JointTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
