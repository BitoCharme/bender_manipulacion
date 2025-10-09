#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np

def euler_to_quaternion(yaw, pitch, roll):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__("publish_target_pose")

    def move_left_arm(self):
        input_values = input("Enter x, y, z, yaw, pitch, roll (in radians) separated by spaces: ")
        x, y, z, yaw, pitch, roll = map(float, input_values.split())
        q = self.euler_to_quaternion(yaw, pitch, roll)
        self.pub = self.create_publisher(Pose, "target_pose", 10)
        self.get_logger().info("Publishing Pose on 'target_pose'")
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        self.pub.publish(msg)
        return [x,y,z,q[0],q[1],q[2],q[3]]


def main():
    rclpy.init()
    node = TargetPosePublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
