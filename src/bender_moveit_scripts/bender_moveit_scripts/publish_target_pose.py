#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np
import time

class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__("publish_target_pose")

    def publish_target_pose(self, x, y, z, qx, qy, qz, qw):
        self.pub = self.create_publisher(Pose, "target_pose", 10)
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        while True:
            self.pub.publish(msg)
            self.get_logger().info("Publishing Pose on 'target_pose'")
            self.get_logger().info(f"Published Pose: {msg}")
            time.sleep(1)  # Ensure the message is sent before shutting down
        return []

def main():
    rclpy.init()
    node = TargetPosePublisher()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
