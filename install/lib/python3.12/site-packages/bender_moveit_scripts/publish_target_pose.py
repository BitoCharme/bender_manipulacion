#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np

def move_left_arm(x,y,z,yaw,pitch,roll):

    def euler_to_quaternion(yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
    q = euler_to_quaternion(yaw, pitch, roll)
    return [x,y,z,q[0],q[1],q[2],q[3]]

class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__("publish_target_pose")
        self.pub = self.create_publisher(Pose, "target_pose", 10)
        # Build Pose from the example
        msg = Pose()
        ()
        input_values = input("Enter x, y, z, yaw, pitch, roll (in radians) separated by spaces: ")
        x, y, z, yaw, pitch, roll = map(float, input_values.split())
        pose_values = move_left_arm(x, y, z, yaw, pitch, roll)
        msg.position.x = pose_values[0]
        msg.position.y = pose_values[1]
        msg.position.z = pose_values[2]
        msg.orientation.x = pose_values[3]
        msg.orientation.y = pose_values[4]
        msg.orientation.z = pose_values[5]
        msg.orientation.w = pose_values[6]
        # Publish once (and a couple of repeats to avoid race conditions)
        self.get_logger().info("Publishing Pose on 'target_pose'")
        self.pub.publish(msg)
        # Exit
        rclpy.shutdown()

def main():
    rclpy.init()
    node = TargetPosePublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
