from .publish_target_pose import TargetPosePublisher
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

def move_arm(x, y, z, yaw, pitch, roll):
    rclpy.init()
    node = TargetPosePublisher()
    qx, qy, qz, qw = euler_to_quaternion(yaw, pitch, roll)
    node.publish_target_pose(x, y, z, qx, qy, qz, qw)
    return

def main():
    input_values = input("Enter x, y, z, yaw, pitch, roll (in radians) separated by spaces: ")
    x, y, z, yaw, pitch, roll = map(float, input_values.split())
    move_arm(x, y, z, yaw, pitch, roll)

if __name__ == "__main__":
    main()
