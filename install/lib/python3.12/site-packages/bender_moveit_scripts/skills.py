from .publish_target_pose import TargetPosePublisher
from .move_grippers import GripperActionCommander
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

def move_arm(side, sleep_time, x, y, z, yaw, pitch, roll):
    PoseNode = TargetPosePublisher()
    qx, qy, qz, qw = euler_to_quaternion(yaw, pitch, roll)
    PoseNode.publish_target_pose(side, sleep_time, x, y, z, qx, qy, qz, qw)
    PoseNode.publish_target_pose(side, sleep_time, x, y, z, qx, qy, qz, qw)
    return

def move_gripper(side, action):
    GripperNode = GripperActionCommander()
    GripperNode.send_goal(side, action)
    return

def main():
    rclpy.init()
    input_value_side = input("Enter arm side (right/left): ")
    input_values = input("Enter x, y, z, yaw, pitch, roll (in radians) separated by spaces: ")
    x, y, z, yaw, pitch, roll = map(float, input_values.split())
    move_arm(input_value_side, 4, x, y, z, yaw, pitch, roll)

if __name__ == "__main__":
    main()