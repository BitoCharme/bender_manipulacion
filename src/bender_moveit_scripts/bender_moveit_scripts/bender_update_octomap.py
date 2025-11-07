#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_ros_planning_interface._moveit_cpp import MoveItCpp
from moveit_ros_perception.occupancy_map_monitor import PointCloudOctomapUpdater

class OctomapUpdaterNode(Node):
    def __init__(self):
        super().__init__("bender_update_octomap")

        # Parámetros
        self.declare_parameter("point_cloud_topic", "/camera/depth/color/points")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("octomap_resolution", 0.05)
        self.declare_parameter("max_range", 1.5)

        point_cloud_topic = self.get_parameter("point_cloud_topic").get_parameter_value().string_value
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        resolution = self.get_parameter("octomap_resolution").get_parameter_value().double_value
        max_range = self.get_parameter("max_range").get_parameter_value().double_value

        # Inicializa MoveItCpp
        self.moveit_cpp = MoveItCpp(self)

        # Inicializa PointCloudOctomapUpdater
        self.pc_updater = PointCloudOctomapUpdater(
            topic=point_cloud_topic,
            frame_id=frame_id,
            resolution=resolution,
            max_range=max_range,
        )

        # Conecta el updater al PlanningSceneMonitor
        self.moveit_cpp.get_planning_scene_monitor().add_point_cloud_updater(self.pc_updater)
        self.get_logger().info("Octomap updater iniciado con MoveItCpp")

def main(args=None):
    rclpy.init(args=args)
    node = OctomapUpdaterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
