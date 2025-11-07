#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from visualization_msgs.msg import Marker


class WorkspaceGridSampler(Node):
    def __init__(self):
        super().__init__("workspace_grid_sampler_right")

        # Parámetros
        self.group_name = "right_arm"
        self.link_name = "l6r_1"  # efector final
        self.base_frame = "base_link"

        # Crear clientes a los servicios de MoveIt2
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.validity_client = self.create_client(GetStateValidity, '/check_state_validity')
        self.marker_pub = self.create_publisher(Marker, '/workspace_right_points', 10)

        self.get_logger().info("Esperando servicios de MoveIt2...")
        self.ik_client.wait_for_service()
        self.validity_client.wait_for_service()
        self.get_logger().info("Servicios listos ✅")

        # Definir cubo de muestreo (metros)
        self.x_range = np.linspace(0.0, 0.5, 10)
        self.y_range = np.linspace(-0.3, -0.6, 10)
        self.z_range = np.linspace(0.5, 1.2, 15)

        self.valid_points = []

        self.sample_workspace()
        self.save_csv("workspace_right_grid.csv")

    def sample_workspace(self):
        self.get_logger().info("Comenzando muestreo de grilla...")

        total = len(self.x_range) * len(self.y_range) * len(self.z_range)
        checked = 0

        for x in self.x_range:
            for y in self.y_range:
                for z in self.z_range:
                    checked += 1
                    if checked % 100 == 0:
                        self.get_logger().info(f"Progreso: {checked}/{total}")
                    # Crear pose objetivo
                    print(x,y,z)
                    pose = PoseStamped()
                    pose.header.frame_id = self.base_frame
                    pose.pose.position.x = float(x)
                    pose.pose.position.y = float(y)
                    pose.pose.position.z = float(z)
                    pose.pose.orientation.w = 1.0  # sin rotación (neutral)

                    # Crear solicitud IK
                    ik_req = GetPositionIK.Request()
                    ik_req.ik_request.group_name = self.group_name
                    ik_req.ik_request.pose_stamped = pose
                    ik_req.ik_request.ik_link_name = self.link_name
                    ik_req.ik_request.timeout.sec = 1  # tiempo máximo para resolver IK

                    future = self.ik_client.call_async(ik_req)
                    rclpy.spin_until_future_complete(self, future)
                    ik_res = future.result()

                    if ik_res is None or ik_res.error_code.val != 1:
                        print("noooooooo")
                        continue  # no hay solución IK

                    # Validar colisiones
                    validity_req = GetStateValidity.Request()
                    validity_req.robot_state = ik_res.solution
                    validity_req.group_name = self.group_name

                    valid_future = self.validity_client.call_async(validity_req)
                    rclpy.spin_until_future_complete(self, valid_future)
                    valid_res = valid_future.result()

                    if valid_res and valid_res.valid:
                        print("wooooooo")
                        self.valid_points.append((x, y, z))
                        self.publish_marker(x, y, z)

        self.get_logger().info(f"Muestreo terminado: {len(self.valid_points)} puntos alcanzables encontrados ✅")

    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.id = len(self.valid_points)
        self.marker_pub.publish(marker)

    def save_csv(self, filename):
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z'])
            writer.writerows(self.valid_points)
        self.get_logger().info(f"Puntos guardados en {filename}")


def main():
    rclpy.init()
    node = WorkspaceGridSampler()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
