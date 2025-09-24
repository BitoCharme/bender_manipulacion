#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class CylinderPublisher(Node):
    def __init__(self):
        super().__init__('cylinder_publisher')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_cylinder)

    def publish_cylinder(self):
        # Primer cilindro en frame "map"
        marker = Marker()
        marker.header.frame_id = "map"    
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "basic_shapes"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = 3.0   
        marker.pose.position.y = 1.0   
        marker.pose.position.z = 1.5   
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 2.0   
        marker.scale.y = 2.0   
        marker.scale.z = 3.0   

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0   

        self.publisher.publish(marker)

        # Segundo cilindro en frame "base_link" 
        marker2 = Marker()
        marker2.header.frame_id = "base_link"    
        marker2.header.stamp = self.get_clock().now().to_msg()
        marker2.ns = "basic_shapes"
        marker2.id = 1
        marker2.type = Marker.CYLINDER
        marker2.action = Marker.ADD

        marker2.pose.position.x = 2.0   
        marker2.pose.position.y = -1.0   
        marker2.pose.position.z = 1.0   
        marker2.pose.orientation.x = 0.0
        marker2.pose.orientation.y = 0.0
        marker2.pose.orientation.z = 0.0
        marker2.pose.orientation.w = 1.0

        marker2.scale.x = 1.5   
        marker2.scale.y = 1.5   
        marker2.scale.z = 2.5   

        marker2.color.r = 0.0
        marker2.color.g = 0.0
        marker2.color.b = 1.0  # Azul
        marker2.color.a = 1.0   

        self.publisher.publish(marker2)
        
        self.get_logger().info("2 Cilindros: ROJO en MAP (3,1,1.5) y AZUL en base_link (2,-1,1)")

def main(args=None):
    rclpy.init(args=args)
    node = CylinderPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
