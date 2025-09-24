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
        marker = Marker()
        marker.header.frame_id = "world"        # o "map", "odom", etc.
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "basic_shapes"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Posición del cilindro
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Escala (dimensiones del cilindro)
        marker.scale.x = 0.5   # diámetro en X
        marker.scale.y = 0.5   # diámetro en Y
        marker.scale.z = 1.0   # altura

        # Color (RGBA)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0   # 1.0 = opaco

        self.publisher.publish(marker)
        self.get_logger().info("Cilindro publicado en RViz")

def main(args=None):
    rclpy.init(args=args)
    node = CylinderPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
