#!/usr/bin/env python3
"""
Dynamic Obstacle Manager - Maneja obstáculos dinámicos que pueden aparecer/desaparecer
durante la ejecución, por ejemplo, objetos detectados por sensores.
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import random
import math

class DynamicObstacleManager(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_manager')
        self.publisher = self.create_publisher(MarkerArray, 'dynamic_obstacles', 10)
        self.timer = self.create_timer(2.0, self.update_obstacles)  # Cada 2 segundos
        
        self.obstacles = []
        self.obstacle_id = 0
        
        self.get_logger().info("Dynamic Obstacle Manager iniciado")

    def create_obstacle(self, x, y, z, radius=0.3, height=1.0, color=(1.0, 0.0, 0.0)):
        """Crea un nuevo obstáculo dinámico"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dynamic_obstacles"
        marker.id = self.obstacle_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        marker.scale.x = radius * 2
        marker.scale.y = radius * 2  
        marker.scale.z = height

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.7  # Semi-transparente

        # Vida útil del obstáculo
        marker.lifetime.sec = 5  # 5 segundos
        
        self.obstacle_id += 1
        return marker

    def update_obstacles(self):
        """Simula la detección de nuevos obstáculos dinámicos"""
        marker_array = MarkerArray()
        
        # Simular la detección de un obstáculo en posición aleatoria
        if random.random() > 0.3:  # 70% probabilidad de crear obstáculo
            x = random.uniform(0.5, 3.0)
            y = random.uniform(-2.0, 2.0)
            z = random.uniform(0.5, 1.0)
            
            # Color aleatorio
            colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 1.0, 0.0)]
            color = random.choice(colors)
            
            obstacle = self.create_obstacle(x, y, z, color=color)
            marker_array.markers.append(obstacle)
            
            self.get_logger().info(f"Obstáculo dinámico creado en ({x:.2f}, {y:.2f}, {z:.2f})")
        
        self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
