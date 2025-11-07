import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Float64, 'set_velocity', 10)
        self.timer_period = 1.0  # segundos
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        # Publicar un valor Float64 que varía (por ejemplo 6 rps en rad/s)
        rad_per_sec = float(input(":"))
        msg = Float64()
        msg.data = rad_per_sec
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicado: {msg.data:.3f} rad/s')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()