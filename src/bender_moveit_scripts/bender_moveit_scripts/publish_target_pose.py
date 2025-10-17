#!/usr/bin/env python3
import argparse
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__("publish_target_pose")
        self.pub = self.create_publisher(PoseStamped, "target_pose_right", 10)
        self.get_logger().info("Nodo listo. Publica geometry_msgs/PoseStamped en /target_pose_right")

    def publish_target_pose(self, x, y, z, qx, qy, qz, qw, frame_id: str = "base_link", rate_hz: float = 10.0):
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        i = 0
        try:
            while i < 2:
                i += 1
                msg.header.stamp = self.get_clock().now().to_msg()
                self.pub.publish(msg)
                self.get_logger().info(f"Publicado PoseStamped en 'target_pose_right' (#{i})")
                time.sleep(1.5)
        except KeyboardInterrupt:
            self.get_logger().info("Publicación interrumpida por el usuario.")

def main():
    # CLI opcional para publicar una vez y salir
    parser = argparse.ArgumentParser(description="Publica geometry_msgs/PoseStamped en /target_pose_right")
    parser.add_argument("--x", type=float)
    parser.add_argument("--y", type=float)
    parser.add_argument("--z", type=float)
    parser.add_argument("--qx", type=float)
    parser.add_argument("--qy", type=float)
    parser.add_argument("--qz", type=float)
    parser.add_argument("--qw", type=float)
    parser.add_argument("--frame-id", type=str, default="base_link")
    # Sin 'repeat': por defecto publica indefinidamente hasta Ctrl+C
    parser.add_argument("--rate", type=float, default=10.0, help="Hz")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = TargetPosePublisher()

    provided = all(v is not None for v in [args.x, args.y, args.z, args.qx, args.qy, args.qz, args.qw])
    if provided:
        try:
            node.publish_target_pose(args.x, args.y, args.z, args.qx, args.qy, args.qz, args.qw,
                                     frame_id=args.frame_id, rate_hz=args.rate)
        finally:
            rclpy.shutdown()
        return

    # Si no hay args, se queda vivo esperando que otro código llame a publish_target_pose()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
