#!/usr/bin/env python3
import argparse
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__("target_pose_publisher")
        self.pub_r = self.create_publisher(PoseStamped, "target_pose_right", 10)
        self.pub_l = self.create_publisher(PoseStamped, "target_pose_left", 10)
        self.get_logger().info("Nodo listo. Publica geometry_msgs/PoseStamped en /target_pose_right")

    def publish_target_pose(self, side, sleep_time, x, y, z, qx, qy, qz, qw, frame_id="base_link"):
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        # Asegurar tipos float para evitar fallos PyFloat_Check en el binding C
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        pub = self.pub_r if side == "right" else self.pub_l

        msg.header.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)
        self.get_logger().info(f"Publicado PoseStamped en 'target_pose_{side}'")
        time.sleep(sleep_time)

        # #Se publica 2 veces para asegurar la recepción y que llegue con más precisión
        # for i in range(2):
        #     msg.header.stamp = self.get_clock().now().to_msg()
        #     pub.publish(msg)
        #     self.get_logger().info(f"Publicado PoseStamped en 'target_pose_{side}' (#{i+1})")
        #     time.sleep(sleep_time)


# def main():
#     # CLI opcional para publicar una vez y salir
#     parser = argparse.ArgumentParser(description="Publica geometry_msgs/PoseStamped en /target_pose_right")
#     parser.add_argument("--x", type=float)
#     parser.add_argument("--y", type=float)
#     parser.add_argument("--z", type=float)
#     parser.add_argument("--qx", type=float)
#     parser.add_argument("--qy", type=float)
#     parser.add_argument("--qz", type=float)
#     parser.add_argument("--qw", type=float)
#     parser.add_argument("--frame-id", type=str, default="base_link")
#     # Sin 'repeat': por defecto publica indefinidamente hasta Ctrl+C
#     parser.add_argument("--rate", type=float, default=10.0, help="Hz")
#     args, _ = parser.parse_known_args()

#     rclpy.init()
#     node = TargetPosePublisher()

#     provided = all(v is not None for v in [args.x, args.y, args.z, args.qx, args.qy, args.qz, args.qw])
#     if provided:
#         try:
#             node.publish_target_pose_right(args.x, args.y, args.z, args.qx, args.qy, args.qz, args.qw,
#                                      frame_id=args.frame_id, rate_hz=args.rate)
#         finally:
#             rclpy.shutdown()
#         return

#     # Si no hay args, se queda vivo esperando que otro código llame a publish_target_pose()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()
