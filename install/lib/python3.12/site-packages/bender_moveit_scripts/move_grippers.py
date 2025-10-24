#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
import sys

class GripperActionCommander(Node):
    def __init__(self):
        super().__init__('gripper_action_commander')

        # Action clients
        self.left_client = ActionClient(self, GripperCommand, '/left_gripper_controller/gripper_cmd')
        self.right_client = ActionClient(self, GripperCommand, '/right_gripper_controller/gripper_cmd')

    def send_goal(self, side: str, action: str):
        if side == "left":
            client = self.left_client
        elif side == "right":
            client = self.right_client
        else:
            self.get_logger().error(f"Lado desconocido: {side}")
            return

        goal = GripperCommand.Goal()
        goal.command.max_effort = 10.0

        # Si le mandas "open" abre el gripper, si le mandas cualquier otra cosa lo cierra
        goal.command.position = 0.8 if action == "open" else 0.0

        self.get_logger().info(f"👉 Enviando '{action}' al {side} gripper...")

        client.wait_for_server()
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rechazado.")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(f"✅ {side} gripper {action} completado. Posición final: {result.position}")

def main(args=None):
    rclpy.init(args=args)
    node = GripperActionCommander()

    if len(sys.argv) == 3:
        action = sys.argv[1].lower()
        side = sys.argv[2].lower()
        node.send_goal(side, action)
    else:
        node.get_logger().info("Uso: ros2 run bender_moveit_scripts move_grippers open left")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
