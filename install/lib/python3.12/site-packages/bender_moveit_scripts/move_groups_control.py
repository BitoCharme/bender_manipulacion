#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    OrientationConstraint,
    PositionConstraint,
    WorkspaceParameters,
)
from rclpy.action import ActionClient
from std_msgs.msg import String
from control_msgs.action import GripperCommand
from rclpy.task import Future


class MoveGroupsNode(Node):
    def __init__(self):
        super().__init__("move_groups_node")

        # Clientes de acción por grupo
        self.action_clients = {
            "left_arm": ActionClient(self, MoveGroup, "/move_action"),
            "right_arm": ActionClient(self, MoveGroup, "/move_action"),
            "head": ActionClient(self, MoveGroup, "/move_action"),
        }

        # Suscripciones a tópicos por grupo
        self.create_subscription(Pose, "/left_arm_pose", self.left_arm_cb, 10)
        self.create_subscription(Pose, "/right_arm_pose", self.right_arm_cb, 10)
        self.create_subscription(String, "/left_gripper_cmd", self.left_gripper_cb, 10)
        self.create_subscription(String, "/right_gripper_cmd", self.right_gripper_cb, 10)
        self.create_subscription(Pose, "/head_pose", self.head_cb, 10)

        # End-effectors según SRDF
        self.eef_links = {
            "left_arm": "l6l_1",  
            "right_arm": "l6r_1",
            "head": "l2h_1",
        }

        self.get_logger().info("Nodo de MoveGroups listo y suscrito a todos los tópicos")

    # ================= CALLBACKS =================
    def left_arm_cb(self, msg: Pose):
        self.send_move_group_goal("left_arm", msg)

    def right_arm_cb(self, msg: Pose):
        self.send_move_group_goal("right_arm", msg)

    def head_cb(self, msg: Pose):
        self.send_move_group_goal("head", msg)

    def left_gripper_cb(self, msg: String):
        self.send_gripper_command("left_gripper", msg.data)

    def right_gripper_cb(self, msg: String):
        self.send_gripper_command("right_gripper", msg.data)

    # ================= FUNCIONES AUXILIARES =================
    def send_move_group_goal(self, group_name, pose: Pose):
        client = self.action_clients[group_name]

        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Action server para {group_name} no disponible")
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        goal_msg.request.workspace_parameters = WorkspaceParameters()
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.goal_constraints = []

        # Constraints de posición y orientación
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base_link"
        position_constraint.link_name = self.eef_links[group_name]
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = self.eef_links[group_name]
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.05
        orientation_constraint.absolute_y_axis_tolerance = 0.05
        orientation_constraint.absolute_z_axis_tolerance = 0.05
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        future = client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def send_gripper_command(self, gripper_name, command: str):
        self.get_logger().info(f"Ejecutando {command} en {gripper_name}")

        # Cliente de acción para el gripper
        gripper_action_name = f"/{gripper_name}_controller/gripper_cmd"
        gripper_client = ActionClient(self, GripperCommand, gripper_action_name)

        if not gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f"No se encontró el action server para {gripper_action_name}"
            )
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.max_effort = 10.0

        if command == "open":
            goal_msg.command.position = math.radians(90)
        elif command == "close":
            goal_msg.command.position = 0.0
        else:
            self.get_logger().warn(
                f"Comando desconocido para {gripper_name}: {command}"
            )
            return

        send_future = gripper_client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda future: self.gripper_goal_response_callback(future, gripper_name, command)
        )

    def gripper_goal_response_callback(self, future: Future, gripper_name: str, command: str):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"El comando {command} no fue aceptado por {gripper_name}")
            return
        self.get_logger().info(f"Comando {command} aceptado en {gripper_name}, esperando resultado...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.gripper_result_callback(future, gripper_name, command)
        )

    def gripper_result_callback(self, future: Future, gripper_name: str, command: str):
        result = future.result().result
        self.get_logger().info(f"Resultado {command} en {gripper_name}: {result}")

    def goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rechazado")
            return
        self.get_logger().info("Goal aceptado, esperando resultado...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        result = future.result().result
        self.get_logger().info(f"Movimiento completado: {result.error_code}")


def main():
    rclpy.init()
    node = MoveGroupsNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
