#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint, WorkspaceParameters
from moveit_msgs.msg import PlanningOptions
from rclpy.action import ActionClient

class MoveLeftArm(Node):
    def __init__(self):
        super().__init__("move_left_arm_node")

        # Cliente de acción
        self._action_client = ActionClient(self, MoveGroup, '/move_action')

        # Suscripción al tópico /target_pose
        self.subscription = self.create_subscription(
            Pose,
            '/target_pose',
            self.pose_callback,
            10
        )

        self.get_logger().info("Nodo listo, publica una geometry_msgs/Pose en /target_pose")

    def pose_callback(self, msg: Pose):
        self.get_logger().info(f"Recibida target pose: {msg}")

        # Espera que el action server esté listo
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server /move_action no disponible")
            return

        # Crear goal de MoveGroup
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "left_arm"
        goal_msg.request.workspace_parameters = WorkspaceParameters()
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.goal_constraints = []

        # Crear constraints de posición y orientación
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base_link"
        position_constraint.link_name = "l6l_1"  # ⚠️ Cambia según tu robot
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        position_constraint.constraint_region.primitives = []
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = "l6l_1"
        orientation_constraint.orientation = msg.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.05
        orientation_constraint.absolute_y_axis_tolerance = 0.05
        orientation_constraint.absolute_z_axis_tolerance = 0.05
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        goal_msg.request.goal_constraints.append(constraints)

        # Enviar goal
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rechazado por MoveGroup")
            return

        self.get_logger().info("Goal aceptado, esperando resultado...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Movimiento completado: {result.error_code}")

def main():
    rclpy.init()
    node = MoveLeftArm()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
