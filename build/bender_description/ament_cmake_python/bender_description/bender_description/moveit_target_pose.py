#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped

# API de MoveIt2 en ROS 2 Jazzy
from moveit_py.core import MoveItPy
from moveit_py.planning import MultiPipelinePlanRequestParameters


class MoveItTargetPose(Node):
    def __init__(self):
        super().__init__("moveit_target_pose")

        # Inicializa MoveItPy
        self.moveit = MoveItPy(node_name="moveit_py")

        # Obtén el planning component (usa el nombre de tu move_group, ej: "arm")
        self.arm = self.moveit.get_planning_component("left_arm")

        # Suscripción al tópico con la pose objetivo
        self.subscription = self.create_subscription(
            Pose, "target_pose", self.target_pose_callback, 10
        )
        self.get_logger().info("Nodo listo ✅ publica un geometry_msgs/Pose en /target_pose")

    def target_pose_callback(self, msg: Pose):
        self.get_logger().info(f"📍 Recibida target pose: {msg}")

        # Convertir a PoseStamped (se necesita el frame_id de referencia de tu robot)
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"   # ⚠️ cámbialo según tu TF
        target_pose.pose = msg

        # Definir la pose como objetivo
        self.arm.set_goal_state(pose_stamped_msg=target_pose)

        # Configurar parámetros de planificación
        params = MultiPipelinePlanRequestParameters()
        params.plan_request_params.num_planning_attempts = 5
        params.plan_request_params.allowed_planning_time = 5.0

        # Planear
        plan_result = self.arm.plan(params)

        if plan_result:
            self.get_logger().info("✅ Plan encontrado, ejecutando...")
            self.arm.execute()
        else:
            self.get_logger().warn("⚠️ No se encontró un plan para la pose dada")


def main():
    rclpy.init()
    node = MoveItTargetPose()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
