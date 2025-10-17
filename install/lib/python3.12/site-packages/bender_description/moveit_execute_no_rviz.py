#!/usr/bin/env python3
import rclpy
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose
from moveit_configs_utils import MoveItConfigsBuilder

def main():
    rclpy.init()

    # 🔹 Cargar la configuración de MoveIt del robot "bender"
    moveit_config = (
        MoveItConfigsBuilder("bender", package_name="bender_moveit_config")
        .to_moveit_configs()
    )

    # 🔹 Crear instancia de MoveItPy con la configuración cargada
    moveit = MoveItPy(node_name="move_group")

    # 🔹 Obtener el grupo de planificación
    arm = moveit.get_planning_component("right_arm")

    # 🔹 Definir la pose objetivo
    target_pose = Pose()
    target_pose.orientation.w = 1.0
    target_pose.position.x = 0.4
    target_pose.position.y = 0.0
    target_pose.position.z = 0.8

    # 🔹 Establecer el objetivo
    arm.set_goal_state(pose_stamped=target_pose, pose_link="tool0")

    # 🔹 Planificar y ejecutar
    plan_result = arm.plan()
    if plan_result:
        print("✅ Trayectoria planificada, ejecutando...")
        arm.execute()
    else:
        print("❌ No se pudo planificar una trayectoria válida.")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
