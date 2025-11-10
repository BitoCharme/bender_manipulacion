#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    desc_pkg = get_package_share_directory('bender_description')
    moveit_pkg = get_package_share_directory('bender_moveit_scripts')
    perception_pkg = get_package_share_directory('bender_perception')

    # === 1️⃣ Launch de MoveIt ===
    launch_bender_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'bender_moveit.launch.py')
        )
    )

    # === 2️⃣ Launch de Move Group ===
    launch_move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'bender_move_group.launch.py')
        )
    )

    # === 3️⃣ Nodos de percepción ===
    yolo_node = Node(
        package='bender_perception',
        executable='yolo_node',
        name='yolo_depth_node',
        output='screen'
    )

    head_tracker = Node(
        package='bender_perception',
        executable='head_tracker',
        name='head_tracker',
        output='screen'
    )

    # === 4️⃣ Nodos de brazos ===
    move_left_arm = Node(
        package='bender_moveit_scripts',
        executable='move_left_arm',
        name='move_left_arm',
        output='screen'
    )

    move_right_arm = Node(
        package='bender_moveit_scripts',
        executable='move_right_arm',
        name='move_right_arm',
        output='screen'
    )

    # ------------------------------------------------------------
    # USO DE TimerAction PARA SECUENCIAR (Jazzy-compatible)
    # ------------------------------------------------------------
    # Se espera unos segundos entre cada bloque para asegurar que los previos están listos.
    start_yolo = TimerAction(period=8.0, actions=[yolo_node])
    start_head_tracker = TimerAction(period=12.0, actions=[head_tracker])
    start_arms = TimerAction(period=18.0, actions=[move_left_arm, move_right_arm])

    # ------------------------------------------------------------
    # LaunchDescription final
    # ------------------------------------------------------------
    ld = LaunchDescription()
    ld.add_action(launch_bender_moveit)
    ld.add_action(launch_move_group)
    ld.add_action(start_yolo)
    # ld.add_action(start_head_tracker)
    ld.add_action(start_arms)

    return ld
