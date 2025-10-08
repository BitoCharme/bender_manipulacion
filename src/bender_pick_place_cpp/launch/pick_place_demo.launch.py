from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "bender", package_name="bender_moveit_config"
    ).to_moveit_configs()

    # Nodo principal de MoveIt (move_group)
    move_group = generate_move_group_launch(moveit_config)

    # Tu nodo de pick & place C++
    pick_place_demo = Node(
        package="bender_pick_place_cpp",  # <-- aquí cambias el package
        executable="pick_place_demo",    # debe coincidir con tu add_executable
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([move_group, pick_place_demo])
