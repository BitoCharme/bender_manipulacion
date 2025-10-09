from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("bender", package_name="bender_moveit_config").to_moveit_configs()

    # MTC Demo node
    pick_place_demo = Node(
        # package="mtc_tutorial",
        # executable="mtc_node",
        package="bender_pick_place_cpp",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([pick_place_demo])