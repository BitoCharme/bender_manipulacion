#!/usr/bin/python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.actions import GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# --- Funciones auxiliares ---
# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None


def generate_launch_description():
    # Paquete base
    pkg_bender_desc = 'bender_description'
    pkg_bender_share = get_package_share_directory(pkg_bender_desc)

    # --- Launch args ---
    world = LaunchConfiguration('world')
    world_path = os.path.join(pkg_bender_share, 'worlds', 'empty.world')
    declare_world = DeclareLaunchArgument('world', default_value=world_path)

    declare_rviz = DeclareLaunchArgument('rviz', default_value='True')

    urdf_path = PathJoinSubstitution([pkg_bender_share, 'urdf', 'bender.xacro'])
    declare_urdf = DeclareLaunchArgument('urdf', default_value=urdf_path)

    robot_description = {'robot_description': Command(["xacro ", urdf_path])}

    # --- robot_state_publisher ---
    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description,
        {"use_sim_time": True}]
    )

    # Static TF:
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # --- Gazebo ---
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[('gz_args', [' -r 4 ', world])]
    )

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='both',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'bender',
            '-allow_renaming', 'true',
            '-pause', 'false',
        ]
    )

    # Gazebo-ROS bridge
    bridge_params = os.path.join(pkg_bender_share,'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}']
    )

    # image bridge
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image'],
        remappings=[('/camera/image', '/camera/color/image_raw')]
    )

    # --- ROS2 Control ---
    controller_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            PathJoinSubstitution([pkg_bender_share, 'config', 'controllers', 'controllers.yaml']),
        ],
        output='screen'
    )

    # --- Spawners MoveIt controllers
    # Joint STATE BROADCASTER:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    left_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', "--controller-manager", "/controller_manager"]
    )

    right_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller', "--controller-manager", "/controller_manager"]
    )

    head_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['head_controller', "--controller-manager", "/controller_manager"]
    )

    left_gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_gripper_controller']
    )

    right_gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_gripper_controller']
    )

    return LaunchDescription([
        declare_world,
        declare_urdf,
        declare_rviz,
        start_gazebo_server_cmd,
        start_gazebo_ros_spawner_cmd,
        ros_gz_bridge,
        start_gazebo_ros_image_bridge_cmd,
        robot_state_node,
        static_tf,
        RegisterEventHandler(
            OnProcessExit(
                target_action = start_gazebo_ros_spawner_cmd,
                on_exit = [
                    joint_state_broadcaster_spawner,  
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[left_arm_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=left_arm_spawner,
                on_exit=[right_arm_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=right_arm_spawner,
                on_exit=[head_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=head_spawner,
                on_exit=[left_gripper_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=left_gripper_spawner,
                on_exit=[right_gripper_spawner],
            )
        )
    ])