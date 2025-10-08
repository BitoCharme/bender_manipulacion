#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <string>
#include <chrono>

using namespace std::chrono_literals;
const double tau = 2 * M_PI;

// ------------------ Gripper ------------------
void openGripper(trajectory_msgs::msg::JointTrajectory &posture)
{
    posture.joint_names = {"g1r_to_l6r"};
    posture.points.resize(1);
    posture.points[0].positions = {0.0};
    posture.points[0].time_from_start = rclcpp::Duration::from_seconds(0.5);
}

void closeGripper(trajectory_msgs::msg::JointTrajectory &posture)
{
    posture.joint_names = {"g1r_to_l6r"};
    posture.points.resize(1);
    posture.points[0].positions = {0.05};
    posture.points[0].time_from_start = rclcpp::Duration::from_seconds(0.5);
}

// ------------------ Helper para mover brazo ------------------
bool moveToPose(moveit::planning_interface::MoveGroupInterface &group, const geometry_msgs::msg::Pose &target)
{
    group.setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
        group.execute(plan);
    return success;
}

// ------------------ Crear objetos de colisión ------------------
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Table 1
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions = {0.2, 0.4, 0.4};
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 1.0;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    collision_objects[0].operation = collision_objects[0].ADD;

    // Table 2
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "base_link";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions = {0.2, 0.4, 0.4};
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.0;
    collision_objects[1].primitive_poses[0].position.y = 1.0;
    collision_objects[1].primitive_poses[0].position.z = 0.2;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    collision_objects[1].operation = collision_objects[1].ADD;

    // Object
    collision_objects[2].id = "object";
    collision_objects[2].header.frame_id = "base_link";
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions = {0.02, 0.02, 0.2};
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 1.0;
    collision_objects[2].primitive_poses[0].position.y = 0.0;
    collision_objects[2].primitive_poses[0].position.z = 0.5;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

// ------------------ Secuencia pick and place ------------------
void pickAndPlace(moveit::planning_interface::MoveGroupInterface &arm_group)
{
    trajectory_msgs::msg::JointTrajectory gripper_posture;

    // Pre-grasp
    geometry_msgs::msg::Pose pre_grasp;
    pre_grasp.position.x = 1.0;
    pre_grasp.position.y = 0.0;
    pre_grasp.position.z = 0.4;
    tf2::Quaternion q;
    q.setRPY(0, -tau / 4, 0);
    pre_grasp.orientation = tf2::toMsg(q);

    moveToPose(arm_group, pre_grasp);

    // Grasp
    geometry_msgs::msg::Pose grasp = pre_grasp;
    grasp.position.z = 0.25;
    moveToPose(arm_group, grasp);

    closeGripper(gripper_posture); 
    // Publicar gripper_posture al JointTrajectory controller aquí

    // Post-grasp lift
    geometry_msgs::msg::Pose post_grasp = grasp;
    post_grasp.position.z += 0.15;
    moveToPose(arm_group, post_grasp);

    // Pre-place approach
    geometry_msgs::msg::Pose place = post_grasp;
    place.position.x = 0.0;
    place.position.y = 1.0;
    place.position.z = 0.4;
    q.setRPY(0, 0, tau / 4);
    place.orientation = tf2::toMsg(q);

    moveToPose(arm_group, place);

    // Lower to place
    geometry_msgs::msg::Pose place_down = place;
    place_down.position.z = 0.25;
    moveToPose(arm_group, place_down);

    // Open gripper
    openGripper(gripper_posture);
    // Publicar gripper_posture al JointTrajectory controller aquí

    // Retreat
    geometry_msgs::msg::Pose retreat = place_down;
    retreat.position.z += 0.15;
    moveToPose(arm_group, retreat);
}

// ------------------ Main ------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bender_pick_and_place");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    moveit::planning_interface::MoveGroupInterface arm_group(node, "right_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Añadir mesas y objeto
    addCollisionObjects(planning_scene_interface);
    rclcpp::sleep_for(1s);

    // Ejecutar pick and place
    pickAndPlace(arm_group);

    rclcpp::shutdown();
    return 0;
}
