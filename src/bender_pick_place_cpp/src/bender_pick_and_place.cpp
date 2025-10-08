#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
    group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.1);
    group.setPlanningTime(10.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
        RCLCPP_INFO(rclcpp::get_logger("moveit"), "Planning successful, executing...");
        group.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("moveit"), "Planning failed for pose: x=%.2f, y=%.2f, z=%.2f",
                     target.position.x, target.position.y, target.position.z);
    }
    return success;
}

// ------------------ Crear objetos de colisión ------------------
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Mesa
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions = {0.4, 0.6, 0.2};  // ancho, largo, alto
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.40;  // 🔧 antes 0.30 → +10 cm
    collision_objects[0].primitive_poses[0].position.y = -0.30;
    collision_objects[0].primitive_poses[0].position.z = 0.50;  // 🔧 antes 0.30 → +20 cm
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    collision_objects[0].operation = collision_objects[0].ADD;

    // Objeto
    collision_objects[1].id = "object";
    collision_objects[1].header.frame_id = "base_link";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions = {0.03, 0.03, 0.15};
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.40;  // 🔧 antes 0.30 → +10 cm
    collision_objects[1].primitive_poses[0].position.y = -0.30;
    collision_objects[1].primitive_poses[0].position.z = 0.80;  // 🔧 antes 0.60 → +20 cm
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    collision_objects[1].operation = collision_objects[1].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}


// ------------------ Secuencia pick and place ------------------
void pickAndPlace(moveit::planning_interface::MoveGroupInterface &arm_group,
                  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub)
{
    trajectory_msgs::msg::JointTrajectory gripper_posture;

    // Pre-grasp - cerca del objeto, ligeramente arriba
    geometry_msgs::msg::Pose pre_grasp;
    pre_grasp.position.x = 0.38;   // 🔧 más lejos
    pre_grasp.position.y = -0.30;
    pre_grasp.position.z = 0.85;   // 🔧 superficie +20cm
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    pre_grasp.orientation = tf2::toMsg(q);
    RCLCPP_INFO(rclcpp::get_logger("moveit"), "Moving to pre-grasp position...");
    if (!moveToPose(arm_group, pre_grasp)) return;

    // Grasp - baja un poco
    geometry_msgs::msg::Pose grasp = pre_grasp;
    grasp.position.z = 0.80;
    moveToPose(arm_group, grasp);

    // Cerrar gripper
    closeGripper(gripper_posture);
    gripper_pub->publish(gripper_posture);
    rclcpp::sleep_for(1s);

    // Levantar el objeto
    geometry_msgs::msg::Pose post_grasp = grasp;
    post_grasp.position.z += 0.10;
    moveToPose(arm_group, post_grasp);

    // Posición de colocación (más a la derecha, misma altura)
    geometry_msgs::msg::Pose place = post_grasp;
    place.position.x = 0.35;   // 🔧 consistente con nuevo alcance
    place.position.y = -0.15;
    place.position.z = 0.85;
    q.setRPY(0, 0, 0);
    place.orientation = tf2::toMsg(q);
    RCLCPP_INFO(rclcpp::get_logger("moveit"), "Moving to place approach position...");
    if (!moveToPose(arm_group, place)) return;

    // Bajar para soltar
    geometry_msgs::msg::Pose place_down = place;
    place_down.position.z = 0.80;
    moveToPose(arm_group, place_down);

    // Abrir gripper
    openGripper(gripper_posture);
    gripper_pub->publish(gripper_posture);
    rclcpp::sleep_for(1s);

    // Retirada
    geometry_msgs::msg::Pose retreat = place_down;
    retreat.position.z += 0.10;
    moveToPose(arm_group, retreat);
}

// ------------------ Main ------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bender_pick_and_place");

    auto gripper_pub =
        node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/right_gripper_controller/joint_trajectory", 10);

    moveit::planning_interface::MoveGroupInterface arm(node, "right_arm");
    moveit::planning_interface::PlanningSceneInterface psi;

    arm.setPlanningTime(10.0);
    arm.setNumPlanningAttempts(10);
    arm.setMaxVelocityScalingFactor(0.2);
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setEndEffectorLink("l6r_1");

    RCLCPP_INFO(rclcpp::get_logger("moveit"), "Planning frame: %s", arm.getPlanningFrame().c_str());
    RCLCPP_INFO(rclcpp::get_logger("moveit"), "End effector link: %s", arm.getEndEffectorLink().c_str());

    // Posición inicial (home)
    std::vector<double> home = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
    arm.setJointValueTarget(home);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        arm.execute(plan);
        rclcpp::sleep_for(1s);
    }

    // Añadir escena
    addCollisionObjects(psi);
    rclcpp::sleep_for(1s);

    // Ejecutar pick & place
    pickAndPlace(arm, gripper_pub);

    rclcpp::shutdown();
    return 0;
}
