#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

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
        RCLCPP_INFO(rclcpp::get_logger("moveit"), "✅ Planning successful, executing...");
        group.execute(plan);
    }
    else
    {
    // Extra diagnóstico cuando falla el plan
    auto eef = group.getEndEffectorLink();
    auto current = group.getCurrentPose(eef);
    double dx = target.position.x - current.pose.position.x;
    double dy = target.position.y - current.pose.position.y;
    double dz = target.position.z - current.pose.position.z;
    RCLCPP_ERROR(rclcpp::get_logger("moveit"),
             "❌ Planning failed. Target pose: (%.3f, %.3f, %.3f), Current pose: (%.3f, %.3f, %.3f), Δ=(%.3f, %.3f, %.3f), EEF=%s",
             target.position.x, target.position.y, target.position.z,
             current.pose.position.x, current.pose.position.y, current.pose.position.z,
             dx, dy, dz, eef.c_str());
    RCLCPP_ERROR(rclcpp::get_logger("moveit"),
             "   Target quat: [x=%.3f y=%.3f z=%.3f w=%.3f], Current quat: [x=%.3f y=%.3f z=%.3f w=%.3f]",
             target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w,
             current.pose.orientation.x, current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w);
    }
    return success;
}

// ------------------ Crear objetos de colisión ------------------
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(2);

    // Mesa del laboratorio
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions = {0.8, 1.6, 0.74};  // ancho, largo, alto
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.80;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = 0.37;  // mitad de la altura
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    collision_objects[0].operation = collision_objects[0].ADD;

    // Objeto sobre la mesa, casi al borde más cercano al robot
    collision_objects[1].id = "object";
    collision_objects[1].header.frame_id = "base_link";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions = {0.03, 0.03, 0.15};
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.55;  // movido -0.15 m en X
    collision_objects[1].primitive_poses[0].position.y = 0.0;
    collision_objects[1].primitive_poses[0].position.z = 0.74 + 0.075;  // altura mesa + mitad del objeto
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    collision_objects[1].operation = collision_objects[1].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
    RCLCPP_INFO(rclcpp::get_logger("scene"), "🧱 Mesa y objeto añadidos a la escena.");
}

// ------------------ Secuencia pick and place ------------------
void pickAndPlace(moveit::planning_interface::MoveGroupInterface &arm_group,
                  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub)
{
    trajectory_msgs::msg::JointTrajectory gripper_posture;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);

    // Pre-grasp (frente al objeto, 10 cm delante, a la misma altura del centro del objeto)
    geometry_msgs::msg::Pose pre_grasp;
    pre_grasp.position.x = 0.45;  // objeto x=0.55 -> pregrasp 10 cm delante
    pre_grasp.position.y = 0.0;
    pre_grasp.position.z = 0.74 + 0.075;  // misma altura que el centro del objeto
    pre_grasp.orientation = tf2::toMsg(q);
    // Logs adicionales de diagnóstico SOLO en el primer movimiento (donde está fallando)
    auto eef = arm_group.getEndEffectorLink();
    auto current = arm_group.getCurrentPose(eef);
    double dx = pre_grasp.position.x - current.pose.position.x;
    double dy = pre_grasp.position.y - current.pose.position.y;
    double dz = pre_grasp.position.z - current.pose.position.z;
    // Referencias del entorno (según addCollisionObjects)
    const double table_x = 0.80, table_y = 0.0, table_z = 0.37; // centro de la mesa
    const double obj_x = 0.55, obj_y = 0.0, obj_z = 0.74 + 0.075; // centro del objeto
    RCLCPP_INFO(rclcpp::get_logger("moveit"),
                "➡️ Pre-grasp target: (%.3f, %.3f, %.3f) | Current EEF: (%.3f, %.3f, %.3f) | Δ=(%.3f, %.3f, %.3f)",
                pre_grasp.position.x, pre_grasp.position.y, pre_grasp.position.z,
                current.pose.position.x, current.pose.position.y, current.pose.position.z,
                dx, dy, dz);
    RCLCPP_INFO(rclcpp::get_logger("scene"),
                "Scene refs -> table center: (%.2f, %.2f, %.2f), object center: (%.2f, %.2f, %.2f)",
                table_x, table_y, table_z, obj_x, obj_y, obj_z);
    RCLCPP_INFO(rclcpp::get_logger("moveit"), "➡️ Moving to pre-grasp position...");
    if (!moveToPose(arm_group, pre_grasp)) return;

    // Grasp (avanza en X hasta el objeto)
    geometry_msgs::msg::Pose grasp = pre_grasp;
    grasp.position.x = 0.55;  // avanzar hasta la cara del objeto
    moveToPose(arm_group, grasp);

    // Cerrar gripper
    closeGripper(gripper_posture);
    gripper_pub->publish(gripper_posture);
    rclcpp::sleep_for(1s);

    // Levantar el objeto
    geometry_msgs::msg::Pose post_grasp = grasp;
    post_grasp.position.z += 0.10;
    moveToPose(arm_group, post_grasp);

    // Lugar de colocación (solo 5 cm hacia atrás en X)
    geometry_msgs::msg::Pose place = post_grasp;
    place.position.x = 0.50;  // mantiene -5 cm desde post_grasp tras mover -0.15 m
    place.position.y = 0.0;
    place.position.z = post_grasp.position.z;
    place.orientation = tf2::toMsg(q);
    RCLCPP_INFO(rclcpp::get_logger("moveit"), "➡️ Moving to place position...");
    if (!moveToPose(arm_group, place)) return;

    // Bajar para soltar
    geometry_msgs::msg::Pose place_down = place;
    place_down.position.z = grasp.position.z;
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

    RCLCPP_INFO(rclcpp::get_logger("moveit"), "🦾 Planning frame: %s", arm.getPlanningFrame().c_str());
    RCLCPP_INFO(rclcpp::get_logger("moveit"), "🦾 End effector link: %s", arm.getEndEffectorLink().c_str());

    // Esperar a tener estado de robot válido y tiempo avanzado (procesando callbacks)
    {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node);
        auto eef = arm.getEndEffectorLink();
        const auto start = std::chrono::steady_clock::now();
        bool ready = false;
        while (rclcpp::ok() && !ready)
        {
            auto now = node->get_clock()->now();
            auto cp = arm.getCurrentPose(eef);
            // tiempo > 0 y pose no nula
            if (now.seconds() > 0.1 && !(cp.pose.position.x == 0.0 && cp.pose.position.y == 0.0 && cp.pose.position.z == 0.0))
            {
                ready = true;
                break;
            }
            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(10))
            {
                RCLCPP_WARN(rclcpp::get_logger("moveit"), "Waiting for /joint_states and /clock... still not ready");
                break; // no bloquear indefinidamente; continuamos, pero puede fallar
            }
            exec.spin_some();
            rclcpp::sleep_for(200ms);
        }
    }

    // Añadir escena
    addCollisionObjects(psi);
    rclcpp::sleep_for(1s);

    // Ejecutar pick & place
    pickAndPlace(arm, gripper_pub);

    rclcpp::shutdown();
    return 0;
}
