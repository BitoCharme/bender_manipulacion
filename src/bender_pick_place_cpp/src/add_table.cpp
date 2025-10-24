#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("add_table_and_object");

    moveit::planning_interface::PlanningSceneInterface psi;

    // === 1. Crear objeto de colisión: mesa con 4 patas ===
    moveit_msgs::msg::CollisionObject table;
    table.id = "table1";
    table.header.frame_id = "base_link";

    double table_length = 1.6;
    double table_width  = 0.8;
    double table_height = 0.74;
    double top_thickness = 0.04;
    double leg_side = 0.05;
    double leg_height = table_height - top_thickness;

    table.primitives.resize(5);
    table.primitive_poses.resize(5);

    // --- Tablero ---
    table.primitives[0].type = table.primitives[0].BOX;
    table.primitives[0].dimensions = {table_width, table_length, top_thickness};
    table.primitive_poses[0].position.x = 0.80;
    table.primitive_poses[0].position.y = 0.0;
    table.primitive_poses[0].position.z = leg_height + top_thickness / 2.0;
    table.primitive_poses[0].orientation.w = 1.0;

    // --- 4 Patas ---
    for (int i = 0; i < 4; ++i)
    {
        table.primitives[i + 1].type = table.primitives[i + 1].BOX;
        table.primitives[i + 1].dimensions = {leg_side, leg_side, leg_height};

        double dx = (table_width / 2.0 - leg_side / 2.0);
        double dy = (table_length / 2.0 - leg_side / 2.0);
        double sign_x = (i < 2) ? 1.0 : -1.0;
        double sign_y = (i % 2 == 0) ? 1.0 : -1.0;

        table.primitive_poses[i + 1].position.x = 0.80 + sign_x * dx;
        table.primitive_poses[i + 1].position.y = sign_y * dy;
        table.primitive_poses[i + 1].position.z = leg_height / 2.0;
        table.primitive_poses[i + 1].orientation.w = 1.0;
    }

    table.operation = table.ADD;

    // // === 2. Objeto móvil ===
    // moveit_msgs::msg::CollisionObject object;
    // object.id = "moving_object";
    // object.header.frame_id = "base_link";
    // object.primitives.resize(1);
    // object.primitives[0].type = object.primitives[0].BOX;
    // object.primitives[0].dimensions = {0.04, 0.04, 0.1};
    // object.primitive_poses.resize(1);
    // object.primitive_poses[0].position.x = 0.45;
    // object.primitive_poses[0].position.y = -0.3;
    // object.primitive_poses[0].position.z = 0.8;
    // object.primitive_poses[0].orientation.w = 1.0;
    // object.operation = object.ADD;

    // // === 3. Aplicar objetos ===
    // psi.applyCollisionObjects({table, object});
    // RCLCPP_INFO(node->get_logger(), "Mesa y objeto añadidos al PlanningScene.");

    psi.applyCollisionObjects({table});
    RCLCPP_INFO(node->get_logger(), "Mesa añadida al PlanningScene.");

    // // === 4. Permitir colisión entre el gripper y el objeto ===
    // auto client = node->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
    // if (!client->wait_for_service(std::chrono::seconds(5))) {
    //     RCLCPP_ERROR(node->get_logger(), "No se encontró el servicio apply_planning_scene");
    // } else {
    //     moveit_msgs::msg::PlanningScene ps;
    //     ps.is_diff = true;

    //     // Lista de enlaces del gripper (ajusta estos nombres según tu robot)
    //     std::vector<std::string> gripper_links = {
    //         "r63", "g2ra", "g2rb",
    //         "l6l", "g2la", "g2lb"
    //     };

    //     for (const auto& link : gripper_links)
    //     {
    //         ps.allowed_collision_matrix.entry_names.push_back(link);
    //         ps.allowed_collision_matrix.entry_values.push_back(moveit_msgs::msg::AllowedCollisionEntry());
    //         ps.allowed_collision_matrix.entry_values.back().enabled = {true};  // permitir con este objeto
    //     }

    //     ps.allowed_collision_matrix.entry_names.push_back(object.id);
    //     ps.allowed_collision_matrix.entry_values.push_back(moveit_msgs::msg::AllowedCollisionEntry());
    //     ps.allowed_collision_matrix.entry_values.back().enabled = std::vector<bool>(gripper_links.size(), true);

    //     auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    //     request->scene = ps;
    //     client->async_send_request(request);
    //     RCLCPP_INFO(node->get_logger(), "Permitida colisión entre el gripper y el objeto.");
    // }

    rclcpp::sleep_for(std::chrono::seconds(2));
    rclcpp::shutdown();
    return 0;
}
