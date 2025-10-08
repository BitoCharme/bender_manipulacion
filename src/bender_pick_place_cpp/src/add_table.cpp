#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("add_table_only");

    moveit::planning_interface::PlanningSceneInterface psi;

    // Crear objeto de colisión: mesa
    moveit_msgs::msg::CollisionObject table;
    table.id = "table1";
    table.header.frame_id = "base_link";
    table.primitives.resize(1);
    table.primitives[0].type = table.primitives[0].BOX;
    table.primitives[0].dimensions = {0.8, 1.6, 0.74};  // ancho, largo, alto
    table.primitive_poses.resize(1);
    table.primitive_poses[0].position.x = 0.80;
    table.primitive_poses[0].position.y = 0.0;
    table.primitive_poses[0].position.z = 0.37;  // mitad de la altura
    table.primitive_poses[0].orientation.w = 1.0;
    table.operation = table.ADD;

    psi.applyCollisionObjects({table});

    RCLCPP_INFO(node->get_logger(), "Mesa añadida al PlanningScene.");

    // Mantener el nodo activo unos segundos para asegurarse de que la escena se actualice
    rclcpp::sleep_for(std::chrono::seconds(2));

    rclcpp::shutdown();
    return 0;
}
