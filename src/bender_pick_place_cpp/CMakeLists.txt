cmake_minimum_required(VERSION 3.22)
project(bender_pick_place_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pick_place_demo src/bender_pick_and_place.cpp)
ament_target_dependencies(pick_place_demo
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
)

install(TARGETS pick_place_demo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
