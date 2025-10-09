cmake_minimum_required(VERSION 3.22)
project(bender_pick_place_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

# ------------------- Ejecutable pick & place -------------------
add_executable(pick_place_demo src/bender_pick_and_place.cpp)
ament_target_dependencies(pick_place_demo
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
)

# ------------------- Ejecutable solo para agregar la mesa -------------------
add_executable(add_table src/add_table.cpp)
ament_target_dependencies(add_table
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
  geometry_msgs
)

add_executable(mtc_node src/mtc_node.cpp)
ament_target_dependencies(mtc_node
  rclcpp
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_ros_planning_interface
  moveit_core
  tf2_geometry_msgs
  tf2_eigen
  geometry_msgs
  )

# ------------------- Instalación -------------------
install(TARGETS 
  pick_place_demo
  add_table
  mtc_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
