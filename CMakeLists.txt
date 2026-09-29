cmake_minimum_required(VERSION 3.15)
project(eddie_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionConstraints.msg"
  "action/ConstrainedPass.action"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(constrained_pass SHARED src/constrained_pass.cpp)
target_link_libraries(constrained_pass "${cpp_typesupport_target}")
ament_target_dependencies(constrained_pass
  geometry_msgs nav_msgs rclcpp rclcpp_action rclcpp_components tf2 tf2_ros tf2_geometry_msgs
  nav2_costmap_2d)
rclcpp_components_register_node(constrained_pass
  PLUGIN "eddie_navigation::ConstrainedPass"
  EXECUTABLE constrained_pass_node)

add_library(eddie_zone_bt_nodes SHARED src/zone_bt_nodes.cpp)
target_link_libraries(eddie_zone_bt_nodes "${cpp_typesupport_target}" yaml-cpp)
ament_target_dependencies(eddie_zone_bt_nodes
  geometry_msgs nav_msgs rclcpp tf2 tf2_ros tf2_geometry_msgs nav2_behavior_tree behaviortree_cpp)
target_compile_definitions(eddie_zone_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS constrained_pass eddie_zone_bt_nodes
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config maps worlds behavior_trees
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/nav_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
