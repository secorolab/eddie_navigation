cmake_minimum_required(VERSION 3.15)
project(eddie_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps worlds
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/nav_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
