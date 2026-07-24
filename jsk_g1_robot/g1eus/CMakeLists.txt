cmake_minimum_required(VERSION 3.8)
project(g1eus)

find_package(ament_cmake REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(roseus REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

install(
  DIRECTORY config figs models
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(
  FILES g1.l g1-interface.l jazzy.repos README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
