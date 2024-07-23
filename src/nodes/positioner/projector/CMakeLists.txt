cmake_minimum_required(VERSION 3.5)
project(projector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(projector src/projector.cpp)

ament_target_dependencies(projector
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

install(TARGETS
  projector
  DESTINATION lib/${PROJECT_NAME})

ament_package()
