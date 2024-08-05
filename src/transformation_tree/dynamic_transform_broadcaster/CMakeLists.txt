cmake_minimum_required(VERSION 3.5)
project(dynamic_transform_broadcaster)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_executable(dynamic_transform_broadcaster src/dynamic_transform_broadcaster.cpp)

ament_target_dependencies(dynamic_transform_broadcaster
  rclcpp
  tf2_ros
  geometry_msgs
  nav_msgs
)

install(TARGETS
  dynamic_transform_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
