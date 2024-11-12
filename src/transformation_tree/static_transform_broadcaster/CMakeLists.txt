cmake_minimum_required(VERSION 3.5)
project(static_transform_broadcaster)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Declare executable
add_executable(static_transform_broadcaster src/static_transform_broadcaster.cpp)

# Link libraries
ament_target_dependencies(static_transform_broadcaster
  rclcpp
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
)

# Install target
install(TARGETS
  static_transform_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package
ament_package()
