cmake_minimum_required(VERSION 3.5)
project(offline_map)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge)

# Include directories
include_directories(include ${OpenCV_INCLUDE_DIRS})

# Executable
add_executable(offline_map src/map.cpp)

# Link libraries
ament_target_dependencies(offline_map rclcpp sensor_msgs OpenCV cv_bridge)

# Install
install(TARGETS offline_map
  DESTINATION lib/${PROJECT_NAME})

ament_package()

