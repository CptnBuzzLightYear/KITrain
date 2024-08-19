cmake_minimum_required(VERSION 3.5)
project(clearance_profile_observer)

# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lidar_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(observer_msgs REQUIRED)

include_directories(
  include
  ${lidar_msgs_INCLUDE_DIRS}
  )


# Include directories
include_directories(
  include
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

# Define the executable
add_executable(clearance_profile_observer src/clearance_profile_observer.cpp)

# Link dependencies
ament_target_dependencies(clearance_profile_observer
  rclcpp
  geometry_msgs
  sensor_msgs
  observer_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  lidar_msgs
)

# Install target
install(TARGETS
  clearance_profile_observer
  DESTINATION lib/${PROJECT_NAME}
)

# Register package
ament_package()
