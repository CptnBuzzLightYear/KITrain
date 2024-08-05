cmake_minimum_required(VERSION 3.8)
project(clearance_profile_observer)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  # For transform utilities

find_package(lidar_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(
  include
  ${lidar_msgs_INCLUDE_DIRS}
  )
# Include directories
include_directories(include)

# Add executable
add_executable(clearance_profile_observer src/clearance_profile_observer.cpp)

# Link against dependencies
ament_target_dependencies(clearance_profile_observer
  rclcpp
  visualization_msgs
  std_msgs
  lidar_msgs 
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install targets
install(TARGETS clearance_profile_observer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
