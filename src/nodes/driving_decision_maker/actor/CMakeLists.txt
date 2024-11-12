cmake_minimum_required(VERSION 3.5)
project(actor)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(jsoncpp REQUIRED) 
find_package(geometry_msgs REQUIRED)  
find_package(visualization_msgs REQUIRED)
find_package(observer_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)
# Create executable
add_executable(actor src/actor.cpp)

# Link dependencies
ament_target_dependencies(actor rclcpp std_msgs nlohmann_json geometry_msgs visualization_msgs observer_msgs tf2_ros tf2_geometry_msgs)


# Install executable
install(TARGETS
  actor
  DESTINATION lib/${PROJECT_NAME})

ament_package()
