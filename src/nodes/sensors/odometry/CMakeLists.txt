cmake_minimum_required(VERSION 3.8)
project(odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(odometry_receiver src/odometry_receiver.cpp)
ament_target_dependencies(odometry_receiver rclcpp nav_msgs geometry_msgs nlohmann_json)

install(TARGETS
  odometry_receiver
  DESTINATION lib/${PROJECT_NAME})

ament_package()

