cmake_minimum_required(VERSION 3.8)
project(longitudinal_position_corrector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(longitudinal_position_corrector src/longitudinal_positioner.cpp)

ament_target_dependencies(
  longitudinal_position_corrector
  rclcpp
  geometry_msgs
  visualization_msgs
)

install(TARGETS
  longitudinal_position_corrector
  DESTINATION lib/${PROJECT_NAME})


ament_package()
