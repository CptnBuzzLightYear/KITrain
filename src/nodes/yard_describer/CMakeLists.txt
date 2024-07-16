cmake_minimum_required(VERSION 3.8)
project(yard_describer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(map_node src/map_main.cpp)
ament_target_dependencies(map_node rclcpp nav_msgs geometry_msgs)

install(TARGETS
  map_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()
