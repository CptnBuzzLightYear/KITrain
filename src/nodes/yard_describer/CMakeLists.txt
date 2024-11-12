cmake_minimum_required(VERSION 3.8)
project(yard_describer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add the directory containing FindGeographicLib.cmake to CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib")

find_package(rclcpp REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


add_executable(yard_describer src/map_main.cpp)
ament_target_dependencies(yard_describer rclcpp GeographicLib nav_msgs sensor_msgs visualization_msgs)


install(TARGETS
yard_describer
  DESTINATION lib/${PROJECT_NAME})
ament_package()
