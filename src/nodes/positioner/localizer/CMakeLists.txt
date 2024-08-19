cmake_minimum_required(VERSION 3.8)
project(localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add the directory containing FindGeographicLib.cmake to CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(GeographicLib REQUIRED)

add_executable(localizer src/localizer.cpp)

ament_target_dependencies(
  localizer
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2_geometry_msgs
  tf2
  GeographicLib
)

install(TARGETS
  localizer
  DESTINATION lib/${PROJECT_NAME})


ament_package()
