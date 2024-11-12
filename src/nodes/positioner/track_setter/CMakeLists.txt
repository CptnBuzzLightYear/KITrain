cmake_minimum_required(VERSION 3.5)
project(track_setter)

# Add the directory containing FindGeographicLib.cmake to CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(track_setter src/track_setter.cpp)

ament_target_dependencies(track_setter
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  GeographicLib
  tf2_geometry_msgs
  tf2
  std_msgs
)

install(TARGETS
  track_setter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
