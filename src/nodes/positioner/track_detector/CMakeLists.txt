cmake_minimum_required(VERSION 3.8)
project(track_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(track_detect_main src/track_detect.cpp)


 ament_target_dependencies(track_detect_main 
 rclcpp 
 sensor_msgs 
 OpenCV
 std_msgs
 cv_bridge)

 install(TARGETS 
 track_detect_main
 DESTINATION lib/track_detector #$(PROJECT_NAME)
 )

#Add dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)
ament_package()
