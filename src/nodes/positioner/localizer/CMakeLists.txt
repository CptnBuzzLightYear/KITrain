cmake_minimum_required(VERSION 3.8)
project(localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#add_executable(live_position_main src/live_position.cpp)

#ament_target_dependencies(live_position_main 
#rclcpp 
#sensor_msgs 
#std_msgs)

#install(TARGETS 
#live_position_main
#DESTINATION lib/ 
#)

#ament_export_dependencies(
 # rclcpp
 # std_msgs
 # visualization_msgs
 # sensor_msgs
#)

ament_package()
