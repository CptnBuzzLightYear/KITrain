cmake_minimum_required(VERSION 3.8)
project(lidar)

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

add_executable(lidar_udp_publisher src/lidar_udp_to_topic.cpp)


 ament_target_dependencies(lidar_udp_publisher
 rclcpp 
 sensor_msgs 
 std_msgs
)

 install(TARGETS 
 lidar_udp_publisher
 DESTINATION lib/lidar #$(PROJECT_NAME)
 )

#Add dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs

)
ament_package()
