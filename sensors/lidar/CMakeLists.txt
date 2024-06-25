cmake_minimum_required(VERSION 3.8)
project(lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lidar_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(
  include
  ${lidar_msgs_INCLUDE_DIRS}
  )

#find_package(ki_train REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(lidar_udp_publisher src/lidar_udp_to_topic.cpp)
 ament_target_dependencies(lidar_udp_publisher
 rclcpp 
 sensor_msgs 
 std_msgs
 lidar_msgs
 visualization_msgs
 tf2
 tf2_ros
 tf2_geometry_msgs
)


 install(TARGETS 
 lidar_udp_publisher
 DESTINATION lib/${PROJECT_NAME}
 )

#Add dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  lidar_msgs
  visualization_msgs

)
#install launch files
#install(DIRECTORY lauch DESTINATION share/lidar)

ament_package()
