cmake_minimum_required(VERSION 3.8)
project(gnss)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(Boost REQUIRED COMOONENT system)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(gnss_udp_publisher src/gnss_udp_to_topic.cpp)

 ament_target_dependencies(gnss_udp_publisher
 rclcpp 
 sensor_msgs 
 )

 install(TARGETS 
 gnss_udp_publisher
 DESTINATION lib/${PROJECT_NAME}
 )


ament_package()
