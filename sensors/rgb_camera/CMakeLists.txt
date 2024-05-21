cmake_minimum_required(VERSION 3.8)
project(rgb_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(rgb_camera_receiver_main src/rgb_camera_receiver_main.cpp)


ament_target_dependencies(rgb_camera_receiver_main 
rclcpp 
sensor_msgs 
std_msgs 
OpenCV 
image_transport 
cv_bridge
)

install(TARGETS 
rgb_camera_receiver_main
DESTINATION lib/$(PROJECT_NAME) #rgb_camera
)

install(DIRECTORY 
launch
DESTINATION share/rgb_camera
)

ament_package()
