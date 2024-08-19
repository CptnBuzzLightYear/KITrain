cmake_minimum_required(VERSION 3.8)
project(observer_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)

# Generate message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObserverInfo.msg"
 
  DEPENDENCIES std_msgs visualization_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Ensure include directory is exported
include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_lint_auto_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
