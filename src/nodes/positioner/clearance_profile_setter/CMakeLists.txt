cmake_minimum_required(VERSION 3.8)
project(clearance_profile_setter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# Create executable
add_executable(clearance_profile_setter src/clearance_profile_setter.cpp)

# Link libraries
ament_target_dependencies(clearance_profile_setter
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)
<<<<<<< HEAD
  install(TARGETS clearance_profile_setter
=======

install(TARGETS clearance_profile_setter
>>>>>>> c7cf2acda0e92c215ccd2ebd0ed7f5e8446a5f86
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
