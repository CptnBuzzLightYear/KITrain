cmake_minimum_required(VERSION 3.8)
project(clustering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(lidar_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${lidar_msgs_INCLUDE_DIRS}
  )

add_executable(clustering src/pc_clustering.cpp)
ament_target_dependencies(clustering rclcpp std_msgs sensor_msgs PCL lidar_msgs visualization_msgs pcl_conversions)


install(TARGETS
  clustering
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) 
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

target_link_libraries(clustering ${ament_LIBRARIES} ${PCL_LIBRARIES})

ament_package()
