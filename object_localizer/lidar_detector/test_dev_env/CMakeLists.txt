cmake_minimum_required(VERSION 3.8)
project(test_dev_env)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


# From here all things have to be added from this Template (everything above is default)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)


add_executable(test_dev_env_LidarFilter src/lidar_template.cpp)
ament_target_dependencies(test_dev_env_LidarFilter rclcpp std_msgs sensor_msgs PCL pcl_conversions)

install(TARGETS
  test_dev_env_LidarFilter
  DESTINATION lib/${PROJECT_NAME})

# to here

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

ament_package()
