cmake_minimum_required(VERSION 3.8)
project(order_definer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(jsoncpp REQUIRED)  # Assuming you use jsoncpp for JSON parsing

include_directories(
  include
)

add_executable(order_definer_node src/receive_order.cpp)

ament_target_dependencies(order_definer_node rclcpp std_msgs nlohmann_json)

install(TARGETS
  order_definer_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
