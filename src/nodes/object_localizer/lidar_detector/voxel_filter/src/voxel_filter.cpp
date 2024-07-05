#include "VoxelFilter.h"

int main( int argc, char **argv )
{

  //ROS_INFO("Starting the Voxel-filter...");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting the Voxel-filter...");

  rclcpp::init( argc, argv);
  VoxelFilter voxelFilter;
  voxelFilter.run();
  
  //ROS_INFO("Shutting down");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down");

  return 0;
}

//Wegen Config nochmal schauen