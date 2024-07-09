#include "ROIFilter.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //In ROS 2, node names are typically set in the node's constructor, not in the init function. Todo

    auto roifilter = std::make_shared<ROIFilter>();
    roifilter->run();

    rclcpp::shutdown();
}