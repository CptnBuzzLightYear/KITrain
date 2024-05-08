#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rcl_interfaces/msg/parameter_event.hpp>
//#include <voxel_filter/ReconfigureVoxelFilterConfig.h> //no clue what this is and where it comes from

class VoxelFilter : public rclcpp::Node
{
    public:

        VoxelFilter();
        void run() {rclcpp::spin(std::make_shared<VoxelFilter>());};

    private:
        //void _dynamicReconfigureCallback(voxel_filter::ReconfigureVoxelFilterConfig& config, uint32_t level);
        void _dynamicReconfigureCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
        void _rcvPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr& pointCloudMsg);

        // ROS variables
        //ros::NodeHandle _nh;
        //ros::Publisher _pub;
        //ros::Subscriber _sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub;

        double _leafSize = 0.25;

        //dynamic_reconfigure::Server<voxel_filter::ReconfigureVoxelFilterConfig> _server; //not needed in ROS2 
};