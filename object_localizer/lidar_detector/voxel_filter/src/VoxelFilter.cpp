#include "VoxelFilter.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

VoxelFilter::VoxelFilter() : rclcpp::Node("voxel_filter")
/*{
    _server.setCallback(boost::bind(&VoxelFilter::_dynamicReconfigureCallback, this, _1, _2));

    ros::NodeHandle prvNh("~");
    prvNh.param<double>("leaf_size", _leafSize, 0.25);

    _pub = _nh.advertise<sensor_msgs::PointCloud2>("/rt40/voxelcloud", 1);
    _sub = _nh.subscribe<sensor_msgs::PointCloud2>("/foursightm/points", 1, &VoxelFilter::_rcvPointCloudCallback, this);
}*/
{
    this->declare_parameter<double>("leaf_size", 0.25);
    _leafSize = this->get_parameter("leaf_size").as_double();

    _pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rt40/voxelcloud", 1);
    _sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/foursightm/points", 1, std::bind(&VoxelFilter::_rcvPointCloudCallback, this, std::placeholders::_1));
}

/*void VoxelFilter::_dynamicReconfigureCallback(voxel_filter::ReconfigureVoxelFilterConfig& config, uint32_t level)
{
    _leafSize = config.leafSize;
}*/ //not needed in ROS2

void VoxelFilter::_rcvPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr& pointCloudMsg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::msg::PointCloud2 filteredCloud;

    pcl::fromROSMsg(*pointCloudMsg, *pointCloud);

    //ros::Time start = ros::Time::now();
    auto start = this->now();

    pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
    voxelFilter.setInputCloud(pointCloud);
    voxelFilter.setLeafSize(_leafSize, _leafSize, _leafSize);
    voxelFilter.filter(*pointCloud);

    pcl::toROSMsg(*pointCloud, filteredCloud);

    _pub->publish(filteredCloud);

    //ros::Time stop = ros::Time::now();
    auto stop = this->now();
    RCLCPP_INFO(this->get_logger(), "Voxel Filter: Avg. Time: %d", (stop - start).nanoseconds() / 1000000);
}