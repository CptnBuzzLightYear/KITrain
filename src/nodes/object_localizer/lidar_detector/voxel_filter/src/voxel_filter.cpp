#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "std_msgs/msg/string.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;

class VoxelFilter : public rclcpp::Node
{
public:
  VoxelFilter()
      : Node("voxel_filter")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/pointcloud", 1, std::bind(&VoxelFilter::topic_callback, this, _1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("VoxelFilterPub", 1);
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    unsigned int num_points = msg->width;
    RCLCPP_INFO(this->get_logger(), "INPUT VOXELFILTER: The number of points in the pointcloud is %i", num_points);

    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    // ROS2 Pointcloud2 to PCL Pointcloud2

    pcl_conversions::toPCL(*msg, *cloud);

    // voxelize the point cloud (ToDo)

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f); // size is subject to change & in meters
    sor.filter(*cloud);

    cloud_filtered = cloud;

    // PCL message to ROS2 message

    sensor_msgs::msg::PointCloud2 cloud_out;
    pcl_conversions::fromPCL(*cloud_filtered, cloud_out);

    unsigned int num_points_out = cloud_out.width;
    RCLCPP_INFO(this->get_logger(), "OUTPUT VOXELFILTER: The number of points in the pointcloud is %i", num_points_out);

    cloud_out.header.frame_id = msg->header.frame_id;
    cloud_out.header.stamp = msg->header.stamp;

    publisher_->publish(cloud_out);
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelFilter>());
  rclcpp::shutdown();
  return 0;
}