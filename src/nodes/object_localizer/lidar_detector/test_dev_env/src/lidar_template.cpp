#include <memory>

/*
To Use This Template:
1. Change Class Name to the respective Pointcloud Processing Step
2. Change the topic_callback function to the respective processing function
3. Add the respective processing function
4. Add all necessary includes
5. add dependencies to CMakeLists.txt
6. change file name to the respective processing step
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "std_msgs/msg/string.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;

class LidarFilter : public rclcpp::Node
{
public:
  LidarFilter()
      : Node("lidar_filter")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/foursightm/points", 1, std::bind(&LidarFilter::topic_callback, this, _1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("LidarFilterPub", 1);
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    unsigned int num_points = msg->width;
    RCLCPP_INFO(this->get_logger(), "INPUT: The number of points in the pointcloud is %i", num_points);

    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    // ROS2 Pointcloud2 to PCL Pointcloud2 Conversion
    pcl_conversions::toPCL(*msg, *cloud);

    // Insert your pcl object here

    cloud_filtered = cloud;

    // PCL Pointcloud2 to ROS2 Pointcloud2 Conversion
    sensor_msgs::msg::PointCloud2 cloud_out;
    pcl_conversions::fromPCL(*cloud_filtered, cloud_out);

    unsigned int num_points_out = cloud_out.width;
    RCLCPP_INFO(this->get_logger(), "OUTPUT: The number of points in the pointcloud is %i", num_points_out);

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
  rclcpp::spin(std::make_shared<LidarFilter>());
  rclcpp::shutdown();
  return 0;
}