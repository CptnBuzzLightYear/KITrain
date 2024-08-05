#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class DynamicTransformBroadcaster : public rclcpp::Node
{
public:
    DynamicTransformBroadcaster()
        : Node("dynamic_transform_broadcaster")
    {
        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Subscribe to the odometry topic to get the ego vehicle's position
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&DynamicTransformBroadcaster::handle_odom, this, std::placeholders::_1));
    }

private:
    void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        // Set the frame IDs
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "lidar_frame";
        transformStamped.child_frame_id = "first_person_view";

        // Set the transform using the odometry data
        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = msg->pose.pose.position.z;
        transformStamped.transform.rotation = msg->pose.pose.orientation;

        // Broadcast the transform
       // tf_broadcaster_->sendTransform(transformStamped);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicTransformBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
