#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class PositionCorrector : public rclcpp::Node
{
public:
    PositionCorrector() // to be included in launch file as soon as technically regardable!
    : Node("longitudinal_position_corrector")
    {
        // Subscription to the odometry velocity topic
        velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "velocity_topic", 10, std::bind(&PositionCorrector::velocity_callback, this, std::placeholders::_1));

        // Subscription to the gnss_marker topic
        gnss_marker_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
            "gnss_marker", 10, std::bind(&PositionCorrector::gnss_marker_callback, this, std::placeholders::_1));

        // Subscription to the first_person_view topic
        first_person_view_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "first_person_view", 10, std::bind(&PositionCorrector::first_person_view_callback, this, std::placeholders::_1));

        // Subscription to the highlighted_marker_array topic
        highlighted_marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "highlighted_marker_array", 10, std::bind(&PositionCorrector::highlighted_marker_callback, this, std::placeholders::_1));

        // Initialize publisher for corrected GNSS marker
        corrected_gnss_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("long_and_lat_corrected_gnss_marker", 10);

        // Initialize publisher for corrected first-person view pose
        corrected_first_person_view_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("long_and_lat_corrected_first_person_view", 10);

        RCLCPP_INFO(this->get_logger(), "PositionCorrector node has been initialized.");
    }

private:
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Placeholder function for velocity callback
        RCLCPP_INFO(this->get_logger(), "Received velocity data.");
    }

    void gnss_marker_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
    {
        // Placeholder function for GNSS marker callback
        RCLCPP_INFO(this->get_logger(), "Received GNSS marker.");
    }

    void first_person_view_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Placeholder function for first-person view pose callback
        RCLCPP_INFO(this->get_logger(), "Received first-person view pose.");
    }

    void highlighted_marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        // Placeholder function for highlighted marker array callback
        RCLCPP_INFO(this->get_logger(), "Received highlighted marker array.");
    }

    // ROS2 subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr gnss_marker_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr first_person_view_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr highlighted_marker_sub_;

    // ROS2 publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr corrected_gnss_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr corrected_first_person_view_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionCorrector>());
    rclcpp::shutdown();
    return 0;
}
