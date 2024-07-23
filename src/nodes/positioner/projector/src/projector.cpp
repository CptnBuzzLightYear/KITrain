#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class MapGnssVisualizer : public rclcpp::Node
{
public:
    MapGnssVisualizer()
        : Node("map_gnss_visualizer")
    {
        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "map", 10, std::bind(&MapGnssVisualizer::path_callback, this, std::placeholders::_1));

        gnss_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "gnss", 10, std::bind(&MapGnssVisualizer::gnss_callback, this, std::placeholders::_1));

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "GNSS_Projection", 10);
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received path data with %lu poses", msg->poses.size());
        // Store the path data for visualization
        path_ = msg;
        publish_markers();
    }

    void gnss_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received GNSS data");
        // Store the GNSS pose
        gnss_pose_ = msg;
        publish_markers();
    }

    void publish_markers()
    {
        if (path_ == nullptr || gnss_pose_ == nullptr)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for both path and GNSS data");
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;

        // Create markers for the path
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "map2";
        path_marker.header.stamp = this->get_clock()->now();
        path_marker.ns = "path_marker";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.1;  // Line width
        path_marker.color.r = 0.0;
        path_marker.color.g = 1.0;
        path_marker.color.b = 0.0;
        path_marker.color.a = 1.0;

        for (const auto &pose : path_->poses)
        {
            path_marker.points.push_back(pose.pose.position);
        }

        if (!path_marker.points.empty())
        {
            marker_array.markers.push_back(path_marker);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Path marker points are empty");
        }

        // Create marker for the GNSS pose
        visualization_msgs::msg::Marker gnss_marker;
        gnss_marker.header.frame_id = "map";
        gnss_marker.header.stamp = this->get_clock()->now();
        gnss_marker.ns = "gnss_marker";
        gnss_marker.id = 1;
        gnss_marker.type = visualization_msgs::msg::Marker::SPHERE;
        gnss_marker.action = visualization_msgs::msg::Marker::ADD;
        gnss_marker.pose = *gnss_pose_;
        gnss_marker.scale.x = 1.0;
        gnss_marker.scale.y = 1.0;
        gnss_marker.scale.z = 1.0;
        gnss_marker.color.r = 1.0;
        gnss_marker.color.g = 0.0;
        gnss_marker.color.b = 0.0;
        gnss_marker.color.a = 1.0;

        marker_array.markers.push_back(gnss_marker);

        RCLCPP_INFO(this->get_logger(), "Publishing %lu markers", marker_array.markers.size());
        marker_publisher_->publish(marker_array);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr gnss_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    nav_msgs::msg::Path::SharedPtr path_;
    geometry_msgs::msg::Pose::SharedPtr gnss_pose_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapGnssVisualizer>());
    rclcpp::shutdown();
    return 0;
}
