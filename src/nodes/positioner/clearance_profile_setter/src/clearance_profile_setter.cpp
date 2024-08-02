#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ClearanceProfileSetter : public rclcpp::Node
{
public:
    ClearanceProfileSetter()
    : Node("clearance_profile_setter")
    {
        // Initialize subscribers
        highlighted_marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "highlighted_driving_path", 10, std::bind(&ClearanceProfileSetter::highlightedMarkerCallback, this, std::placeholders::_1));
        first_person_view_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "lateral_corrected_first_person_view", 10, std::bind(&ClearanceProfileSetter::firstPersonViewCallback, this, std::placeholders::_1));

        // Initialize publisher for clearance profile body
        clearance_profile_body_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("clearance_profile_body", 10);

        // Define the 2D clearance profile polygon
        clearance_profile_points_ = {
            createPoint(-1.274, 0.0, 0.0),
            createPoint(-1.583, 0.0, 0.380),
            createPoint(-1.683, 0.0, 0.380),
            createPoint(-1.711, 0.0, 0.760),
            createPoint(-1.862, 0.0, 3.590),
            createPoint(-1.706, 0.0, 3.895),
            createPoint(-1.070, 0.0, 4.740),
            createPoint( 1.070, 0.0, 4.740),
            createPoint( 1.706, 0.0, 3.895),
            createPoint( 1.862, 0.0, 3.590),
            createPoint( 1.711, 0.0, 0.760),
            createPoint( 1.683, 0.0, 0.380),
            createPoint( 1.583, 0.0, 0.380),
            createPoint( 1.274, 0.0, 0.0)
        };
    }

private:
    void highlightedMarkerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        highlighted_marker_array_ = *msg;
        generateClearanceProfileBody();
    }

    void firstPersonViewCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        first_person_view_pose_ = *msg;
        generateClearanceProfileBody();
    }

    void generateClearanceProfileBody()
    {
        if (highlighted_marker_array_.markers.empty() || !first_person_view_pose_.header.stamp.sec)
        {
            return;
        }

        visualization_msgs::msg::MarkerArray body_marker_array;

        for (size_t i = 0; i < highlighted_marker_array_.markers.size() - 1; ++i)
        {
            const auto& marker1 = highlighted_marker_array_.markers[i];
            const auto& marker2 = highlighted_marker_array_.markers[i + 1];

            visualization_msgs::msg::Marker body_marker;
            body_marker.header.frame_id = marker1.header.frame_id;
            body_marker.header.stamp = this->now();
            body_marker.ns = "clearance_profile";
            body_marker.id = i;
            body_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            body_marker.action = visualization_msgs::msg::Marker::ADD;
            body_marker.scale.x = 1.0;
            body_marker.scale.y = 1.0;
            body_marker.scale.z = 1.0;
            body_marker.color.r = 1.0;
            body_marker.color.g = 0.0;
            body_marker.color.b = 0.0;
            body_marker.color.a = 1.0;

            for (size_t j = 0; j < clearance_profile_points_.size(); ++j)
            {
                const auto& p1 = clearance_profile_points_[j];
                const auto& p2 = clearance_profile_points_[(j + 1) % clearance_profile_points_.size()];

                geometry_msgs::msg::Point p1_marker1, p2_marker1, p1_marker2, p2_marker2;
                p1_marker1 = transformPoint(p1, marker1.pose.position, first_person_view_pose_.pose.orientation);
                p2_marker1 = transformPoint(p2, marker1.pose.position, first_person_view_pose_.pose.orientation);
                p1_marker2 = transformPoint(p1, marker2.pose.position, first_person_view_pose_.pose.orientation);
                p2_marker2 = transformPoint(p2, marker2.pose.position, first_person_view_pose_.pose.orientation);

                body_marker.points.push_back(p1_marker1);
                body_marker.points.push_back(p2_marker1);
                body_marker.points.push_back(p1_marker2);

                body_marker.points.push_back(p2_marker1);
                body_marker.points.push_back(p1_marker2);
                body_marker.points.push_back(p2_marker2);
            }

            body_marker_array.markers.push_back(body_marker);
        }

        clearance_profile_body_pub_->publish(body_marker_array);
    }

    geometry_msgs::msg::Point32 createPoint(float x, float y, float z)
    {
        geometry_msgs::msg::Point32 point;
        point.x = x;
        point.y = y;
        point.z = z;
        return point;
    }

    geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point32& point, const geometry_msgs::msg::Point& transform, const geometry_msgs::msg::Quaternion& orientation)
    {
        // Rotate the point according to the orientation
        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        tf2::Vector3 v(point.x, point.y, point.z);
        tf2::Vector3 v_rotated = tf2::quatRotate(q, v);

        // Translate the point
        geometry_msgs::msg::Point transformed_point;
        transformed_point.x = v_rotated.x() + transform.x;
        transformed_point.y = v_rotated.y() + transform.y;
        transformed_point.z = v_rotated.z() + transform.z;
        return transformed_point;
    }

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr highlighted_marker_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr first_person_view_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clearance_profile_body_pub_;
    visualization_msgs::msg::MarkerArray highlighted_marker_array_;
    geometry_msgs::msg::PoseStamped first_person_view_pose_;
    std::vector<geometry_msgs::msg::Point32> clearance_profile_points_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClearanceProfileSetter>());
    rclcpp::shutdown();
    return 0;
}
