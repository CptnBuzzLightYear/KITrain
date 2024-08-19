#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <optional>

using namespace std::chrono_literals;

struct Coordinate {
    double longitude;
    double latitude;
    double elevation;
};

class Positioner : public rclcpp::Node {
public:
    Positioner() 
        : Node("Positioner") {
        path_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "highlighted_driving_path", 10, std::bind(&Positioner::pathCallback, this, std::placeholders::_1));
        
        gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gnss", 10, std::bind(&Positioner::gnssCallback, this, std::placeholders::_1));

        velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "vehicle_velocity", 10, std::bind(&Positioner::velocityCallback, this, std::placeholders::_1));

        gnss_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("corrected_gnss_marker", 10);

        first_person_view_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("corrected_first_person_view", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        previous_position_ = std::nullopt;
    }

private:
    void pathCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        highlighted_path_ = msg;
        RCLCPP_INFO(this->get_logger(), "Received highlighted driving path.");
    }

    void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (!highlighted_path_) {
            RCLCPP_WARN(this->get_logger(), "No highlighted path data received yet.");
            return;
        }

        Coordinate coord;
        coord.latitude = msg->latitude;
        coord.longitude = msg->longitude;
        coord.elevation = msg->altitude;

        geometry_msgs::msg::Point gnss_point = transformToMetric(coord);

        auto [interpolated_point, segment_end] = findInterpolatedPointOnHighlightedLine(gnss_point);
        if (!interpolated_point) {
            RCLCPP_WARN(this->get_logger(), "No interpolated point found on the highlighted line.");
            return;
        }

        auto smoothed_position = applyVelocitySmoothing(*interpolated_point);

        auto orientation = calculateOrientation(smoothed_position, segment_end);

        publishGnssMarker(smoothed_position);

        publishFirstPersonView(smoothed_position, orientation);
    }

    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_velocity_ = msg->linear.x;
    }

    geometry_msgs::msg::Point applyVelocitySmoothing(const geometry_msgs::msg::Point& new_position) {
        if (!previous_position_) {
            previous_position_ = new_position;
            return new_position;
        }

        // The smoothing factor alpha is based on the velocity
        double alpha = std::clamp(current_velocity_ * 0.1, 0.1, 0.9);

        geometry_msgs::msg::Point smoothed_position;
        smoothed_position.x = alpha * new_position.x + (1.0 - alpha) * previous_position_->x;
        smoothed_position.y = alpha * new_position.y + (1.0 - alpha) * previous_position_->y;
        smoothed_position.z = 0;

        previous_position_ = smoothed_position;

        return smoothed_position;
    }

    geometry_msgs::msg::Point transformToMetric(const Coordinate& coord) {
        double x, y;
        int zone;
        bool northp;

        GeographicLib::UTMUPS::Forward(coord.latitude, coord.longitude, zone, northp, x, y);

        RCLCPP_INFO(this->get_logger(), "UTM coordinates: Zone %d, %s, X: %f, Y: %f",
                    zone, northp ? "North" : "South", x, y);

        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = 0;
        return point;
    }

    std::pair<std::optional<geometry_msgs::msg::Point>, geometry_msgs::msg::Point> findInterpolatedPointOnHighlightedLine(const geometry_msgs::msg::Point& gnss_point) {
        if (!highlighted_path_) return {std::nullopt, geometry_msgs::msg::Point()};

        double min_dist = std::numeric_limits<double>::max();
        std::optional<geometry_msgs::msg::Point> closest_point = std::nullopt;
        geometry_msgs::msg::Point segment_end;

        for (const auto& marker : highlighted_path_->markers) {
            if (marker.type == visualization_msgs::msg::Marker::LINE_STRIP) {
                for (size_t i = 0; i < marker.points.size() - 1; ++i) {
                    auto proj_point = projectPointOntoLineSegment(marker.points[i], marker.points[i + 1], gnss_point);
                    double dist = std::hypot(proj_point.x - gnss_point.x, proj_point.y - gnss_point.y);
                    if (dist < min_dist) {
                        min_dist = dist;
                        closest_point = proj_point;
                        segment_end = marker.points[i + 1];
                    }
                }
            }
        }

        return {closest_point, segment_end};
    }

    geometry_msgs::msg::Point projectPointOntoLineSegment(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2, const geometry_msgs::msg::Point& p) {
        double lambda = ((p.x - p1.x) * (p2.x - p1.x) + (p.y - p1.y) * (p2.y - p1.y)) /
                        ((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));

        lambda = std::max(0.0, std::min(1.0, lambda)); 

        geometry_msgs::msg::Point projected_point;
        projected_point.x = p1.x + lambda * (p2.x - p1.x);
        projected_point.y = p1.y + lambda * (p2.y - p1.y);
        projected_point.z = 0;

        return projected_point;
    }

    geometry_msgs::msg::Quaternion calculateOrientation(const geometry_msgs::msg::Point& current_point, const geometry_msgs::msg::Point& next_point) {
        double yaw = std::atan2(next_point.y - current_point.y, next_point.x - current_point.x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        geometry_msgs::msg::Quaternion orientation;
        orientation.x = q.x();
        orientation.y = q.y();
        orientation.z = q.z();
        orientation.w = q.w();
        return orientation;
    }

    void publishGnssMarker(const geometry_msgs::msg::Point& gnss_point) {
        visualization_msgs::msg::Marker gnss_marker;
        gnss_marker.header.stamp = this->get_clock()->now();
        gnss_marker.header.frame_id = "map";
        gnss_marker.ns = "gnss_position";
        gnss_marker.id = 0;
        gnss_marker.type = visualization_msgs::msg::Marker::SPHERE;
        gnss_marker.action = visualization_msgs::msg::Marker::ADD;
        
        gnss_marker.pose.position = gnss_point;
        gnss_marker.pose.orientation.w = 1.0;

        gnss_marker.scale.x = 1.0;
        gnss_marker.scale.y = 1.0;
        gnss_marker.scale.z = 1.0;

        gnss_marker.color.a = 1.0;
        gnss_marker.color.r = 0.0;
        gnss_marker.color.g = 1.0;
        gnss_marker.color.b = 0.0;

        gnss_marker_pub_->publish(gnss_marker);
    }

    void publishFirstPersonView(const geometry_msgs::msg::Point& gnss_point, const geometry_msgs::msg::Quaternion& orientation) {
        geometry_msgs::msg::PoseStamped first_person_view;
        first_person_view.header.stamp = this->get_clock()->now();
        first_person_view.header.frame_id = "map";
        
        first_person_view.pose.position = gnss_point;
        first_person_view.pose.orientation = orientation;

        first_person_view_pub_->publish(first_person_view);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "first_person_view";
        transform.transform.translation.x = gnss_point.x;
        transform.transform.translation.y = gnss_point.y;
        transform.transform.translation.z = 0.0;

        transform.transform.rotation = orientation;

        tf_broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gnss_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr first_person_view_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    visualization_msgs::msg::MarkerArray::SharedPtr highlighted_path_;
    std::optional<geometry_msgs::msg::Point> previous_position_;
    double current_velocity_ = 0.0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Positioner>());
    rclcpp::shutdown();
    return 0;
}

