#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <optional>

using namespace std::chrono_literals;

// Structure to store coordinates
struct Coordinate {
    double longitude;
    double latitude;
    double elevation;
};

class Projector : public rclcpp::Node {
public:
    Projector() 
        : Node("lateral_projector"), selected_track_id_(1) {
        // Initialize subscriber for marker array
        marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "yard_topology", 10, std::bind(&Projector::markerCallback, this, std::placeholders::_1));
        
        // Initialize subscriber for GNSS data
        gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gnss", 10, std::bind(&Projector::gnssCallback, this, std::placeholders::_1));

        // Initialize subscriber for order info
        order_sub_ = this->create_subscription<std_msgs::msg::String>(
            "order_info", 10, std::bind(&Projector::orderCallback, this, std::placeholders::_1));

        // Initialize publisher for transformed GNSS position as a marker
        gnss_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lateral_corrected_gnss_marker", 10);

        // Initialize publisher for first-person view pose
        first_person_view_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("lateral_corrected_first_person_view", 10);

        // Initialize publisher for track markers with highlighting
        highlighted_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("highlighted_driving_path", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void markerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        markers_ = msg;
    }

    void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (!markers_) {
            RCLCPP_WARN(this->get_logger(), "No marker data received yet.");
            return;
        }

        // Transform GNSS coordinates to UTM
        Coordinate coord;
        coord.latitude = msg->latitude;
        coord.longitude = msg->longitude;
        coord.elevation = msg->altitude;

        geometry_msgs::msg::Point gnss_point = transformToMetric(coord);

        // Find the closest points on the highlighted line and interpolate
        auto [interpolated_point, segment_end] = findInterpolatedPointOnHighlightedLine(gnss_point);
        if (!interpolated_point) {
            RCLCPP_WARN(this->get_logger(), "No interpolated point found on the highlighted line.");
            return;
        }

        // Determine the orientation towards the next point on the line
        auto orientation = calculateOrientation(*interpolated_point, segment_end);

        // Publish GNSS position as a marker
        publishGnssMarker(*interpolated_point);

        // Publish first-person view pose
        publishFirstPersonView(*interpolated_point, orientation);

        // Highlight and publish the track lines
        publishTrackLine();
    }

    // void orderCallback(const std_msgs::msg::String::SharedPtr msg) {
    //     try {
    //         selected_track_id_ = std::stoi(msg->data);
    //     } catch (const std::exception &e) {
    //         RCLCPP_ERROR(this->get_logger(), "Invalid order info received: %s", e.what());
    //     }
    // }
    void orderCallback(const std_msgs::msg::String::SharedPtr msg) {
    try {
        // Extract the target position from the message data
        std::string data = msg->data;
        std::string target_pos_key = "Target Position: ";
        std::size_t target_pos_start = data.find(target_pos_key);

        if (target_pos_start != std::string::npos) {
            target_pos_start += target_pos_key.length();
            std::size_t target_pos_end = data.find(",", target_pos_start);

            std::string target_pos_str;
            if (target_pos_end != std::string::npos) {
                target_pos_str = data.substr(target_pos_start, target_pos_end - target_pos_start);
            } else {
                target_pos_str = data.substr(target_pos_start);
            }

            selected_track_id_ = std::stoi(target_pos_str); //)-100; //id counts from 1 to 18, but track id is 101 to 118 - fixed in yard describer
            RCLCPP_INFO(this->get_logger(), "Selected Track ID set to: %d", selected_track_id_);
       
            publishTrackLine();

        } else {
            RCLCPP_ERROR(this->get_logger(), "Target Position not found in the order message");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing order info: %s", e.what());
    }
}


    geometry_msgs::msg::Point transformToMetric(const Coordinate& coord) {
        double x, y;
        int zone;
        bool northp;
        GeographicLib::UTMUPS::Forward(coord.latitude, coord.longitude, zone, northp, x, y);
        geometry_msgs::msg::Point point;
        
       // RCLCPP_INFO(this->get_logger(), "UTM coordinates: Zone %d, %s, X: %f, Y: %f",
      //              zone, northp ? "North" : "South", x, y); //tatsächliche GPS Position, passt!
        point.x = x;
        point.y = y;
        point.z = 0; // Ignore elevation and set it to 0
        return point;
    }

    std::pair<std::optional<geometry_msgs::msg::Point>, geometry_msgs::msg::Point> findInterpolatedPointOnHighlightedLine(const geometry_msgs::msg::Point& gnss_point) {
    if (!markers_) return {std::nullopt, geometry_msgs::msg::Point()}; // Use explicit initialization

    double min_dist = std::numeric_limits<double>::max();
    std::optional<geometry_msgs::msg::Point> closest_point = std::nullopt;
    geometry_msgs::msg::Point segment_end;

    for (const auto& marker : markers_->markers) {
        if (marker.id == selected_track_id_ && marker.type == visualization_msgs::msg::Marker::LINE_STRIP) {
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

        lambda = std::max(0.0, std::min(1.0, lambda)); // Clamp lambda between 0 and 1

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

        // Broadcast the transform for first-person view
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

void clearAllMarkers() {
    if (!markers_) {
        RCLCPP_WARN(this->get_logger(), "No marker data received yet.");
        return;
    }

    visualization_msgs::msg::MarkerArray delete_marker_array;
    for (const auto& marker : markers_->markers) {
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = "map";
        delete_marker.header.stamp = this->get_clock()->now();
        delete_marker.ns = "track_lines";  // Ensure the namespace matches
        delete_marker.id = marker.id;
        delete_marker.action = visualization_msgs::msg::Marker::DELETE;
        delete_marker_array.markers.push_back(delete_marker);
    }
    highlighted_marker_pub_->publish(delete_marker_array);
}


void publishTrackLine() {
    if (!markers_) {
        RCLCPP_WARN(this->get_logger(), "No marker data received yet.");
        return;
    }

    // Clear existing markers
    clearAllMarkers();

    // Find the marker corresponding to the selected track ID and publish it as highlighted
    visualization_msgs::msg::MarkerArray highlighted_marker_array;
    for (const auto& marker : markers_->markers) {
        if (marker.id == selected_track_id_) {
            visualization_msgs::msg::Marker highlighted_marker = marker;
            highlighted_marker.color.r = 1.0; // Set color to red
            highlighted_marker.color.g = 0.0;
            highlighted_marker.color.b = 0.0;
            highlighted_marker.color.a = 1.0; // Ensure the alpha channel is set to 1 (fully opaque)
            highlighted_marker_array.markers.push_back(highlighted_marker);
        }
    }

    if (highlighted_marker_array.markers.empty()) {
        RCLCPP_WARN(this->get_logger(), "No marker found for the selected track ID.");
        return;
    }

    highlighted_marker_pub_->publish(highlighted_marker_array);
}





    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr order_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gnss_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr first_person_view_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr highlighted_marker_pub_;
    visualization_msgs::msg::MarkerArray::SharedPtr markers_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    int selected_track_id_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Projector>());
    rclcpp::shutdown();
    return 0;
}
