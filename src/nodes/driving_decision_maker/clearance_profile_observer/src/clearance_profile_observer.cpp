#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "observer_msgs/msg/observer_info.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <vector>
#include <limits>
#include <deque>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ClearanceObserver : public rclcpp::Node {
public:
    ClearanceObserver()
        : Node("clearance_observer"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_) {

        // Subscription to the clearance profile tube
        clearance_profile_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "clearance_profile_tubes", 50,
            std::bind(&ClearanceObserver::clearanceProfileCallback, this, std::placeholders::_1));

        // Subscription to the LiDAR bounding boxes
        bounding_boxes_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "LiDAR_Clustering_BoundingBoxes", 10,
            std::bind(&ClearanceObserver::boundingBoxesCallback, this, std::placeholders::_1));

        // Subscription to the vehicle velocity
        velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "vehicle_velocity", 10,
            std::bind(&ClearanceObserver::velocityCallback, this, std::placeholders::_1));

        // Subscriptions for signal detection and setting
        signal_detected_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "signal_detected", 50,
            std::bind(&ClearanceObserver::signalDetectedCallback, this, std::placeholders::_1));

        signal_setting_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "signal_setting", 50,
            std::bind(&ClearanceObserver::signalSettingCallback, this, std::placeholders::_1));

        // Publisher for the ObserverInfo message
        observer_info_publisher_ = this->create_publisher<observer_msgs::msg::ObserverInfo>(
            "observer_info", rclcpp::QoS(10).keep_last(100));

        // Publisher for the transformed and filtered bounding boxes
        filtered_bounding_boxes_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "filtered_bounding_boxes", 10);

        // Publisher for the interactive clearance profile tubes
        interactive_clearance_profile_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "interactive_clearance_profile_tubes", 50);
    }

private:
    static constexpr size_t HISTORY_SIZE = 5;  // Number of past distances to keep
    std::deque<double> distance_history_;      // Circular buffer for distance history
    double current_velocity_ = 0.0;            // Current vehicle velocity
    bool signal_detected_ = false;
    std::string signal_setting_ = "white";

    visualization_msgs::msg::MarkerArray clearance_profile_tube_;

    // Publishers
    rclcpp::Publisher<observer_msgs::msg::ObserverInfo>::SharedPtr observer_info_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr filtered_bounding_boxes_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr interactive_clearance_profile_publisher_;

    // Subscriptions
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr clearance_profile_subscription_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr bounding_boxes_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr signal_detected_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr signal_setting_subscription_;

    // TF buffer and listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Callback for signal detection
    void signalDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        signal_detected_ = msg->data;
    }

    // Callback for signal setting
    void signalSettingCallback(const std_msgs::msg::String::SharedPtr msg) {
        signal_setting_ = msg->data;
    }

    // // Callback for clearance profile tubes
    // void clearanceProfileCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    //     clearance_profile_tube_ = *msg;

    //     // Prepare a copy for interactive clearance profile tubes
    //     visualization_msgs::msg::MarkerArray interactive_clearance_profile = *msg;

    //     // Check if the signal is red, and recolor the tubes if necessary
    //     if (signal_setting_ == "red") {
    //         for (auto& marker : interactive_clearance_profile.markers) {
    //             marker.color.r = 1.0;
    //             marker.color.g = 0.0;
    //             marker.color.b = 0.0;
    //             marker.color.a = 0.7;  // Set alpha to make it slightly opaque
    //         }
    //     }

    //     // Publish the modified clearance profile tubes
    //     interactive_clearance_profile_publisher_->publish(interactive_clearance_profile);
    // }

    void clearanceProfileCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    clearance_profile_tube_ = *msg;

    // Prepare a copy for interactive clearance profile tubes
    visualization_msgs::msg::MarkerArray interactive_clearance_profile = *msg;

    // Clear previous markers
    clearAllMarkers();

    // Sleep for a short time to allow for old markers to be cleared
    std::this_thread::sleep_for(100ms);

    // Check if the signal is red, and recolor the tubes if necessary
    if (signal_setting_ == "red") {
        for (auto& marker : interactive_clearance_profile.markers) {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.7;  // Set alpha to make it slightly opaque
        }
    } else {
        // Optionally, restore original color when the signal is not red
        for (auto& marker : interactive_clearance_profile.markers) {
           
        marker.color.r = 0.0;
        marker.color.g = 0.588; // KIT green (0, 150, 130)
        marker.color.b = 0.51;
        marker.color.a = 0.1; // 10% visibility for the tube
        marker.scale.x = 1.0; // Not used for TRIANGLE_LIST but still needed
        }
    }

    // Publish the modified clearance profile tubes
    interactive_clearance_profile_publisher_->publish(interactive_clearance_profile);
}

void clearAllMarkers() {
    if (clearance_profile_tube_.markers.empty()) {
        RCLCPP_WARN(this->get_logger(), "No clearance profile tubes to clear.");
        return;
    }

    visualization_msgs::msg::MarkerArray delete_marker_array;
    for (const auto& marker : clearance_profile_tube_.markers) {
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = marker.header.frame_id;
        delete_marker.header.stamp = this->get_clock()->now();
        delete_marker.ns = "clearance_profile_tubes";  // Ensure this matches the namespace used
        delete_marker.id = marker.id;
        delete_marker.action = visualization_msgs::msg::Marker::DELETE;
        delete_marker_array.markers.push_back(delete_marker);
    }

    // Publish the delete marker array
    interactive_clearance_profile_publisher_->publish(delete_marker_array);
}


    // Callback for bounding boxes (unchanged from original)
    void boundingBoxesCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        visualization_msgs::msg::MarkerArray filtered_markers;
        double min_distance = std::numeric_limits<double>::infinity();

        for (const auto& marker : msg->markers) {
            visualization_msgs::msg::Marker transformed_marker;
            if (transformBoundingBoxToMapFrame(marker, transformed_marker)) {
                bool is_in_path = checkCollision(transformed_marker);
                transformed_marker.color.a = 0.3;

                if (is_in_path) {
                    transformed_marker.color.r = 1.0;
                    transformed_marker.color.g = 0.0;
                    transformed_marker.color.b = 0.0;
                    transformed_marker.color.a = 0.3;

                    visualization_msgs::msg::Marker lidar_frame_marker;
                    if (transformBoundingBoxToLidarFrame(transformed_marker, lidar_frame_marker)) {
                        double distance = calculateDistanceToLidar(lidar_frame_marker);
                        if (distance < min_distance) {
                            min_distance = distance;
                        }
                    }
                } else {
                    transformed_marker.color.r = 0.5;
                    transformed_marker.color.g = 0.5;
                    transformed_marker.color.b = 0.5;
                    transformed_marker.color.a = 0.1;
                }

                transformed_marker.type = visualization_msgs::msg::Marker::CUBE;
                transformed_marker.points.clear();
                filtered_markers.markers.push_back(transformed_marker);
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to transform bounding box.");
            }
        }

        // Publish the filtered bounding boxes
        filtered_bounding_boxes_publisher_->publish(filtered_markers);

        double smoothed_distance = smoothDistance(min_distance);

        observer_msgs::msg::ObserverInfo observer_info_msg;
        observer_info_msg.distance = smoothed_distance;
        observer_info_msg.signal_detected = signal_detected_;
        observer_info_msg.signal_setting = signal_setting_;

        observer_info_publisher_->publish(observer_info_msg);
    }

    // Callback for vehicle velocity (unchanged)
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_velocity_ = msg->linear.x;
    }

    double smoothDistance(double new_distance) {
        if (distance_history_.size() >= HISTORY_SIZE) {
            distance_history_.pop_front();
        }
        distance_history_.push_back(new_distance);

        double sum = 0.0;
        for (double distance : distance_history_) {
            sum += distance;
        }
        return sum / distance_history_.size();
    }

    // Utility functions for transforms and collision checking (unchanged)
    bool transformBoundingBoxToMapFrame(const visualization_msgs::msg::Marker& marker, visualization_msgs::msg::Marker& transformed_marker) {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped_to_fpv = tf_buffer_.lookupTransform("first_person_view", marker.header.frame_id, tf2::TimePointZero);
            transformed_marker = marker;
            tf2::doTransform(marker.pose, transformed_marker.pose, transform_stamped_to_fpv);
            transformed_marker.header.frame_id = "first_person_view";

            geometry_msgs::msg::TransformStamped transform_stamped_to_map = tf_buffer_.lookupTransform("map", "first_person_view", tf2::TimePointZero);
            tf2::doTransform(transformed_marker.pose, transformed_marker.pose, transform_stamped_to_map);
            transformed_marker.header.frame_id = "map";
            return true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform bounding box: %s", ex.what());
            return false;
        }
    }

    bool transformBoundingBoxToLidarFrame(const visualization_msgs::msg::Marker& marker, visualization_msgs::msg::Marker& transformed_marker) {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped_to_lidar = tf_buffer_.lookupTransform("lidar_frame", "map", tf2::TimePointZero);
            transformed_marker = marker;
            tf2::doTransform(marker.pose, transformed_marker.pose, transform_stamped_to_lidar);
            transformed_marker.header.frame_id = "lidar_frame";
            return true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform bounding box: %s", ex.what());
            return false;
        }
    }

    bool checkCollision(const visualization_msgs::msg::Marker& bounding_box) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header = bounding_box.header;
        transform_stamped.transform.translation.x = bounding_box.pose.position.x;
        transform_stamped.transform.translation.y = bounding_box.pose.position.y;
        transform_stamped.transform.translation.z = bounding_box.pose.position.z;
        transform_stamped.transform.rotation = bounding_box.pose.orientation;

        std::vector<geometry_msgs::msg::Point> corners = getBoundingBoxCorners(bounding_box);

        for (auto& corner : corners) {
            tf2::doTransform(corner, corner, transform_stamped);
        }

        for (const auto& corner : corners) {
            if (isPointInsideTube(corner, clearance_profile_tube_)) {
                return true;
            }
        }

        return false;
    }

    std::vector<geometry_msgs::msg::Point> getBoundingBoxCorners(const visualization_msgs::msg::Marker& bounding_box) {
        std::vector<geometry_msgs::msg::Point> corners;
        double half_x = bounding_box.scale.x / 2.0;
        double half_y = bounding_box.scale.y / 2.0;
        double half_z = bounding_box.scale.z / 2.0;

        geometry_msgs::msg::Point p;
        
        p.x = half_x; p.y = half_y; p.z = half_z; corners.push_back(p);
        p.x = half_x; p.y = half_y; p.z = -half_z; corners.push_back(p);
        p.x = half_x; p.y = -half_y; p.z = half_z; corners.push_back(p);
        p.x = half_x; p.y = -half_y; p.z = -half_z; corners.push_back(p);
        p.x = -half_x; p.y = half_y; p.z = half_z; corners.push_back(p);
        p.x = -half_x; p.y = half_y; p.z = -half_z; corners.push_back(p);
        p.x = -half_x; p.y = -half_y; p.z = half_z; corners.push_back(p);
        p.x = -half_x; p.y = -half_y; p.z = -half_z; corners.push_back(p);

        return corners;
    }

    bool isPointInsideTube(const geometry_msgs::msg::Point& point, const visualization_msgs::msg::MarkerArray& tube) {
        for (const auto& tube_marker : tube.markers) {
            for (size_t i = 0; i < tube_marker.points.size(); i += 2) {
                const auto& p1 = tube_marker.points[i];
                const auto& p2 = tube_marker.points[i + 1];

                double distance = pointLineDistance(point, p1, p2);
                if (distance < tube_marker.scale.x) {
                    return true;
                }
            }
        }
        return false;
    }

    double pointLineDistance(const geometry_msgs::msg::Point& p, const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;
        double length_squared = dx*dx + dy*dy + dz*dz;

        if (length_squared == 0.0) {
            dx = p.x - p1.x;
            dy = p.y - p1.y;
            dz = p.z - p1.z;
            return sqrt(dx*dx + dy*dy + dz*dz);
        }

        double t = ((p.x - p1.x) * dx + (p.y - p1.y) * dy + (p.z - p1.z) * dz) / length_squared;
        t = std::max(0.0, std::min(1.0, t));

        geometry_msgs::msg::Point projection;
        projection.x = p1.x + t * dx;
        projection.y = p1.y + t * dy;
        projection.z = p1.z + t * dz;

        dx = p.x - projection.x;
        dy = p.y - projection.y;
        dz = p.z - projection.z;

        return sqrt(dx*dx + dy*dy + dz*dz);
    }

    double calculateDistanceToLidar(const visualization_msgs::msg::Marker& marker) {
        return sqrt(pow(marker.pose.position.x, 2) + pow(marker.pose.position.y, 2) + pow(marker.pose.position.z, 2));
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClearanceObserver>());
    rclcpp::shutdown();
    return 0;
}

