#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "observer_msgs/msg/observer_info.hpp"  // Include the ObserverInfo message
#include "geometry_msgs/msg/twist.hpp"          // Include for Twist message type
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <vector>
#include <limits>  // For std::numeric_limits
#include <deque>

class ClearanceObserver : public rclcpp::Node {
public:
    ClearanceObserver()
        : Node("clearance_observer"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_) {

        // Subscription to the clearance profile tube, which is in the map frame
        clearance_profile_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "clearance_profile_tubes", 10,
            std::bind(&ClearanceObserver::clearanceProfileCallback, this, std::placeholders::_1));

        // Subscription to the LiDAR bounding boxes, which are in the lidar_frame
        bounding_boxes_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "LiDAR_Clustering_BoundingBoxes", 10,
            std::bind(&ClearanceObserver::boundingBoxesCallback, this, std::placeholders::_1));

        // Subscription to the vehicle velocity
        velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "vehicle_velocity", 10,
            std::bind(&ClearanceObserver::velocityCallback, this, std::placeholders::_1));

        // Publisher for the ObserverInfo message
        observer_info_publisher_ = this->create_publisher<observer_msgs::msg::ObserverInfo>(
            "observer_info", 10);

        // Publisher for the transformed and filtered bounding boxes
        filtered_bounding_boxes_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "filtered_bounding_boxes", 10);
    }

private:
    static constexpr size_t HISTORY_SIZE = 5;  // Number of past distances to keep
    std::deque<double> distance_history_;      // Circular buffer for distance history
    double current_velocity_ = 0.0;            // Current vehicle velocity

    void clearanceProfileCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        clearance_profile_tube_ = *msg;
    }

    void boundingBoxesCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        visualization_msgs::msg::MarkerArray filtered_markers;
        double min_distance = std::numeric_limits<double>::infinity();  // Start with infinity

        for (const auto& marker : msg->markers) {
            visualization_msgs::msg::Marker transformed_marker;
            if (transformBoundingBoxToMapFrame(marker, transformed_marker)) {
                bool is_in_path = checkCollision(transformed_marker);

                // Set transparency (alpha) for the bounding boxes
                transformed_marker.color.a = 0.3; // 30% visibility by default

                if (is_in_path) {
                    // Make the box red and see-through
                    transformed_marker.color.r = 1.0;
                    transformed_marker.color.g = 0.0;
                    transformed_marker.color.b = 0.0;
                    transformed_marker.color.a = 0.3; // 30% transparency for red boxes

                    // Transform the red box to the lidar_frame
                    visualization_msgs::msg::Marker lidar_frame_marker;
                    if (transformBoundingBoxToLidarFrame(transformed_marker, lidar_frame_marker)) {
                        // Calculate distance to the LiDAR scanner and update min_distance if necessary
                        double distance = calculateDistanceToLidar(lidar_frame_marker);
                        if (distance < min_distance) {
                            min_distance = distance;
                        }
                    }
                } else {
                    // Color in grey if not inside the path
                    transformed_marker.color.r = 0.5;
                    transformed_marker.color.g = 0.5;
                    transformed_marker.color.b = 0.5;
                    transformed_marker.color.a = 0.1; // 10% visibility for grey boxes
                }

                // Ensure the box is displayed as a solid cube
                transformed_marker.type = visualization_msgs::msg::Marker::CUBE;

                // Clear any points or lines that might be set inadvertently
                transformed_marker.points.clear();

                filtered_markers.markers.push_back(transformed_marker);
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to transform bounding box.");
            }
        }

        // Publish the filtered bounding boxes
        filtered_bounding_boxes_publisher_->publish(filtered_markers);

        // Smooth the distance
        double smoothed_distance = smoothDistance(min_distance);

        // Prepare and publish the ObserverInfo message
        observer_msgs::msg::ObserverInfo observer_info_msg;
        observer_info_msg.distance = smoothed_distance;
        observer_info_msg.signal_detected = signal_detected_;
        observer_info_msg.signal_setting = signal_setting_;

        observer_info_publisher_->publish(observer_info_msg);
    }

    double smoothDistance(double new_distance) {
        // Add new distance to history and remove oldest if the buffer is full
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

    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_velocity_ = msg->linear.x;  // Update current vehicle velocity
    }

    bool transformBoundingBoxToMapFrame(const visualization_msgs::msg::Marker& marker, visualization_msgs::msg::Marker& transformed_marker) {
        try {
            // Step 1: Transform the entire marker from lidar_frame to first_person_view
            geometry_msgs::msg::TransformStamped transform_stamped_to_fpv = tf_buffer_.lookupTransform("first_person_view", marker.header.frame_id, tf2::TimePointZero);

            RCLCPP_INFO(this->get_logger(), "Transforming from %s to first_person_view", marker.header.frame_id.c_str());
            printTransform(transform_stamped_to_fpv);

            transformed_marker = marker;  // Copy the original marker
            tf2::doTransform(marker.pose, transformed_marker.pose, transform_stamped_to_fpv);
            transformed_marker.header.frame_id = "first_person_view";  // Update frame to first_person_view

            RCLCPP_INFO(this->get_logger(), "After transforming to first_person_view:");
            RCLCPP_INFO(this->get_logger(), "Pose: [%.3f, %.3f, %.3f] Rotation: [%.3f, %.3f, %.3f, %.3f]",
                        transformed_marker.pose.position.x, transformed_marker.pose.position.y, transformed_marker.pose.position.z,
                        transformed_marker.pose.orientation.x, transformed_marker.pose.orientation.y, 
                        transformed_marker.pose.orientation.z, transformed_marker.pose.orientation.w);

            // Step 2: Transform the marker from first_person_view to map
            geometry_msgs::msg::TransformStamped transform_stamped_to_map = tf_buffer_.lookupTransform("map", "first_person_view", tf2::TimePointZero);

            RCLCPP_INFO(this->get_logger(), "Transforming from first_person_view to map");
            printTransform(transform_stamped_to_map);

            tf2::doTransform(transformed_marker.pose, transformed_marker.pose, transform_stamped_to_map);
            transformed_marker.header.frame_id = "map";  // Set the final frame_id to map

            RCLCPP_INFO(this->get_logger(), "After transforming to map frame:");
            RCLCPP_INFO(this->get_logger(), "Pose: [%.3f, %.3f, %.3f] Rotation: [%.3f, %.3f, %.3f, %.3f]",
                        transformed_marker.pose.position.x, transformed_marker.pose.position.y, transformed_marker.pose.position.z,
                        transformed_marker.pose.orientation.x, transformed_marker.pose.orientation.y, 
                        transformed_marker.pose.orientation.z, transformed_marker.pose.orientation.w);

            return true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform bounding box: %s", ex.what());
            return false;
        }
    }

    bool transformBoundingBoxToLidarFrame(const visualization_msgs::msg::Marker& marker, visualization_msgs::msg::Marker& transformed_marker) {
        try {
            // Transform the marker from map to lidar_frame
            geometry_msgs::msg::TransformStamped transform_stamped_to_lidar = tf_buffer_.lookupTransform("lidar_frame", "map", tf2::TimePointZero);

            RCLCPP_INFO(this->get_logger(), "Transforming from map to lidar_frame");
            printTransform(transform_stamped_to_lidar);

            transformed_marker = marker;  // Copy the original marker
            tf2::doTransform(marker.pose, transformed_marker.pose, transform_stamped_to_lidar);
            transformed_marker.header.frame_id = "lidar_frame";  // Set the final frame_id to lidar_frame

            RCLCPP_INFO(this->get_logger(), "After transforming to lidar_frame:");
            RCLCPP_INFO(this->get_logger(), "Pose: [%.3f, %.3f, %.3f] Rotation: [%.3f, %.3f, %.3f, %.3f]",
                        transformed_marker.pose.position.x, transformed_marker.pose.position.y, transformed_marker.pose.position.z,
                        transformed_marker.pose.orientation.x, transformed_marker.pose.orientation.y, 
                        transformed_marker.pose.orientation.z, transformed_marker.pose.orientation.w);

            return true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform bounding box: %s", ex.what());
            return false;
        }
    }

    void printTransform(const geometry_msgs::msg::TransformStamped& transform) {
        RCLCPP_INFO(this->get_logger(), "Translation: [%.3f, %.3f, %.3f]", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
        RCLCPP_INFO(this->get_logger(), "Rotation: [%.3f, %.3f, %.3f, %.3f]", transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
    }

    bool checkCollision(const visualization_msgs::msg::Marker& bounding_box) {
        // Convert Pose to TransformStamped
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header = bounding_box.header;
        transform_stamped.transform.translation.x = bounding_box.pose.position.x;
        transform_stamped.transform.translation.y = bounding_box.pose.position.y;
        transform_stamped.transform.translation.z = bounding_box.pose.position.z;
        transform_stamped.transform.rotation = bounding_box.pose.orientation;

        // Define the corners of the bounding box in its local frame
        std::vector<geometry_msgs::msg::Point> corners = getBoundingBoxCorners(bounding_box);

        // Transform the corners to the global (map) frame using the bounding box's transform
        for (auto& corner : corners) {
            tf2::doTransform(corner, corner, transform_stamped);
        }

        // Check each transformed corner against the clearance profile tube
        for (const auto& corner : corners) {
            if (isPointInsideTube(corner, clearance_profile_tube_)) {
                return true;  // Collision detected
            }
        }
        
        return false;  // No collision detected
    }

    std::vector<geometry_msgs::msg::Point> getBoundingBoxCorners(const visualization_msgs::msg::Marker& bounding_box) {
        std::vector<geometry_msgs::msg::Point> corners;
        double half_x = bounding_box.scale.x / 2.0;
        double half_y = bounding_box.scale.y / 2.0;
        double half_z = bounding_box.scale.z / 2.0;

        geometry_msgs::msg::Point p;
        
        // Add all eight corners
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
                if (distance < tube_marker.scale.x) {  // tube_marker.scale.x is used as the radius here
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
        // Assuming the LiDAR scanner is at the origin of the "lidar_frame"
        return sqrt(pow(marker.pose.position.x, 2) + pow(marker.pose.position.y, 2) + pow(marker.pose.position.z, 2));
    }

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr clearance_profile_subscription_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr bounding_boxes_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;  // New subscription
    rclcpp::Publisher<observer_msgs::msg::ObserverInfo>::SharedPtr observer_info_publisher_;  // New publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr filtered_bounding_boxes_publisher_;  // Re-introduced publisher

    visualization_msgs::msg::MarkerArray clearance_profile_tube_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Signal-related placeholders (you should replace these with your actual data handling)
    bool signal_detected_ = false;
    std::string signal_setting_ = "white";
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClearanceObserver>());
    rclcpp::shutdown();
    return 0;
}



// #include "rclcpp/rclcpp.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"
// #include "visualization_msgs/msg/marker.hpp"
// #include "std_msgs/msg/float64.hpp"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include <vector>
// #include <limits>  // For std::numeric_limits

// class ClearanceObserver : public rclcpp::Node {
// public:
//     ClearanceObserver()
//         : Node("clearance_observer"),
//           tf_buffer_(this->get_clock()),
//           tf_listener_(tf_buffer_) {

//         // Subscription to the clearance profile tube, which is in the map frame
//         clearance_profile_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
//             "clearance_profile_tubes", 10,
//             std::bind(&ClearanceObserver::clearanceProfileCallback, this, std::placeholders::_1));

//         // Subscription to the LiDAR bounding boxes, which are in the lidar_frame
//         bounding_boxes_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
//             "LiDAR_Clustering_BoundingBoxes", 10,
//             std::bind(&ClearanceObserver::boundingBoxesCallback, this, std::placeholders::_1));

//         // Publisher for the transformed bounding boxes
//         filtered_bounding_boxes_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
//             "filtered_bounding_boxes", 10);

//         // Publisher for the distance to the closest collision
//         distance_to_collision_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
//             "distance_to_collision", 10);
//     }

// private:
//     void clearanceProfileCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
//         clearance_profile_tube_ = *msg;
//     }

//     void boundingBoxesCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
//         visualization_msgs::msg::MarkerArray filtered_markers;
//         double min_distance = std::numeric_limits<double>::infinity();  // Start with infinity

//         for (const auto& marker : msg->markers) {
//             visualization_msgs::msg::Marker transformed_marker;
//             if (transformBoundingBoxToMapFrame(marker, transformed_marker)) {
//                 bool is_in_path = checkCollision(transformed_marker);

//                 // Set transparency (alpha) for the bounding boxes
//                 transformed_marker.color.a = 0.3; // 30% visibility by default

//                 if (is_in_path) {
//                     // Make the box red and see-through
//                     transformed_marker.color.r = 1.0;
//                     transformed_marker.color.g = 0.0;
//                     transformed_marker.color.b = 0.0;
//                     transformed_marker.color.a = 0.3; // 30% transparency for red boxes

//                     // Transform the red box to the lidar_frame
//                     visualization_msgs::msg::Marker lidar_frame_marker;
//                     if (transformBoundingBoxToLidarFrame(transformed_marker, lidar_frame_marker)) {
//                         // Calculate distance to the LiDAR scanner and update min_distance if necessary
//                         double distance = calculateDistanceToLidar(lidar_frame_marker);
//                         if (distance < min_distance) {
//                             min_distance = distance;
//                         }
//                     }
//                 } else {
//                     // Color in grey if not inside the path
//                     transformed_marker.color.r = 0.5;
//                     transformed_marker.color.g = 0.5;
//                     transformed_marker.color.b = 0.5;
//                     transformed_marker.color.a = 0.1; // 10% visibility for grey boxes
//                 }

//                 // Ensure the box is displayed as a solid cube
//                 transformed_marker.type = visualization_msgs::msg::Marker::CUBE;

//                 // Clear any points or lines that might be set inadvertently
//                 transformed_marker.points.clear();

//                 filtered_markers.markers.push_back(transformed_marker);
//             } else {
//                 RCLCPP_WARN(this->get_logger(), "Failed to transform bounding box.");
//             }
//         }

//         filtered_bounding_boxes_publisher_->publish(filtered_markers);

//         // Publish the minimum distance to collision (if any red boxes were found)
//         if (min_distance < std::numeric_limits<double>::infinity()) {
//             std_msgs::msg::Float64 distance_msg;
//             distance_msg.data = min_distance;
//             distance_to_collision_publisher_->publish(distance_msg);
//         }
//     }

//     bool transformBoundingBoxToMapFrame(const visualization_msgs::msg::Marker& marker, visualization_msgs::msg::Marker& transformed_marker) {
//         try {
//             // Step 1: Transform the entire marker from lidar_frame to first_person_view
//             geometry_msgs::msg::TransformStamped transform_stamped_to_fpv = tf_buffer_.lookupTransform("first_person_view", marker.header.frame_id, tf2::TimePointZero);

//             RCLCPP_INFO(this->get_logger(), "Transforming from %s to first_person_view", marker.header.frame_id.c_str());
//             printTransform(transform_stamped_to_fpv);

//             transformed_marker = marker;  // Copy the original marker
//             tf2::doTransform(marker.pose, transformed_marker.pose, transform_stamped_to_fpv);
//             transformed_marker.header.frame_id = "first_person_view";  // Update frame to first_person_view

//             RCLCPP_INFO(this->get_logger(), "After transforming to first_person_view:");
//             RCLCPP_INFO(this->get_logger(), "Pose: [%.3f, %.3f, %.3f] Rotation: [%.3f, %.3f, %.3f, %.3f]",
//                         transformed_marker.pose.position.x, transformed_marker.pose.position.y, transformed_marker.pose.position.z,
//                         transformed_marker.pose.orientation.x, transformed_marker.pose.orientation.y, 
//                         transformed_marker.pose.orientation.z, transformed_marker.pose.orientation.w);

//             // Step 2: Transform the marker from first_person_view to map
//             geometry_msgs::msg::TransformStamped transform_stamped_to_map = tf_buffer_.lookupTransform("map", "first_person_view", tf2::TimePointZero);

//             RCLCPP_INFO(this->get_logger(), "Transforming from first_person_view to map");
//             printTransform(transform_stamped_to_map);

//             tf2::doTransform(transformed_marker.pose, transformed_marker.pose, transform_stamped_to_map);
//             transformed_marker.header.frame_id = "map";  // Set the final frame_id to map

//             RCLCPP_INFO(this->get_logger(), "After transforming to map frame:");
//             RCLCPP_INFO(this->get_logger(), "Pose: [%.3f, %.3f, %.3f] Rotation: [%.3f, %.3f, %.3f, %.3f]",
//                         transformed_marker.pose.position.x, transformed_marker.pose.position.y, transformed_marker.pose.position.z,
//                         transformed_marker.pose.orientation.x, transformed_marker.pose.orientation.y, 
//                         transformed_marker.pose.orientation.z, transformed_marker.pose.orientation.w);

//             return true;
//         } catch (tf2::TransformException &ex) {
//             RCLCPP_WARN(this->get_logger(), "Could not transform bounding box: %s", ex.what());
//             return false;
//         }
//     }

//     bool transformBoundingBoxToLidarFrame(const visualization_msgs::msg::Marker& marker, visualization_msgs::msg::Marker& transformed_marker) {
//         try {
//             // Transform the marker from map to lidar_frame
//             geometry_msgs::msg::TransformStamped transform_stamped_to_lidar = tf_buffer_.lookupTransform("lidar_frame", "map", tf2::TimePointZero);

//             RCLCPP_INFO(this->get_logger(), "Transforming from map to lidar_frame");
//             printTransform(transform_stamped_to_lidar);

//             transformed_marker = marker;  // Copy the original marker
//             tf2::doTransform(marker.pose, transformed_marker.pose, transform_stamped_to_lidar);
//             transformed_marker.header.frame_id = "lidar_frame";  // Set the final frame_id to lidar_frame

//             RCLCPP_INFO(this->get_logger(), "After transforming to lidar_frame:");
//             RCLCPP_INFO(this->get_logger(), "Pose: [%.3f, %.3f, %.3f] Rotation: [%.3f, %.3f, %.3f, %.3f]",
//                         transformed_marker.pose.position.x, transformed_marker.pose.position.y, transformed_marker.pose.position.z,
//                         transformed_marker.pose.orientation.x, transformed_marker.pose.orientation.y, 
//                         transformed_marker.pose.orientation.z, transformed_marker.pose.orientation.w);

//             return true;
//         } catch (tf2::TransformException &ex) {
//             RCLCPP_WARN(this->get_logger(), "Could not transform bounding box: %s", ex.what());
//             return false;
//         }
//     }

//     void printTransform(const geometry_msgs::msg::TransformStamped& transform) {
//         RCLCPP_INFO(this->get_logger(), "Translation: [%.3f, %.3f, %.3f]", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
//         RCLCPP_INFO(this->get_logger(), "Rotation: [%.3f, %.3f, %.3f, %.3f]", transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
//     }

//     bool checkCollision(const visualization_msgs::msg::Marker& bounding_box) {
//         // Convert Pose to TransformStamped
//         geometry_msgs::msg::TransformStamped transform_stamped;
//         transform_stamped.header = bounding_box.header;
//         transform_stamped.transform.translation.x = bounding_box.pose.position.x;
//         transform_stamped.transform.translation.y = bounding_box.pose.position.y;
//         transform_stamped.transform.translation.z = bounding_box.pose.position.z;
//         transform_stamped.transform.rotation = bounding_box.pose.orientation;

//         // Define the corners of the bounding box in its local frame
//         std::vector<geometry_msgs::msg::Point> corners = getBoundingBoxCorners(bounding_box);

//         // Transform the corners to the global (map) frame using the bounding box's transform
//         for (auto& corner : corners) {
//             tf2::doTransform(corner, corner, transform_stamped);
//         }

//         // Check each transformed corner against the clearance profile tube
//         for (const auto& corner : corners) {
//             if (isPointInsideTube(corner, clearance_profile_tube_)) {
//                 return true;  // Collision detected
//             }
//         }
        
//         return false;  // No collision detected
//     }

//     std::vector<geometry_msgs::msg::Point> getBoundingBoxCorners(const visualization_msgs::msg::Marker& bounding_box) {
//         std::vector<geometry_msgs::msg::Point> corners;
//         double half_x = bounding_box.scale.x / 2.0;
//         double half_y = bounding_box.scale.y / 2.0;
//         double half_z = bounding_box.scale.z / 2.0;

//         geometry_msgs::msg::Point p;
        
//         // Add all eight corners
//         p.x = half_x; p.y = half_y; p.z = half_z; corners.push_back(p);
//         p.x = half_x; p.y = half_y; p.z = -half_z; corners.push_back(p);
//         p.x = half_x; p.y = -half_y; p.z = half_z; corners.push_back(p);
//         p.x = half_x; p.y = -half_y; p.z = -half_z; corners.push_back(p);
//         p.x = -half_x; p.y = half_y; p.z = half_z; corners.push_back(p);
//         p.x = -half_x; p.y = half_y; p.z = -half_z; corners.push_back(p);
//         p.x = -half_x; p.y = -half_y; p.z = half_z; corners.push_back(p);
//         p.x = -half_x; p.y = -half_y; p.z = -half_z; corners.push_back(p);

//         return corners;
//     }

//     bool isPointInsideTube(const geometry_msgs::msg::Point& point, const visualization_msgs::msg::MarkerArray& tube) {
//         for (const auto& tube_marker : tube.markers) {
//             for (size_t i = 0; i < tube_marker.points.size(); i += 2) {
//                 const auto& p1 = tube_marker.points[i];
//                 const auto& p2 = tube_marker.points[i + 1];

//                 double distance = pointLineDistance(point, p1, p2);
//                 if (distance < tube_marker.scale.x) {  // tube_marker.scale.x is used as the radius here
//                     return true;
//                 }
//             }
//         }
//         return false;
//     }

//     double pointLineDistance(const geometry_msgs::msg::Point& p, const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
//         double dx = p2.x - p1.x;
//         double dy = p2.y - p1.y;
//         double dz = p2.z - p1.z;
//         double length_squared = dx*dx + dy*dy + dz*dz;

//         if (length_squared == 0.0) {
//             dx = p.x - p1.x;
//             dy = p.y - p1.y;
//             dz = p.z - p1.z;
//             return sqrt(dx*dx + dy*dy + dz*dz);
//         }

//         double t = ((p.x - p1.x) * dx + (p.y - p1.y) * dy + (p.z - p1.z) * dz) / length_squared;
//         t = std::max(0.0, std::min(1.0, t));

//         geometry_msgs::msg::Point projection;
//         projection.x = p1.x + t * dx;
//         projection.y = p1.y + t * dy;
//         projection.z = p1.z + t * dz;

//         dx = p.x - projection.x;
//         dy = p.y - projection.y;
//         dz = p.z - projection.z;

//         return sqrt(dx*dx + dy*dy + dz*dz);
//     }

//     double calculateDistanceToLidar(const visualization_msgs::msg::Marker& marker) {
//         // Assuming the LiDAR scanner is at the origin of the "lidar_frame"
//         return sqrt(pow(marker.pose.position.x, 2) + pow(marker.pose.position.y, 2) + pow(marker.pose.position.z, 2));
//     }

//     rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr clearance_profile_subscription_;
//     rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr bounding_boxes_subscription_;
//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr filtered_bounding_boxes_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_to_collision_publisher_;  // New publisher

//     visualization_msgs::msg::MarkerArray clearance_profile_tube_;

//     tf2_ros::Buffer tf_buffer_;
//     tf2_ros::TransformListener tf_listener_;
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ClearanceObserver>());
//     rclcpp::shutdown();
//     return 0;
// }


// #include "rclcpp/rclcpp.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"
// #include "visualization_msgs/msg/marker.hpp"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include <vector>

// class ClearanceObserver : public rclcpp::Node {
// public:
//     ClearanceObserver()
//         : Node("clearance_observer"),
//           tf_buffer_(this->get_clock()),
//           tf_listener_(tf_buffer_) {

//         // Subscription to the clearance profile tube, which is in the map frame
//         clearance_profile_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
//             "clearance_profile_tubes", 10,
//             std::bind(&ClearanceObserver::clearanceProfileCallback, this, std::placeholders::_1));

//         // Subscription to the LiDAR bounding boxes, which are in the lidar_frame
//         bounding_boxes_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
//             "LiDAR_Clustering_BoundingBoxes", 10,
//             std::bind(&ClearanceObserver::boundingBoxesCallback, this, std::placeholders::_1));

//         // Publisher for the transformed bounding boxes
//         filtered_bounding_boxes_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
//             "filtered_bounding_boxes", 10);
//     }

// private:
//     void clearanceProfileCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
//         clearance_profile_tube_ = *msg;
//     }

//     void boundingBoxesCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
//         visualization_msgs::msg::MarkerArray filtered_markers;

//         for (const auto& marker : msg->markers) {
//             visualization_msgs::msg::Marker transformed_marker;
//             if (transformBoundingBoxToMapFrame(marker, transformed_marker)) {
//                 bool is_in_path = checkCollision(transformed_marker);

//                 // Set transparency (alpha) for the bounding boxes
//                 transformed_marker.color.a = 0.3; // 30% visibility by default

//                 if (is_in_path) {
//                     // Make the box red and see-through
//                     transformed_marker.color.r = 1.0;
//                     transformed_marker.color.g = 0.0;
//                     transformed_marker.color.b = 0.0;
//                     transformed_marker.color.a = 0.3; // 30% transparency for red boxes
//                 } else {
//                     // Color in grey if not inside the path
//                     transformed_marker.color.r = 0.5;
//                     transformed_marker.color.g = 0.5;
//                     transformed_marker.color.b = 0.5;
//                     transformed_marker.color.a = 0.1; // 10% visibility for grey boxes
//                 }

//                 // Ensure the box is displayed as a solid cube
//                 transformed_marker.type = visualization_msgs::msg::Marker::CUBE;

//                 // Clear any points or lines that might be set inadvertently
//                 transformed_marker.points.clear();

//                 filtered_markers.markers.push_back(transformed_marker);
//             } else {
//                 RCLCPP_WARN(this->get_logger(), "Failed to transform bounding box.");
//             }
//         }
//         filtered_bounding_boxes_publisher_->publish(filtered_markers);
//     }

//     bool transformBoundingBoxToMapFrame(const visualization_msgs::msg::Marker& marker, visualization_msgs::msg::Marker& transformed_marker) {
//         try {
//             // Step 1: Transform the entire marker from lidar_frame to first_person_view
//             geometry_msgs::msg::TransformStamped transform_stamped_to_fpv = tf_buffer_.lookupTransform("first_person_view", marker.header.frame_id, tf2::TimePointZero);

//             RCLCPP_INFO(this->get_logger(), "Transforming from %s to first_person_view", marker.header.frame_id.c_str());
//             printTransform(transform_stamped_to_fpv);

//             transformed_marker = marker;  // Copy the original marker
//             tf2::doTransform(marker.pose, transformed_marker.pose, transform_stamped_to_fpv);
//             transformed_marker.header.frame_id = "first_person_view";  // Update frame to first_person_view

//             RCLCPP_INFO(this->get_logger(), "After transforming to first_person_view:");
//             RCLCPP_INFO(this->get_logger(), "Pose: [%.3f, %.3f, %.3f] Rotation: [%.3f, %.3f, %.3f, %.3f]",
//                         transformed_marker.pose.position.x, transformed_marker.pose.position.y, transformed_marker.pose.position.z,
//                         transformed_marker.pose.orientation.x, transformed_marker.pose.orientation.y, 
//                         transformed_marker.pose.orientation.z, transformed_marker.pose.orientation.w);

//             // Step 2: Transform the marker from first_person_view to map
//             geometry_msgs::msg::TransformStamped transform_stamped_to_map = tf_buffer_.lookupTransform("map", "first_person_view", tf2::TimePointZero);

//             RCLCPP_INFO(this->get_logger(), "Transforming from first_person_view to map");
//             printTransform(transform_stamped_to_map);

//             tf2::doTransform(transformed_marker.pose, transformed_marker.pose, transform_stamped_to_map);
//             transformed_marker.header.frame_id = "map";  // Set the final frame_id to map

//             RCLCPP_INFO(this->get_logger(), "After transforming to map frame:");
//             RCLCPP_INFO(this->get_logger(), "Pose: [%.3f, %.3f, %.3f] Rotation: [%.3f, %.3f, %.3f, %.3f]",
//                         transformed_marker.pose.position.x, transformed_marker.pose.position.y, transformed_marker.pose.position.z,
//                         transformed_marker.pose.orientation.x, transformed_marker.pose.orientation.y, 
//                         transformed_marker.pose.orientation.z, transformed_marker.pose.orientation.w);

//             return true;
//         } catch (tf2::TransformException &ex) {
//             RCLCPP_WARN(this->get_logger(), "Could not transform bounding box: %s", ex.what());
//             return false;
//         }
//     }

//     void printTransform(const geometry_msgs::msg::TransformStamped& transform) {
//         RCLCPP_INFO(this->get_logger(), "Translation: [%.3f, %.3f, %.3f]", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
//         RCLCPP_INFO(this->get_logger(), "Rotation: [%.3f, %.3f, %.3f, %.3f]", transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
//     }

//     bool checkCollision(const visualization_msgs::msg::Marker& bounding_box) {
//         // Convert Pose to TransformStamped
//         geometry_msgs::msg::TransformStamped transform_stamped;
//         transform_stamped.header = bounding_box.header;
//         transform_stamped.transform.translation.x = bounding_box.pose.position.x;
//         transform_stamped.transform.translation.y = bounding_box.pose.position.y;
//         transform_stamped.transform.translation.z = bounding_box.pose.position.z;
//         transform_stamped.transform.rotation = bounding_box.pose.orientation;

//         // Define the corners of the bounding box in its local frame
//         std::vector<geometry_msgs::msg::Point> corners = getBoundingBoxCorners(bounding_box);

//         // Transform the corners to the global (map) frame using the bounding box's transform
//         for (auto& corner : corners) {
//             tf2::doTransform(corner, corner, transform_stamped);
//         }

//         // Check each transformed corner against the clearance profile tube
//         for (const auto& corner : corners) {
//             if (isPointInsideTube(corner, clearance_profile_tube_)) {
//                 return true;  // Collision detected
//             }
//         }
        
//         return false;  // No collision detected
//     }

//     std::vector<geometry_msgs::msg::Point> getBoundingBoxCorners(const visualization_msgs::msg::Marker& bounding_box) {
//         std::vector<geometry_msgs::msg::Point> corners;
//         double half_x = bounding_box.scale.x / 2.0;
//         double half_y = bounding_box.scale.y / 2.0;
//         double half_z = bounding_box.scale.z / 2.0;

//         geometry_msgs::msg::Point p;
        
//         // Add all eight corners
//         p.x = half_x; p.y = half_y; p.z = half_z; corners.push_back(p);
//         p.x = half_x; p.y = half_y; p.z = -half_z; corners.push_back(p);
//         p.x = half_x; p.y = -half_y; p.z = half_z; corners.push_back(p);
//         p.x = half_x; p.y = -half_y; p.z = -half_z; corners.push_back(p);
//         p.x = -half_x; p.y = half_y; p.z = half_z; corners.push_back(p);
//         p.x = -half_x; p.y = half_y; p.z = -half_z; corners.push_back(p);
//         p.x = -half_x; p.y = -half_y; p.z = half_z; corners.push_back(p);
//         p.x = -half_x; p.y = -half_y; p.z = -half_z; corners.push_back(p);

//         return corners;
//     }

//     bool isPointInsideTube(const geometry_msgs::msg::Point& point, const visualization_msgs::msg::MarkerArray& tube) {
//         for (const auto& tube_marker : tube.markers) {
//             for (size_t i = 0; i < tube_marker.points.size(); i += 2) {
//                 const auto& p1 = tube_marker.points[i];
//                 const auto& p2 = tube_marker.points[i + 1];

//                 double distance = pointLineDistance(point, p1, p2);
//                 if (distance < tube_marker.scale.x) {  // tube_marker.scale.x is used as the radius here
//                     return true;
//                 }
//             }
//         }
//         return false;
//     }

//     double pointLineDistance(const geometry_msgs::msg::Point& p, const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
//         double dx = p2.x - p1.x;
//         double dy = p2.y - p1.y;
//         double dz = p2.z - p1.z;
//         double length_squared = dx*dx + dy*dy + dz*dz;

//         if (length_squared == 0.0) {
//             dx = p.x - p1.x;
//             dy = p.y - p1.y;
//             dz = p.z - p1.z;
//             return sqrt(dx*dx + dy*dy + dz*dz);
//         }

//         double t = ((p.x - p1.x) * dx + (p.y - p1.y) * dy + (p.z - p1.z) * dz) / length_squared;
//         t = std::max(0.0, std::min(1.0, t));

//         geometry_msgs::msg::Point projection;
//         projection.x = p1.x + t * dx;
//         projection.y = p1.y + t * dy;
//         projection.z = p1.z + t * dz;

//         dx = p.x - projection.x;
//         dy = p.y - projection.y;
//         dz = p.z - projection.z;

//         return sqrt(dx*dx + dy*dy + dz*dz);
//     }

//     rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr clearance_profile_subscription_;
//     rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr bounding_boxes_subscription_;
//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr filtered_bounding_boxes_publisher_;

//     visualization_msgs::msg::MarkerArray clearance_profile_tube_;

//     tf2_ros::Buffer tf_buffer_;
//     tf2_ros::TransformListener tf_listener_;
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ClearanceObserver>());
//     rclcpp::shutdown();
//     return 0;
// }


// #include <memory>
// #include <vector>
// #include <string>

// #include <rclcpp/rclcpp.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// class ClearanceProfileObserver : public rclcpp::Node
// {
// public:
//     ClearanceProfileObserver()
//         : Node("clearance_profile_observer"),
//           tf_buffer_(this->get_clock()),
//           tf_listener_(tf_buffer_)
//     {
//         // Subscription for LiDAR clustering bounding boxes
//         bbox_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
//             "LiDAR_Clustering_BoundingBoxes", 10, std::bind(&ClearanceProfileObserver::bbox_callback, this, std::placeholders::_1));
        
//         // Publisher for transformed bounding boxes
//         bbox_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
//             "BoundingBoxesInFirstPersonView", 10);

//         // Subscription for highlighted driving path
//         highlighted_path_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
//             "highlighted_driving_path", 10, std::bind(&ClearanceProfileObserver::highlighted_path_callback, this, std::placeholders::_1));

//         // Publisher for transformed highlighted driving path
//         highlighted_path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
//             "TransformedHighlightedDrivingPath", 10);

//         // Subscription for clearance profile tubes
//         tube_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
//             "ClearanceProfile_Tube", 10, std::bind(&ClearanceProfileObserver::tube_callback, this, std::placeholders::_1));

//         // Publisher for transformed clearance profile tubes
//         tube_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
//             "TransformedClearanceProfileTube", 10);
//     }

// private:
//     void bbox_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
//     {
//         RCLCPP_INFO(this->get_logger(), "Received MarkerArray with %zu markers", msg->markers.size());

//         transformed_bboxes_ = transform_markers_to_fpv(msg, "corrected_first_person_view");

//         if (!transformed_bboxes_.empty())
//         {
//             visualization_msgs::msg::MarkerArray bbox_array;
//             bbox_array.markers = transformed_bboxes_;
//             bbox_publisher_->publish(bbox_array);
//             RCLCPP_INFO(this->get_logger(), "Published %zu transformed markers", transformed_bboxes_.size());

//             check_intersections();
//         }
//         else
//         {
//             RCLCPP_WARN(this->get_logger(), "No markers were transformed.");
//         }
//     }

//     void highlighted_path_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
//     {
//         RCLCPP_INFO(this->get_logger(), "Received Highlighted Driving Path with %zu markers", msg->markers.size());

//         transformed_highlighted_path_ = transform_markers_to_fpv(msg, "corrected_first_person_view");

//         if (!transformed_highlighted_path_.empty())
//         {
//             visualization_msgs::msg::MarkerArray marker_array;
//             marker_array.markers = transformed_highlighted_path_;
//             highlighted_path_publisher_->publish(marker_array);
//             RCLCPP_INFO(this->get_logger(), "Published %zu transformed highlighted path markers", transformed_highlighted_path_.size());

//             check_intersections();
//         }
//         else
//         {
//             RCLCPP_WARN(this->get_logger(), "No highlighted path markers were transformed.");
//         }
//     }

//     void tube_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
//     {
//         RCLCPP_INFO(this->get_logger(), "Received Clearance Profile Tube with %zu markers", msg->markers.size());

//         transformed_tubes_ = transform_markers_to_fpv(msg, "corrected_first_person_view");

//         if (!transformed_tubes_.empty())
//         {
//             visualization_msgs::msg::MarkerArray tube_array;
//             tube_array.markers = transformed_tubes_;
//             tube_publisher_->publish(tube_array);
//             RCLCPP_INFO(this->get_logger(), "Published %zu transformed tube markers", transformed_tubes_.size());

//             check_intersections();
//         }
//         else
//         {
//             RCLCPP_WARN(this->get_logger(), "No tube markers were transformed.");
//         }
//     }

//     std::vector<visualization_msgs::msg::Marker> transform_markers_to_fpv(const visualization_msgs::msg::MarkerArray::SharedPtr &marker_array, const std::string &target_frame)
//     {
//         std::vector<visualization_msgs::msg::Marker> transformed_markers;

//         for (const auto &marker : marker_array->markers)
//         {
//             geometry_msgs::msg::TransformStamped transform;
//             try {
//                 transform = tf_buffer_.lookupTransform(target_frame, marker.header.frame_id, rclcpp::Time(0));

//                 geometry_msgs::msg::PoseStamped pose_in;
//                 pose_in.header.frame_id = marker.header.frame_id;
//                 pose_in.pose.position = marker.pose.position;
//                 pose_in.pose.orientation = marker.pose.orientation;

//                 geometry_msgs::msg::PoseStamped pose_out;
//                 tf2::doTransform(pose_in, pose_out, transform);

//                 visualization_msgs::msg::Marker transformed_marker = marker;
//                 transformed_marker.pose.position = pose_out.pose.position;
//                 transformed_marker.pose.orientation = pose_out.pose.orientation;
//                 transformed_marker.header.frame_id = target_frame;

//                 transformed_markers.push_back(transformed_marker);
//             } catch (tf2::TransformException &ex) {
//                 RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
//             }
//         }

//         return transformed_markers;
//     }

//     void check_intersections()
//     {
//         if (transformed_bboxes_.empty() || transformed_tubes_.empty())
//         {
//             return;
//         }

//         visualization_msgs::msg::MarkerArray updated_tubes;
//         for (auto &tube_marker : transformed_tubes_)
//         {
//             bool intersects = false;

//             for (const auto &bbox : transformed_bboxes_)
//             {
//                 if (check_intersection(bbox, tube_marker))
//                 {
//                     intersects = true;
//                     break;
//                 }
//             }

//             if (intersects)
//             {
//                 tube_marker.color.r = 1.0; // Red
//                 tube_marker.color.g = 0.0;
//                 tube_marker.color.b = 0.0;
//                 tube_marker.color.a = 1.0;
//             }

//             updated_tubes.markers.push_back(tube_marker);
//         }

//         tube_publisher_->publish(updated_tubes);
//     }

//     bool check_intersection(const visualization_msgs::msg::Marker &bbox, const visualization_msgs::msg::Marker &tube)
//     {
//         // Axis-aligned bounding box (AABB) intersection check
//         double bbox_min_x = bbox.pose.position.x - bbox.scale.x / 2.0;
//         double bbox_max_x = bbox.pose.position.x + bbox.scale.x / 2.0;
//         double bbox_min_y = bbox.pose.position.y - bbox.scale.y / 2.0;
//         double bbox_max_y = bbox.pose.position.y + bbox.scale.y / 2.0;
//         double bbox_min_z = bbox.pose.position.z - bbox.scale.z / 2.0;
//         double bbox_max_z = bbox.pose.position.z + bbox.scale.z / 2.0;

//         double tube_min_x = tube.pose.position.x - tube.scale.x / 2.0;
//         double tube_max_x = tube.pose.position.x + tube.scale.x / 2.0;
//         double tube_min_y = tube.pose.position.y - tube.scale.y / 2.0;
//         double tube_max_y = tube.pose.position.y + tube.scale.y / 2.0;
//         double tube_min_z = tube.pose.position.z - tube.scale.z / 2.0;
//         double tube_max_z = tube.pose.position.z + tube.scale.z / 2.0;

//         bool x_intersect = (bbox_min_x <= tube_max_x) && (bbox_max_x >= tube_min_x);
//         bool y_intersect = (bbox_min_y <= tube_max_y) && (bbox_max_y >= tube_min_y);
//         bool z_intersect = (bbox_min_z <= tube_max_z) && (bbox_max_z >= tube_min_z);

//         return x_intersect && y_intersect && z_intersect;
//     }

//     tf2_ros::Buffer tf_buffer_;
//     tf2_ros::TransformListener tf_listener_;

//     rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_subscription_;
//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_publisher_;

//     rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr highlighted_path_subscription_;
//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr highlighted_path_publisher_;

//     rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr tube_subscription_;
//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tube_publisher_;

//     std::vector<visualization_msgs::msg::Marker> transformed_bboxes_;
//     std::vector<visualization_msgs::msg::Marker> transformed_highlighted_path_;
//     std::vector<visualization_msgs::msg::Marker> transformed_tubes_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ClearanceProfileObserver>());
//     rclcpp::shutdown();
//     return 0;
// }


// // #include <memory>
// // #include <vector>
// // #include <string>

// // #include <rclcpp/rclcpp.hpp>
// // #include <visualization_msgs/msg/marker_array.hpp>
// // #include <tf2_ros/buffer.h>
// // #include <tf2_ros/transform_listener.h>
// // #include <geometry_msgs/msg/pose_stamped.hpp>
// // #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// // class ClearanceProfileObserver : public rclcpp::Node
// // {
// // public:
// //     ClearanceProfileObserver()
// //         : Node("clearance_profile_observer"),
// //           tf_buffer_(this->get_clock()),
// //           tf_listener_(tf_buffer_)
// //     {
// //         // Subscription for LiDAR clustering bounding boxes
// //         bbox_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
// //             "LiDAR_Clustering_BoundingBoxes", 10, std::bind(&ClearanceProfileObserver::bbox_callback, this, std::placeholders::_1));
        
// //         // Publisher for transformed bounding boxes
// //         bbox_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
// //             "BoundingBoxesInFirstPersonView", 10);

// //         // Subscription for highlighted driving path
// //         highlighted_path_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
// //             "highlighted_driving_path", 10, std::bind(&ClearanceProfileObserver::highlighted_path_callback, this, std::placeholders::_1));

// //         // Publisher for transformed highlighted driving path
// //         highlighted_path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
// //             "TransformedHighlightedDrivingPath", 10);

// //         // Subscription for clearance profile tubes
// //         tube_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
// //             "ClearanceProfile_Tube", 10, std::bind(&ClearanceProfileObserver::tube_callback, this, std::placeholders::_1));

// //         // Publisher for transformed clearance profile tubes
// //         tube_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
// //             "TransformedClearanceProfileTube", 10);
// //     }

// // private:
// //     void bbox_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
// //     {
// //         RCLCPP_INFO(this->get_logger(), "Received MarkerArray with %zu markers", msg->markers.size());

// //         transformed_bboxes_ = transform_markers_to_fpv(msg, "corrected_first_person_view");

// //         if (!transformed_bboxes_.empty())
// //         {
// //             visualization_msgs::msg::MarkerArray bbox_array;
// //             bbox_array.markers = transformed_bboxes_;
// //             bbox_publisher_->publish(bbox_array);
// //             RCLCPP_INFO(this->get_logger(), "Published %zu transformed markers", transformed_bboxes_.size());

// //             check_intersections();
// //         }
// //         else
// //         {
// //             RCLCPP_WARN(this->get_logger(), "No markers were transformed.");
// //         }
// //     }

// //     void highlighted_path_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
// //     {
// //         RCLCPP_INFO(this->get_logger(), "Received Highlighted Driving Path with %zu markers", msg->markers.size());

// //         transformed_highlighted_path_ = transform_markers_to_fpv(msg, "corrected_first_person_view");

// //         if (!transformed_highlighted_path_.empty())
// //         {
// //             visualization_msgs::msg::MarkerArray marker_array;
// //             marker_array.markers = transformed_highlighted_path_;
// //             highlighted_path_publisher_->publish(marker_array);
// //             RCLCPP_INFO(this->get_logger(), "Published %zu transformed highlighted path markers", transformed_highlighted_path_.size());

// //             check_intersections();
// //         }
// //         else
// //         {
// //             RCLCPP_WARN(this->get_logger(), "No highlighted path markers were transformed.");
// //         }
// //     }

// //     void tube_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
// //     {
// //         RCLCPP_INFO(this->get_logger(), "Received Clearance Profile Tube with %zu markers", msg->markers.size());

// //         transformed_tubes_ = transform_markers_to_fpv(msg, "corrected_first_person_view");

// //         if (!transformed_tubes_.empty())
// //         {
// //             visualization_msgs::msg::MarkerArray tube_array;
// //             tube_array.markers = transformed_tubes_;
// //             tube_publisher_->publish(tube_array);
// //             RCLCPP_INFO(this->get_logger(), "Published %zu transformed tube markers", transformed_tubes_.size());

// //             check_intersections();
// //         }
// //         else
// //         {
// //             RCLCPP_WARN(this->get_logger(), "No tube markers were transformed.");
// //         }
// //     }

// //     std::vector<visualization_msgs::msg::Marker> transform_markers_to_fpv(const visualization_msgs::msg::MarkerArray::SharedPtr &marker_array, const std::string &target_frame)
// //     {
// //         std::vector<visualization_msgs::msg::Marker> transformed_markers;

// //         for (const auto &marker : marker_array->markers)
// //         {
// //             geometry_msgs::msg::TransformStamped transform;
// //             try {
// //                 transform = tf_buffer_.lookupTransform(target_frame, marker.header.frame_id, rclcpp::Time(0));

// //                 geometry_msgs::msg::PoseStamped pose_in;
// //                 pose_in.header.frame_id = marker.header.frame_id;
// //                 pose_in.pose.position = marker.pose.position;
// //                 pose_in.pose.orientation = marker.pose.orientation;

// //                 geometry_msgs::msg::PoseStamped pose_out;
// //                 tf2::doTransform(pose_in, pose_out, transform);

// //                 visualization_msgs::msg::Marker transformed_marker = marker;
// //                 transformed_marker.pose.position = pose_out.pose.position;
// //                 transformed_marker.pose.orientation = pose_out.pose.orientation;
// //                 transformed_marker.header.frame_id = target_frame;

// //                 transformed_markers.push_back(transformed_marker);
// //             } catch (tf2::TransformException &ex) {
// //                 RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
// //             }
// //         }

// //         return transformed_markers;
// //     }

// //     void check_intersections()
// //     {
// //         if (transformed_bboxes_.empty() || transformed_tubes_.empty())
// //         {
// //             return;
// //         }

// //         visualization_msgs::msg::MarkerArray updated_tubes;
// //         for (auto &tube_marker : transformed_tubes_)
// //         {
// //             bool intersects = false;

// //             for (const auto &bbox : transformed_bboxes_)
// //             {
// //                 if (check_intersection(bbox, tube_marker))
// //                 {
// //                     intersects = true;
// //                     break;
// //                 }
// //             }

// //             if (intersects)
// //             {
// //                 tube_marker.color.r = 1.0; // Red
// //                 tube_marker.color.g = 0.0;
// //                 tube_marker.color.b = 0.0;
// //                 tube_marker.color.a = 1.0;
// //             }

// //             updated_tubes.markers.push_back(tube_marker);
// //         }

// //         tube_publisher_->publish(updated_tubes);
// //     }

// //     bool check_intersection(const visualization_msgs::msg::Marker &bbox, const visualization_msgs::msg::Marker &tube)
// //     {
// //         // Axis-aligned bounding box (AABB) intersection check
// //         double bbox_min_x = bbox.pose.position.x - bbox.scale.x / 2.0;
// //         double bbox_max_x = bbox.pose.position.x + bbox.scale.x / 2.0;
// //         double bbox_min_y = bbox.pose.position.y - bbox.scale.y / 2.0;
// //         double bbox_max_y = bbox.pose.position.y + bbox.scale.y / 2.0;
// //         double bbox_min_z = bbox.pose.position.z - bbox.scale.z / 2.0;
// //         double bbox_max_z = bbox.pose.position.z + bbox.scale.z / 2.0;

// //         double tube_min_x = tube.pose.position.x - tube.scale.x / 2.0;
// //         double tube_max_x = tube.pose.position.x + tube.scale.x / 2.0;
// //         double tube_min_y = tube.pose.position.y - tube.scale.y / 2.0;
// //         double tube_max_y = tube.pose.position.y + tube.scale.y / 2.0;
// //         double tube_min_z = tube.pose.position.z - tube.scale.z / 2.0;
// //         double tube_max_z = tube.pose.position.z + tube.scale.z / 2.0;

// //         bool x_intersect = (bbox_min_x <= tube_max_x) && (bbox_max_x >= tube_min_x);
// //         bool y_intersect = (bbox_min_y <= tube_max_y) && (bbox_max_y >= tube_min_y);
// //         bool z_intersect = (bbox_min_z <= tube_max_z) && (bbox_max_z >= tube_min_z);

// //         return x_intersect && y_intersect && z_intersect;
// //     }

// //     tf2_ros::Buffer tf_buffer_;
// //     tf2_ros::TransformListener tf_listener_;

// //     rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_subscription_;
// //     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_publisher_;

// //     rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr highlighted_path_subscription_;
// //     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr highlighted_path_publisher_;

// //     rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr tube_subscription_;
// //     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tube_publisher_;

// //     std::vector<visualization_msgs::msg::Marker> transformed_bboxes_;
// //     std::vector<visualization_msgs::msg::Marker> transformed_highlighted_path_;
// //     std::vector<visualization_msgs::msg::Marker> transformed_tubes_;
// // };

// // int main(int argc, char *argv[])
// // {
// //     rclcpp::init(argc, argv);
// //     rclcpp::spin(std::make_shared<ClearanceProfileObserver>());
// //     rclcpp::shutdown();
// //     return 0;
// // }



// // #include <memory>
// // #include <vector>
// // #include <string>

// // #include <rclcpp/rclcpp.hpp>
// // #include <visualization_msgs/msg/marker_array.hpp>
// // #include <tf2_ros/buffer.h>
// // #include <tf2_ros/transform_listener.h>
// // #include <geometry_msgs/msg/pose_stamped.hpp>
// // #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// // class ClearanceProfileObserver : public rclcpp::Node
// // {
// // public:
// //     ClearanceProfileObserver()
// //         : Node("clearance_profile_observer"),
// //           tf_buffer_(this->get_clock()),
// //           tf_listener_(tf_buffer_)
// //     {
// //         // Subscription for LiDAR clustering bounding boxes
// //         subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
// //             "LiDAR_Clustering_BoundingBoxes", 10, std::bind(&ClearanceProfileObserver::bbox_callback, this, std::placeholders::_1));
        
// //         // Publisher for transformed bounding boxes
// //         publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
// //             "BoundingBoxesInFirstPersonView", 10);

// //         // Subscription for highlighted driving path
// //         highlighted_path_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
// //             "highlighted_driving_path", 10, std::bind(&ClearanceProfileObserver::highlighted_path_callback, this, std::placeholders::_1));

// //         // Publisher for transformed highlighted driving path
// //         highlighted_path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
// //             "TransformedHighlightedDrivingPath", 10);
// //     }

// // private:
// //     void bbox_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
// //     {
// //         RCLCPP_INFO(this->get_logger(), "Received MarkerArray with %zu markers", msg->markers.size());

// //         auto transformed_bboxes = transform_bboxes_to_fpv(msg);

// //         if (!transformed_bboxes.empty())
// //         {
// //             visualization_msgs::msg::MarkerArray bbox_array;
// //             bbox_array.markers = transformed_bboxes;
// //             publisher_->publish(bbox_array);
// //             RCLCPP_INFO(this->get_logger(), "Published %zu transformed markers", transformed_bboxes.size());
// //         }
// //         else
// //         {
// //             RCLCPP_WARN(this->get_logger(), "No markers were transformed.");
// //         }
// //     }

// //     void highlighted_path_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
// //     {
// //         RCLCPP_INFO(this->get_logger(), "Received Highlighted Driving Path with %zu markers", msg->markers.size());

// //         auto transformed_markers = transform_markers_to_fpv(msg);

// //         if (!transformed_markers.empty())
// //         {
// //             visualization_msgs::msg::MarkerArray marker_array;
// //             marker_array.markers = transformed_markers;
// //             highlighted_path_publisher_->publish(marker_array);
// //             RCLCPP_INFO(this->get_logger(), "Published %zu transformed highlighted path markers", transformed_markers.size());
// //         }
// //         else
// //         {
// //             RCLCPP_WARN(this->get_logger(), "No highlighted path markers were transformed.");
// //         }
// //     }

// //     std::vector<visualization_msgs::msg::Marker> transform_bboxes_to_fpv(const visualization_msgs::msg::MarkerArray::SharedPtr &bbox_array)
// //     {
// //         std::vector<visualization_msgs::msg::Marker> transformed_bboxes;

// //         for (const auto &marker : bbox_array->markers)
// //         {
// //             geometry_msgs::msg::TransformStamped transform;
// //             try {
// //                 transform = tf_buffer_.lookupTransform("first_person_view", marker.header.frame_id, rclcpp::Time(0));

// //                 geometry_msgs::msg::PoseStamped pose_in;
// //                 pose_in.header.frame_id = marker.header.frame_id;
// //                 pose_in.pose.position = marker.pose.position;
// //                 pose_in.pose.orientation = marker.pose.orientation;

// //                 geometry_msgs::msg::PoseStamped pose_out;
// //                 tf2::doTransform(pose_in, pose_out, transform);

// //                 visualization_msgs::msg::Marker transformed_marker = marker;
// //                 transformed_marker.pose.position = pose_out.pose.position;
// //                 transformed_marker.pose.orientation = pose_out.pose.orientation;
// //                 transformed_marker.header.frame_id = "first_person_view";

// //                 transformed_bboxes.push_back(transformed_marker);
// //             } catch (tf2::TransformException &ex) {
// //                 RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
// //             }
// //         }

// //         return transformed_bboxes;
// //     }

// //     std::vector<visualization_msgs::msg::Marker> transform_markers_to_fpv(const visualization_msgs::msg::MarkerArray::SharedPtr &marker_array)
// //     {
// //         std::vector<visualization_msgs::msg::Marker> transformed_markers;

// //         for (const auto &marker : marker_array->markers)
// //         {
// //             geometry_msgs::msg::TransformStamped transform;
// //             try {
// //                 // Transform from "map" frame to "first_person_view"
// //                 transform = tf_buffer_.lookupTransform("first_person_view", "map", rclcpp::Time(0));

// //                 geometry_msgs::msg::PoseStamped pose_in;
// //                 pose_in.header.frame_id = "map";
// //                 pose_in.pose.position = marker.pose.position;
// //                 pose_in.pose.orientation = marker.pose.orientation;

// //                 geometry_msgs::msg::PoseStamped pose_out;
// //                 tf2::doTransform(pose_in, pose_out, transform);

// //                 visualization_msgs::msg::Marker transformed_marker = marker;
// //                 transformed_marker.pose.position = pose_out.pose.position;
// //                 transformed_marker.pose.orientation = pose_out.pose.orientation;
// //                 transformed_marker.header.frame_id = "first_person_view";

// //                 transformed_markers.push_back(transformed_marker);
// //             } catch (tf2::TransformException &ex) {
// //                 RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
// //             }
// //         }

// //         return transformed_markers;
// //     }

// //     tf2_ros::Buffer tf_buffer_;
// //     tf2_ros::TransformListener tf_listener_;

// //     rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
// //     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;

// //     rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr highlighted_path_subscription_;
// //     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr highlighted_path_publisher_;
// // };

// // int main(int argc, char *argv[])
// // {
// //     rclcpp::init(argc, argv);
// //     rclcpp::spin(std::make_shared<ClearanceProfileObserver>());
// //     rclcpp::shutdown();
// //     return 0;
// // }
