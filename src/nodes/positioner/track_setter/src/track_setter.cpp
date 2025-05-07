#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

class TrackSetter : public rclcpp::Node {
public:
    TrackSetter() 
        : Node("track_setter"), selected_track_id_(1) {
        marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "yard_topology", 10, std::bind(&TrackSetter::markerCallback, this, std::placeholders::_1));
        
        order_sub_ = this->create_subscription<std_msgs::msg::String>(
            "order_info", 10, std::bind(&TrackSetter::orderCallback, this, std::placeholders::_1));

        highlighted_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("highlighted_driving_path", 10);

        RCLCPP_INFO(this->get_logger(), "TrackSetter node has been initialized.");
    }

private:
    void markerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        markers_ = msg;
        publishTrackLine();
    }

    void orderCallback(const std_msgs::msg::String::SharedPtr msg) {
        try {
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

                selected_track_id_ = std::stoi(target_pos_str);
                //RCLCPP_INFO(this->get_logger(), "Selected Track ID set to: %d", selected_track_id_);
                publishTrackLine();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Target Position not found in the order message");
            }
        } catch (const std::exception &e) {
           // RCLCPP_ERROR(this->get_logger(), "Error processing order info: %s", e.what());
        }
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
            delete_marker.ns = "track_lines";
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

    clearAllMarkers();
     // Sleep for a short time to ensure the old markers are processed before publishing new ones
    std::this_thread::sleep_for(100ms);

    visualization_msgs::msg::MarkerArray highlighted_marker_array;
    for (const auto& marker : markers_->markers) {
       // RCLCPP_INFO(this->get_logger(), "Checking Marker ID: %d, Points: %zu", marker.id, marker.points.size());

        if (marker.id == selected_track_id_) {
            if (marker.points.empty()) {
                RCLCPP_WARN(this->get_logger(), "Selected marker has no points!");
                continue;
            }
            visualization_msgs::msg::Marker highlighted_marker = marker;
            highlighted_marker.color.r = 1.0;
            highlighted_marker.color.g = 0.843;
            highlighted_marker.color.b = 0.0;
            highlighted_marker.color.a = 1.0;
            highlighted_marker_array.markers.push_back(highlighted_marker);
           // RCLCPP_INFO(this->get_logger(), "Highlighting Marker ID: %d with %zu points", marker.id, marker.points.size());
        }
    }

    if (highlighted_marker_array.markers.empty()) {
        RCLCPP_WARN(this->get_logger(), "No marker found for the selected track ID.");
        return;
    }

    highlighted_marker_pub_->publish(highlighted_marker_array);
}


    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr order_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr highlighted_marker_pub_;
    visualization_msgs::msg::MarkerArray::SharedPtr markers_;
    int selected_track_id_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackSetter>());
    rclcpp::shutdown();
    return 0;
}


