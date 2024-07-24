#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace std::chrono_literals;

// Structure to store coordinates
struct Coordinate {
    double longitude;
    double latitude;
    double elevation;
};

// Main processing class
class TrackProcessor : public rclcpp::Node {
public:
    TrackProcessor() 
        : Node("track_processor") {
        // Initialize publisher for marker array
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

        // Load track data from CSV files
        readCSVFiles();

        // Create a timer to call the publishTrackLines function after 10 seconds
        marker_timer_ = this->create_wall_timer(
            10s, std::bind(&TrackProcessor::publishTrackLines, this));
    }

private:
    // Function to read CSV files
    void readCSVFiles() {
        for (int i = 101; i <= 118; ++i) {
            std::string filename = "src/nodes/yard_describer/src/MapData/track_" + std::to_string(i) + ".csv";
            std::ifstream file(filename);

            if (!file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
                continue;
            }

            std::string line;
            std::getline(file, line);  // Skip the header line

            std::vector<geometry_msgs::msg::Point> track_points;
            while (std::getline(file, line)) {
                std::stringstream ss(line);
                std::string token;
                Coordinate coord;

                std::getline(ss, token, ';');
                coord.longitude = std::stod(token);

                std::getline(ss, token, ';');
                coord.latitude = std::stod(token);

                std::getline(ss, token, ';');
                coord.elevation = std::stod(token);

                geometry_msgs::msg::Point point = transformToMetric(coord); // Transform coordinates to metric
                track_points.emplace_back(point);
            }
            file.close();

            tracks_.emplace_back(track_points);
        }
    }

    // Transform geographic coordinates to UTM and return as geometry_msgs::msg::Point
    geometry_msgs::msg::Point transformToMetric(const Coordinate& coord) {
        double x, y;
        int zone;
        bool northp;
        GeographicLib::UTMUPS::Forward(coord.latitude * 1e-7, coord.longitude * 1e-7, zone, northp, x, y);
        RCLCPP_INFO(this->get_logger(), "UTM coordinates: Zone %d, %s, X: %f, Y: %f",
                    zone, northp ? "North" : "South", x, y);
        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = 0; // Ignore elevation and set it to 0
        return point;
    }

    // Publish the track lines as markers
    void publishTrackLines() {
        if (tracks_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No track points to publish.");
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < tracks_.size(); ++i) {
            visualization_msgs::msg::Marker line_strip;
            line_strip.header.stamp = this->get_clock()->now();
            line_strip.header.frame_id = "map";
            line_strip.ns = "track_lines";
            line_strip.id = static_cast<int>(i);
            line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_strip.action = visualization_msgs::msg::Marker::ADD;

            line_strip.scale.x = 0.2;  // Line width

            line_strip.color.a = 1.0;
            line_strip.color.r = 1.0;
            line_strip.color.g = 0.0;
            line_strip.color.b = 0.0;

            for (const auto& point : tracks_[i]) {
                line_strip.points.push_back(point);
            }

            marker_array.markers.push_back(line_strip);
        }

        marker_pub_->publish(marker_array);

        RCLCPP_INFO(this->get_logger(), "Published %ld track lines as markers.", tracks_.size());
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr marker_timer_;
    std::vector<std::vector<geometry_msgs::msg::Point>> tracks_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackProcessor>());
    rclcpp::shutdown();
    return 0;
}
