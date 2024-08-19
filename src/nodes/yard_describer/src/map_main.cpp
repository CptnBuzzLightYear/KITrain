

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
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("yard_topology", 10);

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

                // Read longitude and divide by 10^6 to convert to degrees
                std::getline(ss, token, ';');
                coord.longitude = std::stod(token) * 1e-6;

                // Read latitude and divide by 10^6 to convert to degrees
                std::getline(ss, token, ';');
                coord.latitude = std::stod(token) * 1e-6;

                // Read elevation directly
                std::getline(ss, token, ';');
                coord.elevation = std::stod(token);

                // Transform coordinates to metric (UTM)
                geometry_msgs::msg::Point point = transformToMetric(coord);
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

        // Convert geographic coordinates to UTM
        GeographicLib::UTMUPS::Forward(coord.latitude, coord.longitude, zone, northp, x, y);

        RCLCPP_INFO(this->get_logger(), "UTM coordinates: Zone %d, %s, X: %f, Y: %f",
                    zone, northp ? "North" : "South", x, y);

        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = 0; // coord.elevation; uncomment to include elevation in the point
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
            if (i<2){
            line_strip.id = static_cast<int>(i) + 1+100; //i von 0-14, tracks von 101-118
            }
            if(i>=2 && i<8){
            line_strip.id = static_cast<int>(i) + 1+2+100; // track 103 and 104 are missing
            }
            if(i>=8 && i<=9){
            line_strip.id = static_cast<int>(i) + 1+2+1+100; // track 111 is missing
            }
            if(i>=10){
            line_strip.id = static_cast<int>(i) + 1+2+1+1+100; // track 114 is missing
            }
            line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_strip.action = visualization_msgs::msg::Marker::ADD;

            line_strip.scale.x = 0.2;  // Line width

            // Color configuration, Set lines in grey
                line_strip.color.r = 0.5;
                line_strip.color.g = 0.5;
                line_strip.color.b = 0.5;

            line_strip.color.a = 0.5; // Fully opaque

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


// #include <rclcpp/rclcpp.hpp>
// #include <GeographicLib/UTMUPS.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <fstream>
// #include <sstream>
// #include <vector>
// #include <string>
// #include <cmath>
// #include <algorithm>

// using namespace std::chrono_literals;

// // Structure to store coordinates
// struct Coordinate {
//     double longitude;
//     double latitude;
//     double elevation;
// };

// // Main processing class
// class TrackProcessor : public rclcpp::Node {
// public:
//     TrackProcessor() 
//         : Node("track_processor") {
//         // Initialize publisher for marker array
//         marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("yard_topology", 10);

//         // Load track data from CSV files
//         readCSVFiles();

//         // Create a timer to call the publishTrackLines function after 10 seconds
//         marker_timer_ = this->create_wall_timer(
//             10s, std::bind(&TrackProcessor::publishTrackLines, this));
//     }

// private:
//     // Function to read CSV files
//     void readCSVFiles() {
//         for (int i = 101; i <= 118; ++i) {
//             std::string filename = "src/nodes/yard_describer/src/MapData/track_" + std::to_string(i) + ".csv";
//             std::ifstream file(filename);

//             if (!file.is_open()) {
//                 RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
//                 continue;
//             }

//             std::string line;
//             std::getline(file, line);  // Skip the header line

//             std::vector<geometry_msgs::msg::Point> track_points;
//             while (std::getline(file, line)) {
//                 std::stringstream ss(line);
//                 std::string token;
//                 Coordinate coord;

//                 // Read longitude and divide by 10^6 to convert to degrees
//                 std::getline(ss, token, ';');
//                 coord.longitude = std::stod(token) * 1e-6;

//                 // Read latitude and divide by 10^6 to convert to degrees
//                 std::getline(ss, token, ';');
//                 coord.latitude = std::stod(token) * 1e-6;

//                 // Read elevation directly
//                 std::getline(ss, token, ';');
//                 coord.elevation = std::stod(token);

//                 // Transform coordinates to metric (UTM)
//                 geometry_msgs::msg::Point point = transformToMetric(coord);
//                 track_points.emplace_back(point);
//             }
//             file.close();

//             tracks_.emplace_back(segmentTrack(smoothPath(track_points), 2.0));
//         }
//     }

//     // Transform geographic coordinates to UTM and return as geometry_msgs::msg::Point
//     geometry_msgs::msg::Point transformToMetric(const Coordinate& coord) {
//         double x, y;
//         int zone;
//         bool northp;

//         // Convert geographic coordinates to UTM
//         GeographicLib::UTMUPS::Forward(coord.latitude, coord.longitude, zone, northp, x, y);

//         RCLCPP_INFO(this->get_logger(), "UTM coordinates: Zone %d, %s, X: %f, Y: %f",
//                     zone, northp ? "North" : "South", x, y);

//         geometry_msgs::msg::Point point;
//         point.x = x;
//         point.y = y;
//         point.z = 0; // coord.elevation; uncomment to include elevation in the point
//         return point;
//     }

//     // Smooth the path using a simple moving average
//     std::vector<geometry_msgs::msg::Point> smoothPath(const std::vector<geometry_msgs::msg::Point>& points) {
//         std::vector<geometry_msgs::msg::Point> smoothed_points = points;
//         int window_size = 5;
//         for (size_t i = 1; i < points.size() - 1; ++i) {
//             geometry_msgs::msg::Point smoothed_point;
//             int count = 0;
//             for (int j = std::max(0, static_cast<int>(i) - window_size); j <= std::min(static_cast<int>(points.size() - 1), static_cast<int>(i) + window_size); ++j) {
//                 smoothed_point.x += points[j].x;
//                 smoothed_point.y += points[j].y;
//                 ++count;
//             }
//             smoothed_point.x /= count;
//             smoothed_point.y /= count;
//             smoothed_points[i] = smoothed_point;
//         }
//         return smoothed_points;
//     }

//     // Segment the track to have points every given distance
//     std::vector<geometry_msgs::msg::Point> segmentTrack(const std::vector<geometry_msgs::msg::Point>& points, double segment_length) {
//         std::vector<geometry_msgs::msg::Point> segmented_points;
//         if (points.empty()) return segmented_points;

//         segmented_points.push_back(points.front());
//         double accumulated_distance = 0.0;

//         for (size_t i = 1; i < points.size(); ++i) {
//             double dx = points[i].x - points[i - 1].x;
//             double dy = points[i].y - points[i - 1].y;
//             double distance = std::sqrt(dx * dx + dy * dy);
//             accumulated_distance += distance;

//             if (accumulated_distance >= segment_length) {
//                 geometry_msgs::msg::Point new_point;
//                 new_point.x = points[i - 1].x + (dx / distance) * (segment_length - (accumulated_distance - distance));
//                 new_point.y = points[i - 1].y + (dy / distance) * (segment_length - (accumulated_distance - distance));
//                 segmented_points.push_back(new_point);
//                 accumulated_distance = 0.0;
//             }
//         }

//         return segmented_points;
//     }

//     // Publish the track lines as markers
//     void publishTrackLines() {
//         if (tracks_.empty()) {
//             RCLCPP_WARN(this->get_logger(), "No track points to publish.");
//             return;
//         }

//         visualization_msgs::msg::MarkerArray marker_array;

//         for (size_t i = 0; i < tracks_.size(); ++i) {
//             visualization_msgs::msg::Marker line_strip;
//             line_strip.header.stamp = this->get_clock()->now();
//             line_strip.header.frame_id = "map";
//             line_strip.ns = "track_lines";
            
//             if (i < 2) {
//                 line_strip.id = static_cast<int>(i) + 1 + 100; //i from 0-1, tracks from 101-102
//             } else if (i >= 2 && i < 8) {
//                 line_strip.id = static_cast<int>(i) + 1 + 2 + 100; // track 103 and 104 are missing
//             } else if (i >= 8 && i <= 9) {
//                 line_strip.id = static_cast<int>(i) + 1 + 2 + 1 + 100; // track 111 is missing
//             } else if (i >= 10) {
//                 line_strip.id = static_cast<int>(i) + 1 + 2 + 1 + 1 + 100; // track 114 is missing
//             }

//             line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
//             line_strip.action = visualization_msgs::msg::Marker::ADD;
//             line_strip.scale.x = 0.2;  // Line width

//             // Color configuration, Set lines in grey
//             line_strip.color.r = 0.5;
//             line_strip.color.g = 0.5;
//             line_strip.color.b = 0.5;
//             line_strip.color.a = 0.5; // Semi-transparent

//             for (const auto& point : tracks_[i]) {
//                 line_strip.points.push_back(point);
//             }

//             marker_array.markers.push_back(line_strip);
//         }

//         marker_pub_->publish(marker_array);

//         RCLCPP_INFO(this->get_logger(), "Published %ld track lines as markers.", tracks_.size());
//     }

//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
//     rclcpp::TimerBase::SharedPtr marker_timer_;
//     std::vector<std::vector<geometry_msgs::msg::Point>> tracks_;
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<TrackProcessor>());
//     rclcpp::shutdown();
//     return 0;
// }
