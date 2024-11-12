#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <vector>

using namespace std::chrono_literals;

struct Point {
    double x;
    double y;
    double z;
};

std::vector<Point> createClearanceProfile() {
     return //{ originales LRP
    //     {0.0, -1.274, 0.0},
    //     {0.0, -1.583, 0.380},
    //     {0.0, -1.683, 0.380},
    //     {0.0, -1.711, 0.760},
    //     {0.0, -1.862, 3.590},
    //     {0.0, -1.706, 3.895},
    //     {0.0, -1.070, 4.740},
    //     {0.0, 1.070, 4.740},
    //     {0.0, 1.706, 3.895},
    //     {0.0, 1.862, 3.590},
    //     {0.0, 1.711, 0.760},
    //     {0.0, 1.683, 0.380},
    //     {0.0, 1.583, 0.380},
    //     {0.0, 1.274, 0.0}
    // };
    {
        {0.0, -1.174, 0.0},
        {0.0, -1.483, 0.380},
        {0.0, -1.583, 0.380},
        {0.0, -1.611, 0.760},
        {0.0, -1.762, 3.590},
        {0.0, -1.606, 3.895},
        {0.0, -0.970, 4.740},
        {0.0, 0.970, 4.740},
        {0.0, 1.606, 3.895},
        {0.0, 1.762, 3.590},
        {0.0, 1.611, 0.760},
        {0.0, 1.583, 0.380},
        {0.0, 1.483, 0.380},
        {0.0, 1.174, 0.0}
    };
}

std::vector<Point> transformClearanceProfile(const Point& path_point, const std::vector<Point>& clearance_profile) {
    std::vector<Point> transformed_profile;
    for (const auto& point : clearance_profile) {
        transformed_profile.push_back({
            path_point.x + point.x,
            path_point.y + point.y,
            path_point.z + point.z
        });
    }
    return transformed_profile;
}

std::vector<Point> createTubeSegment(const std::vector<Point>& start_profile, const std::vector<Point>& end_profile) {
    std::vector<Point> tube_segment;
    tube_segment.insert(tube_segment.end(), start_profile.begin(), start_profile.end());
    tube_segment.insert(tube_segment.end(), end_profile.begin(), end_profile.end());
    return tube_segment;
}

class ClearanceProfileSetter : public rclcpp::Node {
public:
    ClearanceProfileSetter() : Node("clearance_profile_setter") {
        clearance_profile_ = createClearanceProfile();
        
        driving_path_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "highlighted_driving_path", 10, 
            std::bind(&ClearanceProfileSetter::drivingPathCallback, this, std::placeholders::_1));
        
        clearance_profile_polygons_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "clearance_profile_polygons", 10);
        
        clearance_profile_tubes_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "clearance_profile_tubes", 50);
    }

private:
    void drivingPathCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        std::vector<Point> driving_path;
        for (const auto& marker : msg->markers) {
            for (const auto& point : marker.points) {
                driving_path.push_back({point.x, point.y, point.z});
            }
        }

        if (driving_path.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty driving path.");
            return;
        }

        std::vector<std::vector<Point>> polygons = publishClearanceProfilePolygons(driving_path);
        publishClearanceProfileTubes(polygons);
    }

    std::vector<std::vector<Point>> publishClearanceProfilePolygons(const std::vector<Point>& driving_path) {
        std::vector<std::vector<Point>> polygons;
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        for (const auto& path_point : driving_path) {
            std::vector<Point> polygon = transformClearanceProfile(path_point, clearance_profile_);
            polygons.push_back(polygon);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "clearance_profile";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.05;  // Line width

            for (const auto& point : polygon) {
                geometry_msgs::msg::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                marker.points.push_back(p);
            }
            // Close the polygon by adding the first point at the end
            geometry_msgs::msg::Point p;
            p.x = polygon.front().x;
            p.y = polygon.front().y;
            p.z = polygon.front().z;
            marker.points.push_back(p);

            marker.color.r = 0.0;
            marker.color.g = 0.588; // KIT green (0, 150, 130)
            marker.color.b = 0.51;
            marker.color.a = 0.5; // 50% transparent
            marker.scale.x = 0.1; // Line width


            marker_array.markers.push_back(marker);
        }
        clearance_profile_polygons_publisher_->publish(marker_array);
        return polygons;
    }

    void publishClearanceProfileTubes(const std::vector<std::vector<Point>>& polygons) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (size_t i = 0; i < polygons.size() - 1; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "clearance_profile";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;

        for (size_t j = 0; j < polygons[i].size(); ++j) {
            size_t next_j = (j + 1) % polygons[i].size(); // Wrap around to create the tube

            geometry_msgs::msg::Point p1, p2, p3, p4;

            // First triangle
            p1.x = polygons[i][j].x;
            p1.y = polygons[i][j].y;
            p1.z = polygons[i][j].z;

            p2.x = polygons[i + 1][j].x;
            p2.y = polygons[i + 1][j].y;
            p2.z = polygons[i + 1][j].z;

            p3.x = polygons[i][next_j].x;
            p3.y = polygons[i][next_j].y;
            p3.z = polygons[i][next_j].z;

            marker.points.push_back(p1);
            marker.points.push_back(p2);
            marker.points.push_back(p3);

            // Second triangle
            p4.x = polygons[i + 1][next_j].x;
            p4.y = polygons[i + 1][next_j].y;
            p4.z = polygons[i + 1][next_j].z;

            marker.points.push_back(p3);
            marker.points.push_back(p2);
            marker.points.push_back(p4);
        }

        marker.color.r = 0.0;
        marker.color.g = 0.588; // KIT green (0, 150, 130)
        marker.color.b = 0.51;
        marker.color.a = 0.1; // 10% visibility for the tube
        marker.scale.x = 1.0; // Not used for TRIANGLE_LIST but still needed

        marker_array.markers.push_back(marker);
    }
    clearance_profile_tubes_publisher_->publish(marker_array);
}


    std::vector<Point> clearance_profile_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr driving_path_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clearance_profile_polygons_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clearance_profile_tubes_publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClearanceProfileSetter>());
    rclcpp::shutdown();
    return 0;
}
