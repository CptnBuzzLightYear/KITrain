#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>  // Include GNSS message type
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

class MapFilePublisher : public rclcpp::Node
{
public:
    MapFilePublisher() : Node("positioner")
    {
        map_publisher_ = this->create_publisher<nav_msgs::msg::Path>("corrected_position", 10);
        gnss_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
           "gnss", 10, std::bind(&MapFilePublisher::gnssCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MapFilePublisher::publish_map, this));

        RCLCPP_INFO(this->get_logger(), "MapFilePublisher node has been started.");
    }

private:
    void publish_map()
    {
        for (int i = 101; i <= 118; ++i)
        {
            nav_msgs::msg::Path path;
            path.header.stamp = this->get_clock()->now();
            path.header.frame_id = "map";

            std::string filename = "src/nodes/yard_describer/src/MapData/track_" + std::to_string(i) + ".csv";
            std::ifstream file(filename);
            if (!file.is_open())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open the CSV file: %s", filename.c_str());
                continue;
            }

            std::string line;
            bool header = true;

            while (std::getline(file, line))
            {
                if (header)
                {
                    header = false;
                    continue;
                }

                std::stringstream ss(line);
                std::string cell;
                std::vector<std::string> row;

                while (std::getline(ss, cell, ';'))
                {
                    row.push_back(cell);
                }

                if (row.size() < 3)
                {
                    continue;
                }

                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp = this->get_clock()->now();
                pose.header.frame_id = "map";

                // Replace commas with dots in the strings if the CSV uses commas as decimal separators
                std::string longitude_str = row[0];
                std::replace(longitude_str.begin(), longitude_str.end(), ',', '.');

                std::string latitude_str = row[1];
                std::replace(latitude_str.begin(), latitude_str.end(), ',', '.');

                std::string elevation_str = row[2];
                std::replace(elevation_str.begin(), elevation_str.end(), ',', '.');

                pose.pose.position.x = std::stod(longitude_str) / 1e6; // Longitude
                pose.pose.position.y = std::stod(latitude_str) / 1e6;  // Latitude
                pose.pose.position.z = std::stod(elevation_str) - 500; // Elevation - subtract 500
                path.poses.push_back(pose);
            }

            file.close();  // Close the file after reading
            map_publisher_->publish(path);
            RCLCPP_INFO(this->get_logger(), "Published path for track: %d", i);
        }
    }

    void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Assuming you have a method to convert GNSS coordinates to map position
        double longitude = msg->longitude;
        double latitude = msg->latitude;
        double altitude = msg->altitude;

        // Convert GNSS coordinates to map position (if required)
        // For simplicity, let's assume we directly use the GNSS coordinates
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = longitude;
        pose.pose.position.y = latitude;
        pose.pose.position.z = altitude;

        // Update the current position in your map representation or do further processing
        // For now, we'll just print the received GNSS data
        RCLCPP_INFO(this->get_logger(), "Received GNSS Data - Longitude: %f, Latitude: %f, Altitude: %f",
                    longitude, latitude, altitude);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr map_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapFilePublisher>());
    rclcpp::shutdown();
    return 0;
}
