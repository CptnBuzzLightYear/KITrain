#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

class MapFilePublisher : public rclcpp::Node
{
public:
    MapFilePublisher()
        : Node("map_node")
    {
        map_publisher_ = this->create_publisher<nav_msgs::msg::Path>("map", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&MapFilePublisher::publish_map, this));
        RCLCPP_INFO(this->get_logger(), "MapFilePublisher node has been started.");
    }

private:
    void publish_map()
    {
        //for (int i = 101; i <= 118; ++i)
       // {
            nav_msgs::msg::Path path;
            path.header.stamp = this->get_clock()->now();
            path.header.frame_id = "map";

            std::string filename = "src/nodes/yard_describer/src/MapData/track_" + std::to_string(101) + ".csv";
            std::ifstream file(filename);
            // if (!file.is_open())
            // {
            //     RCLCPP_ERROR(this->get_logger(), "Failed to open the CSV file: %s", filename.c_str());
            //     continue;
            // }

            RCLCPP_INFO(this->get_logger(), "Opened CSV file: %s", filename.c_str());

            std::string line;
            bool header = true;
            int line_count = 0;

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
                    RCLCPP_WARN(this->get_logger(), "Skipping incomplete row in CSV file: %s", filename.c_str());
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

                try {
                    pose.pose.position.x = std::stod(longitude_str) - 1e6; // Longitude
                    pose.pose.position.y = std::stod(latitude_str) - 1e6;  // Latitude
                    pose.pose.position.z = std::stod(elevation_str) - 500; // Elevation - subtract 500
                } catch (const std::invalid_argument& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error converting CSV row to numbers: %s", e.what());
                    continue;
                } catch (const std::out_of_range& e) {
                    RCLCPP_ERROR(this->get_logger(), "Number out of range in CSV row: %s", e.what());
                    continue;
                }

                path.poses.push_back(pose);
                line_count++;
            }

            RCLCPP_INFO(this->get_logger(), "Read %d lines from CSV file: %s", line_count, filename.c_str());

            if (!path.poses.empty())
            {
                map_publisher_->publish(path);
             //   RCLCPP_INFO(this->get_logger(), "Published path for track: %d with %lu poses", i, path.poses.size());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No valid data found in CSV file: %s", filename.c_str());
            }
        //}
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr map_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapFilePublisher>());
    rclcpp::shutdown();
    return 0;
}
