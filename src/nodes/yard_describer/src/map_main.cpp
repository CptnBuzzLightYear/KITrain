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
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("driving_path", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MapFilePublisher::publish_paths, this));
        RCLCPP_INFO(this->get_logger(), "MapFilePublisher node has been started.");
    }

private:
    void publish_paths()
    {
        nav_msgs::msg::Path path;
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "map";

        for (int i = 101; i <= 118; ++i)
        {
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

                while (std::getline(ss, cell, ';')) // Changed delimiter to semicolon
                {
                    row.push_back(cell);
                }

                if (row.size() < 6)
                {
                    continue;
                }

                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp = this->get_clock()->now();
                pose.header.frame_id = "map";

                // Replace commas with dots in the strings if the CSV uses commas as decimal separators
                std::string x_str = row[4];
                std::replace(x_str.begin(), x_str.end(), ',', '.');

                std::string y_str = row[5];
                std::replace(y_str.begin(), y_str.end(), ',', '.');

                std::string z_str = row[3];
                std::replace(z_str.begin(), z_str.end(), ',', '.');

                pose.pose.position.x = std::stod(x_str); // E column (lateral coordinate)
                pose.pose.position.y = std::stod(y_str); // F column (longitudinal coordinate)
                pose.pose.position.z = std::stod(z_str)-500; // D column (altitude)
                path.poses.push_back(pose);
            }
        }

        publisher_->publish(path);
        RCLCPP_INFO(this->get_logger(), "Publishing aggregated path of all tracks");
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapFilePublisher>());
    rclcpp::shutdown();
    return 0;
}
