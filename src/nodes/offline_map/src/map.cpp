#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

class OfflineMap : public rclcpp::Node
{
public:
    OfflineMap()
    : Node("offline_map")
    {
        // Subscription to GPS topic (assuming sensor_msgs/NavSatFix)
        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gnss", 10, std::bind(&OfflineMap::gps_callback, this, std::placeholders::_1));

        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("offline_map_munich_GPS", 10);    

        // Load the map image from the hard drive
        map_image_ = cv::imread("/home/virtrack/KITrain/src/nodes/offline_map/src/map.png");
        if (map_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load the map image!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Map image loaded successfully.");
        }

        // Set GPS bounds of the map NW: 48.203527, 11.470329 ; SW: 48.197746, 11.488660
        lat_north_west_ = 48.203532; //414;
        lon_north_west_ = 11.470329; //866;
        lat_south_east_ = 48.197746; //10;
        lon_south_east_ = 11.488660; //09;
        

          image_width_ = map_image_.cols;
        image_height_ = map_image_.rows;

        // Timer to control the frequency of image publishing (e.g., every 2 seconds)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2), std::bind(&OfflineMap::publish_image, this));
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Store the latest GPS coordinates
        gps_lat_ = msg->latitude;
        gps_lon_ = msg->longitude;
    }

    void publish_image()
    {
        if (map_image_.empty()) {
            return;
        }

        // Convert GPS coordinates to image coordinates
        int x, y;
        std::tie(x, y) = gps_to_image_coordinates(gps_lat_, gps_lon_);

        // Create a copy of the map and plot the GPS location as a red dot
        cv::Mat map_with_gps = map_image_.clone();
        cv::circle(map_with_gps, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);  // Red dot

        // Convert OpenCV image to ROS 2 Image message using cv_bridge
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        sensor_msgs::msg::Image::SharedPtr ros_image = cv_bridge::CvImage(header, "bgr8", map_with_gps).toImageMsg();

        // Publish the image on the "offline_map_munich_GPS" topic
        image_publisher_->publish(*ros_image);

        RCLCPP_INFO(this->get_logger(), "Published image with GPS point on offline_map_munich_GPS");
    }

    std::tuple<int, int> gps_to_image_coordinates(double gps_lat, double gps_lon)
    {
        // Map latitude to pixel y-coordinate (invert because pixel y increases downward)
        int y = static_cast<int>(image_height_ * (lat_north_west_ - gps_lat) / (lat_north_west_ - lat_south_east_));

        // Map longitude to pixel x-coordinate
        int x = static_cast<int>(image_width_ * (gps_lon - lon_north_west_) / (lon_south_east_ - lon_north_west_));

        return std::make_tuple(x, y);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    cv::Mat map_image_;

    double lat_north_west_;
    double lon_north_west_;
    double lat_south_east_;
    double lon_south_east_;

    int image_width_;
    int image_height_;

    double gps_lat_;  // Latest latitude
    double gps_lon_;  // Latest longitude

    rclcpp::TimerBase::SharedPtr timer_;  // Timer to publish images at intervals
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OfflineMap>());
    rclcpp::shutdown();
    return 0;
}

// //         image_width_ = map_image_.cols;
// //         image_height_ = map_image_.rows;
// //     }

// // private:
// // void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
// // {
// //     if (map_image_.empty()) {
// //         return;
// //     }

// //     // Extract GPS coordinates from the message
// //     double gps_lat = msg->latitude;
// //     double gps_lon = msg->longitude;

// //     // Convert GPS coordinates to image coordinates
// //     int x, y;
// //     std::tie(x, y) = gps_to_image_coordinates(gps_lat, gps_lon);

// //     // Create a copy of the map and plot the GPS location as a red dot
// //     cv::Mat map_with_gps = map_image_.clone();
// //     cv::circle(map_with_gps, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);  // Red dot

// //     // Convert OpenCV image to ROS 2 Image message using cv_bridge
// //     std_msgs::msg::Header header;
// //     header.stamp = this->get_clock()->now();
// //     sensor_msgs::msg::Image::SharedPtr ros_image = cv_bridge::CvImage(header, "bgr8", map_with_gps).toImageMsg();

// //     // Publish the image on the "offline_map_munich_GPS" topic
// //     image_publisher_->publish(*ros_image);

// //    // RCLCPP_INFO(this->get_logger(), "Published image with GPS point on offline_map_munich_GPS");
// // }


       //__________________SHOW IMAGE WITH GPS
    //void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    
    
 
    // {
    //     if (map_image_.empty()) {
    //         return;
    //     }

    //     // Extract GPS coordinates from the message
    //     double gps_lat = msg->latitude;
    //     double gps_lon = msg->longitude;

    //     // Convert GPS coordinates to image coordinates
    //     int x, y;
    //     std::tie(x, y) = gps_to_image_coordinates(gps_lat, gps_lon);

    //     // Create a copy of the map and plot the GPS location as a red dot
    //     cv::Mat map_with_gps = map_image_.clone();
    //     cv::circle(map_with_gps, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);  // Red dot

    //     // Display the image
    //     cv::imshow("Map with GPS Point", map_with_gps);
    //     cv::waitKey(1);
    // }

// //     std::tuple<int, int> gps_to_image_coordinates(double gps_lat, double gps_lon)
// //     {
// //         // Map latitude to pixel y-coordinate (invert because pixel y increases downward)
// //         int y = static_cast<int>(image_height_ * (lat_north_west_ - gps_lat) / (lat_north_west_ - lat_south_east_));

// //         // Map longitude to pixel x-coordinate
// //         int x = static_cast<int>(image_width_ * (gps_lon - lon_north_west_) / (lon_south_east_ - lon_north_west_));

// //         return std::make_tuple(x, y);
// //     }

// //     rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
// //     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

// //     cv::Mat map_image_;

// //     double lat_north_west_;
// //     double lon_north_west_;
// //     double lat_south_east_;
// //     double lon_south_east_;

// //     int image_width_;
// //     int image_height_;
// // };

// // int main(int argc, char *argv[])
// // {
// //     rclcpp::init(argc, argv);
// //     rclcpp::spin(std::make_shared<OfflineMap>());
// //     rclcpp::shutdown();
// //     return 0;
// // }
