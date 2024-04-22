#ifndef RGB_CAMERA_RECEIVER_HPP_
#define RGB_CAMERA_RECEIVER_HPP_

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msg/msg/image.hpp>

class RgbCameraReceiver : public rclcpp::Node
{
    public:
        RgbCameraReceiver();

    private:
        RgbCameraReceiver();

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif 