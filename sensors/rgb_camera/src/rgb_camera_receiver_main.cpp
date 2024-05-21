#include <iostream>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <rgb_camera_receiver.hpp>

//hallo :) 
using namespace std::chrono_literals;

class RgbCameraReceiver : public rclcpp::Node
{
    public:
        static constexpr int UDP_PORT = 12345;
        static constexpr int MAX_BUFFER_SIZE = 65536;

        RgbCameraReceiver() : Node("rgb_camera_receiver_main")
        {
            //Init UDP receive socket
            //Create Socket
            int sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockfd_ < 0)
            {
                std::cerr << "Error: Failed to create socket" << std::endl;
                return;
            }

            //Bind socket to address and port
            sockaddr_in servaddr;
            servaddr.sin_family = AF_INET;
            servaddr.sin_addr.s_addr = htonl(INADDR_ANY); //to any available interface
            servaddr.sin_port = htons(UDP_PORT); //PORT for video stream
            if(bind(sockfd_,(const sockaddr*)&servaddr, sizeof(servaddr)) < 0)
            {
                std::cerr << "Error: Failed to bind socket" << std::endl;
                close(sockfd_);
                return;
            }

            receiveData();
            //Init ROS2 publisher
           //// timer_ = this->create_wall_timer(100ms, std::bind(&RgbCameraReceiver::publishCameraImage, this));
        }
    
    private:
        void receiveData()
        {
            //Receive Image from UnrealEngine-LAB via UDP
            char buffer[MAX_BUFFER_SIZE];
            struct sockaddr_in clientaddr;
            socklen_t len = sizeof(clientaddr);

            while (rclcpp::ok())
            {
                ssize_t bytes_received = recvfrom(sockfd_, buffer, MAX_BUFFER_SIZE, 0, (struct sockaddr*)&clientaddr, &len);
                if (bytes_received < 0)
                {
                   // std::cerr <<"Error: Failed to receive data" << std::endl;
                    continue;
                }

                  //sample code, publish test image
            cv::Mat frame(bytes_received, 1, CV_8UC1, buffer);
            cv::imshow("Received VideoStream from UE", frame);
            cv::waitKey(1); //Adjust wait time
            }
        }

        void cleanup()
        {
            close(sockfd_);
        }

      
        int sockfd_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RgbCameraReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
