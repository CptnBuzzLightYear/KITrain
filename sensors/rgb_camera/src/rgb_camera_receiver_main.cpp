//#include <iostream>
//#include <opencv2/opencv.hpp>
#include <sys/socket.h>
//#include <netinet/in.h>
//#include <unistd.h>
#include <sensor_msgs/msg/image.hpp>
//#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>
//#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <arpa/inet.h>

struct UdpHeader{
    int8_t image_id[4];
    int8_t packet_id[4];
    int8_t total_packets[4];
};

int32_t toInt32(const int8_t* bytes){
    return static_cast<int32_t>(bytes[0])          |
            (static_cast<int32_t>(bytes[1]) << 8)  |
            (static_cast<int32_t>(bytes[2]) << 16) |
            (static_cast<int32_t>(bytes[3]) << 24);
}

//hallo :) 
using namespace std::chrono_literals;

class RgbCameraReceiver : public rclcpp::Node
{
    public:
        RgbCameraReceiver() : Node("rgb_camera_receiver_main")
        {
            //Init UDP receive socket
            //Create Socket
           
            if ((socketfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0){
                RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
                return;
            }

            //Bind socket to address and port
            sockaddr_in server_addr{};
            server_addr.sin_family = AF_INET;
            server_addr.sin_addr.s_addr = htonl(INADDR_ANY); //to any available interface
            server_addr.sin_port = htons(UDP_PORT); //PORT for video stream

            if(bind(socketfd_,(const sockaddr*)&server_addr, sizeof(server_addr)) < 0){
                RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
                             
                return;
            }

            //Create publisher
            publisher_ = this->create_publisher<sensor_msgs::msg::Image>("rgba_image", 10);

            // Create timer to periodically check fort new data
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10),
                std::bind(&RgbCameraReceiver::receiveData, this)
            );
         }

        ~RgbCameraReceiver(){
            if(socketfd_ != -1){
                close(socketfd_);
            }
        }        
    
    private:
        void receiveData(){
           //S RCLCPP_INFO(this->get_logger(),"Receiving Data via UDP");
            
            //Receive Image from UnrealEngine-LAB via UDP
            constexpr size_t MAX_UDP_SIZE = 37500;
            std::vector<uint8_t> buffer(MAX_UDP_SIZE);
            sockaddr_in client_addr{};
            socklen_t addr_len = sizeof(client_addr);
            ssize_t len = recvfrom(socketfd_, buffer.data(), buffer.size(), 0, (struct sockaddr*)&client_addr, &addr_len);
                
                if (len < 0){
                   RCLCPP_INFO(this->get_logger(),"Failed to receive Images");
                    return;
                }

               // if (len < sizeof(UdpHeader)){
               //    RCLCPP_INFO(this->get_logger(),"Received Packet is too small");
               //     return;
               // }

                //TESTING:: Print UDP Stream buffer.data()
              

                UdpHeader header;
                memcpy(&header, buffer.data(), sizeof(UdpHeader));

                int32_t image_id = toInt32(header.image_id);
                int32_t packet_id = toInt32(header.packet_id);
                int32_t total_packets = toInt32(header.total_packets);

                size_t data_offset = sizeof(UdpHeader);
                size_t data_length = len - data_offset;

                auto &image_buffer = images_[image_id];
                image_buffer.packets_received++;
                image_buffer.data.insert(image_buffer.data.end(), buffer.begin() + data_offset, buffer.begin() + data_offset + data_length);

                if (image_buffer.packets_received == total_packets){
                    //Assemble and publish imgae
                    publishImage(image_id);
                    images_.erase(image_id);
                }
        }
        void publishImage(int32_t image_id){
            RCLCPP_INFO(this->get_logger(),"ABOUT TO PUBLISH!!!!! ");
            auto &image_buffer = images_[image_id];


                // RGBA image dimensions and data
                int width = IMAGE_WIDTH;
                int height = IMAGE_HEIGHT;
                int channels = 4; // RGBA

                 //Check if received data legth matches expected image size
                if (image_buffer.data.size() != width * height * channels){
                     RCLCPP_ERROR(this->get_logger(),"Reassembeled image does not match expected image size");
                    // RCLCPP_INFO(this->get_logger()," LÄNGE IST GLEICH %zu", len);
                     return;
                }

                auto image_msg = sensor_msgs::msg::Image();
                image_msg.header.stamp = this->now();
                image_msg.header.frame_id = "camera_frame";
                image_msg.height = height;
                image_msg.width = width;
                image_msg.encoding = "rgba8";
                image_msg.is_bigendian = false;
                image_msg.step = width * channels;
                 // image_msg.data = std::vector<uint8_t>(buffer.begin(), buffer.begin() + len);
                image_msg.data = std::move(image_buffer.data);

                publisher_->publish(image_msg);
            }
       
        static constexpr int UDP_PORT = 12345;
        static constexpr int IMAGE_WIDTH = 256;
        static constexpr int IMAGE_HEIGHT = 256;

        struct ImageBuffer{
            int packets_received = 0;
            std::vector<uint8_t> data;
        };
      
        int socketfd_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::unordered_map<int32_t, ImageBuffer> images_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RgbCameraReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
