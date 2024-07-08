#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <sys/socket.h>
#include <netinet/in.h>


using std::placeholders::_1;

class GNSSUDPPublisher : public rclcpp::Node
{
    public:
    GNSSUDPPublisher() : Node("gnss_udp_publisher")
    {
        gnssPublisher_ = this ->create_publisher<sensor_msgs::msg::NavSatFix>("gnss", 10);
        
        // Create UDP socket for receiving GNSS data
        sockfd2_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd2_ <0){
            perror("Error: creating GNSS UDP socket");
            return;
        }
        //Bind socket to address and port
            sockaddr_in servaddr2;
            servaddr2.sin_family = AF_INET;
            servaddr2.sin_addr.s_addr = htonl(INADDR_ANY); //to any available interface
            servaddr2.sin_port = htons(25002);             //PORT for GNSS stream
            if(bind(sockfd2_,(const sockaddr*)&servaddr2, sizeof(servaddr2)) < 0){
                std::cerr << "Error: Failed to bind socket" << std::endl;
                close(sockfd2_);
                return;
            }
            //Start receiving UDP packets in a separate thread
            receive_gnss_thread_ = std::thread(&GNSSUDPPublisher::receiveLoop, this);
    }  
    ~GNSSUDPPublisher()
    {
        close(sockfd2_);
        if (receive_gnss_thread_.joinable())
            receive_gnss_thread_.join();
    }

 private: 

 void receiveLoop(){      
        const uint16_t MAX_UDP_PACKET_SIZE = 18928;
        char buffer[MAX_UDP_PACKET_SIZE];

         RCLCPP_INFO(this->get_logger(), "Receive GNSS UDP Loop started");
        
        while (rclcpp::ok()){
            //Receive UDP Data          
            sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            size_t bytes_received = recvfrom(sockfd2_, buffer, MAX_UDP_PACKET_SIZE,0 , (struct sockaddr *)&client_addr, &addr_len);

RCLCPP_INFO(this->get_logger(), "Receive GNSS UDP WHILE Loop started");

            if(bytes_received <= 0){
                 RCLCPP_INFO(this->get_logger(), "Error receiving GNSS UDP data");
                continue;
            }
             //RCLCPP_INFO(this->get_logger(), "Received GNSS UDP");
             //buffer[bytes_received] = '\0';
             //log_buffer_hex(buffer, bytes_received);
             
             //CLCPP_INFO(this->get_logger(), "Received Data %s", buffer);

            if(!convertToNavSatFix(buffer, bytes_received)){
                RCLCPP_ERROR(this->get_logger(), "Error converting UDP data to NavSatFix message");
                continue;
            }
        }
    }

bool convertToNavSatFix(const char *data, size_t size){       
        if (size < 12)
        {
            RCLCPP_ERROR(this->get_logger(), "Insufficient data received");
            return false;
        }

        float longitude, latitude, altitude;
        std::memcpy(&longitude, &data[0], sizeof(float));
        std::memcpy(&latitude, &data[4], sizeof(float));
        std::memcpy(&altitude, &data[8], sizeof(float));

        RCLCPP_INFO(this->get_logger(), "Received GNSS Data - Longitude: %f, Latitude: %f, Altitude: %f",
                    longitude, latitude, altitude);

        sensor_msgs::msg::NavSatFix msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "gps_frame";
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        msg.latitude = latitude;
        msg.longitude = longitude;
        msg.altitude = altitude;
        // Assume covariance is unknown, so use zeros
        std::fill(std::begin(msg.position_covariance), std::end(msg.position_covariance), 0.0);
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        gnssPublisher_->publish(msg);

        return true;
    }

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnssPublisher_;
    int sockfd2_;
    std::thread receive_gnss_thread_;
};


int main(int argc, char **argv){  
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GNSSUDPPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}