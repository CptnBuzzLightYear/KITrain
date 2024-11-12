#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <cstring> // for std::memcpy

class UDPVelocityReceiverNode : public rclcpp::Node
{
public:
    UDPVelocityReceiverNode() : Node("udp_velocity_receiver_node")
    {
        // Set up the UDP socket
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            rclcpp::shutdown();
        }

        sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(50130);  // Updated to use port 50130

        if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "UDP Velocity Receiver Node has started on port 50130");

        // Create publisher for velocity
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("vehicle_velocity", 10);

        receive_thread_ = std::thread(&UDPVelocityReceiverNode::receive_loop, this);
    }

    ~UDPVelocityReceiverNode()
    {
        close(socket_fd_);
        if (receive_thread_.joinable())
        {
            receive_thread_.join();
        }
    }

private:
    void receive_loop()
    {
        char buffer[4];  // Expecting exactly 4 bytes for a single float
        sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);

        while (rclcpp::ok())
        {
            ssize_t bytes_received = recvfrom(socket_fd_, buffer, sizeof(buffer), 0, (struct sockaddr *)&sender_addr, &sender_len);

            if (bytes_received == 4)  // Ensure that we have received exactly 4 bytes
            {
                // Convert the 4 bytes into a float
                float received_value;
                std::memcpy(&received_value, buffer, sizeof(received_value)); // Copy the bytes to a float

                //RCLCPP_INFO(this->get_logger(), "Received float value: %f", received_value);

                // Apply the inverse scaling to recover the original velocity
                float velocity_mps = received_value * (40.0f / 255.0f) / 3.6;

               // RCLCPP_INFO(this->get_logger(), "Calculated Velocity (m/s): %.6f", velocity_mps);

                // Create and publish the velocity message
                geometry_msgs::msg::Twist velocity_msg;
                velocity_msg.linear.x = velocity_mps;
                velocity_msg.angular.z = 0.0;
                velocity_pub_->publish(velocity_msg);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Received incorrect number of bytes. Expected 4 bytes, received: %zd", bytes_received);
            }
        }
    }

    int socket_fd_;
    std::thread receive_thread_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UDPVelocityReceiverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <netinet/in.h>
// #include <sys/socket.h>
// #include <arpa/inet.h>
// #include <unistd.h>
// #include <iomanip>
// #include <sstream>
// #include <iostream>

// class UDPVelocityReceiverNode : public rclcpp::Node
// {
// public:
//     UDPVelocityReceiverNode() : Node("udp_velocity_receiver_node")
//     {
//         // Set up the UDP socket
//         socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
//         if (socket_fd_ < 0)
//         {
//             RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
//             rclcpp::shutdown();
//         }

//         sockaddr_in addr;
//         addr.sin_family = AF_INET;
//         addr.sin_addr.s_addr = INADDR_ANY;
//         addr.sin_port = htons(50130);

//         if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
//         {
//             RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
//             rclcpp::shutdown();
//         }

//         RCLCPP_INFO(this->get_logger(), "UDP Velocity Receiver Node has started");

//         // Create publisher for velocity
//         velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("vehicle_velocity", 10);

//         receive_thread_ = std::thread(&UDPVelocityReceiverNode::receive_loop, this);
//     }

//     ~UDPVelocityReceiverNode()
//     {
//         close(socket_fd_);
//         if (receive_thread_.joinable())
//         {
//             receive_thread_.join();
//         }
//     }

// private:
//     void receive_loop()
//     {
//         char buffer[1024];  // Buffer size large enough to handle multiple values
//         sockaddr_in sender_addr;
//         socklen_t sender_len = sizeof(sender_addr);

//         while (rclcpp::ok())
//         {
//             ssize_t bytes_received = recvfrom(socket_fd_, buffer, sizeof(buffer), 0, (struct sockaddr *)&sender_addr, &sender_len);

//             if (bytes_received > 0)
//             {
//                 // Log the raw byte stream
//                 std::stringstream ss;
//                 for (ssize_t i = 0; i < bytes_received; ++i)
//                 {
//                     ss << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)buffer[i] << " ";
//                 }
//                 RCLCPP_INFO(this->get_logger(), "Received UDP data stream: %s", ss.str().c_str());

//                 // Extract the third byte as the velocity value
//                 uint8_t velocity_value = static_cast<uint8_t>(buffer[2]);

//                 RCLCPP_INFO(this->get_logger(), "Interpreted velocity byte: %u", velocity_value);

//                 // Convert the value to velocity in m/s
//                 double velocity_mps = velocity_value * (40.0 / 255.0) / 3.6;  // Convert to m/s

//                 RCLCPP_INFO(this->get_logger(), "Calculated Velocity: %.2f m/s", velocity_mps);

//                 // Create and publish the velocity message
//                 geometry_msgs::msg::Twist velocity_msg;
//                 velocity_msg.linear.x = velocity_mps;
//                 velocity_msg.angular.z = 0.0;
//                 velocity_pub_->publish(velocity_msg);
//             }
//             else
//             {
//                 RCLCPP_WARN(this->get_logger(), "No data received or error in receiving UDP data.");
//             }
//         }
//     }

//     int socket_fd_;
//     std::thread receive_thread_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<UDPVelocityReceiverNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <chrono>

// class OdometryNode : public rclcpp::Node
// {
// public:
//     OdometryNode()
//     : Node("odometry_node")
//     {
//         // Initialize publisher for the velocity topic
//         velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("vehicle_velocity", 10);

//         // Initialize the placeholder velocity (5 km/h = 1.39 m/s)
//         placeholder_velocity_.linear.x = 1.39;
//         placeholder_velocity_.angular.z = 0.0;

//         // Timer to publish placeholder velocity periodically
//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(1), // Publish every second
//             std::bind(&OdometryNode::publish_velocity, this));
//     }

// private:
//     void publish_velocity()
//     {
//         // Publish the placeholder velocity
//         RCLCPP_INFO(this->get_logger(), "Publishing placeholder velocity: linear.x=%.2f m/s, angular.z=%.2f rad/s",
//                     placeholder_velocity_.linear.x, placeholder_velocity_.angular.z);
//         velocity_pub_->publish(placeholder_velocity_);
//     }

//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
//     geometry_msgs::msg::Twist placeholder_velocity_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<OdometryNode>());
//     rclcpp::shutdown();
//     return 0;
// }
