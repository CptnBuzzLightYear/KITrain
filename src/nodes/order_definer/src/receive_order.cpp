#include <rclcpp/rclcpp.hpp>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>
#include <cstring>
#include <iostream>

using json = nlohmann::json;

class UDPOrderReceiverNode : public rclcpp::Node
{
public:
    UDPOrderReceiverNode() : Node("order_definer_node")
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
        addr.sin_port = htons(50128);

        if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "UDP Receiver Node has started");

        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("order_info", 10);

        receive_thread_ = std::thread(&UDPOrderReceiverNode::receive_loop, this);
    }

    ~UDPOrderReceiverNode()
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
        char buffer[1024];
        sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);

        while (rclcpp::ok())
        {
            ssize_t bytes_received = recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&sender_addr, &sender_len);

            if (bytes_received > 0)
            {
                buffer[bytes_received] = '\0';
                std::string jsonData(buffer, bytes_received);

                // Print raw UDP data to log
                RCLCPP_INFO(this->get_logger(), "Received UDP data: %s", jsonData.c_str());

                // Parse JSON data
                try
                {
                    auto jsonDataParsed = json::parse(jsonData);

                    int currPos = jsonDataParsed["CurrentPosition"];
                    int targetPos = jsonDataParsed["TargetPosition"];
                    int taskID = jsonDataParsed["TaskID"];

                    RCLCPP_INFO(this->get_logger(), "Current Position: %d, Target Position: %d, TaskID: %d",
                                currPos, targetPos, taskID);

                    // Create and publish message
                    auto message = std_msgs::msg::String();
                    message.data = "Current Position: " + std::to_string(currPos) + ", Target Position: " + std::to_string(targetPos) + ", TaskID: " + std::to_string(taskID);
                    publisher_->publish(message);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON data: %s", e.what());
                }
            }
        }
    }

    int socket_fd_;
    std::thread receive_thread_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UDPOrderReceiverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
