#include <rclcpp/rclcpp.hpp>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
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
        char buffer[2024];
        sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);

        while (rclcpp::ok())
        {
            ssize_t bytes_received = recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&sender_addr, &sender_len);

            if (bytes_received > 0)
            {
                buffer[bytes_received] = '\0';
                 // Log the received UDP data
            RCLCPP_INFO(this->get_logger(), "Received UDP data: %s", buffer);

                
                std::string jsonData(buffer, bytes_received);

                

                // Parse JSON data
                try
                {
                    auto jsonDataParsed = json::parse(jsonData);

                   // std::string timestamp = jsonDataParsed["TimeStamp"];
                    std::string currPos = jsonDataParsed["CurrentPosition"];
                    std::string targetPos = jsonDataParsed["TargetPosition"];
                    int taskID = jsonDataParsed["TaskID"];

                    RCLCPP_INFO(this->get_logger(), "Current Position: %s, Target Position: %s, TaskID: %d",
                                currPos.c_str(), targetPos.c_str(), taskID);
                    
                    // RCLCPP_INFO(this->get_logger(), "TimeStamp: %s, Current Position: %s, Target Position: %s, TaskID: %d",
                    //             timestamp.c_str(), currPos.c_str(), targetPos.c_str(), taskID);
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
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UDPOrderReceiverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


// #include <rclcpp/rclcpp.hpp>
// #include <netinet/in.h>
// #include <sys/socket.h>
// #include <unistd.h>
// #include <cstring>
// #include <jsoncpp/json/json.h>  // Assuming you use jsoncpp for JSON parsing

// using namespace std::chrono_literals;

// class UDPOrderReceiverNode : public rclcpp::Node {
// public:
//     UDPOrderReceiverNode() : Node("order_definer_node") {
//         // Create UDP socket
//         sockfd = socket(AF_INET, SOCK_DGRAM, 0);
//         if (sockfd < 0) {
//             RCLCPP_ERROR(get_logger(), "Failed to create socket");
//             return;
//         }

//         // Bind socket to port
//         struct sockaddr_in addr;
//         std::memset(&addr, 0, sizeof(addr));
//         addr.sin_family = AF_INET;
//         addr.sin_addr.s_addr = htonl(INADDR_ANY);  // Accept any incoming interface
//         addr.sin_port = htons(50128);  // Replace with your port number

//         if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
//             RCLCPP_ERROR(get_logger(), "Failed to bind socket");
//             close(sockfd);
//             return;
//         }

//         // Receive loop
//         while (rclcpp::ok()) {
//             struct sockaddr_in cliaddr;
//             socklen_t len = sizeof(cliaddr);
//             char buffer[1024];  // Adjust buffer size as needed

//             ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&cliaddr, &len);
//             if (n < 0) {
//                 RCLCPP_ERROR(get_logger(), "Error in recvfrom");
//                 continue;
//             }

//             // Convert received data to string
//             std::string jsonData(buffer, n);

//             // Parse JSON
//             Json::Value root;
//             Json::Reader reader;
//             bool parsingSuccessful = reader.parse(jsonData, root);
//             if (parsingSuccessful) {
//                 // Extract fields
//                 std::string currentPosition = root["CurrentPosition"].asString();
//                 std::string targetPosition = root["TargetPosition"].asString();
//                 int taskID = root["TaskID"].asInt();

//                 // Process received data
//                 processReceivedData(currentPosition, targetPosition, taskID);
//             } else {
//                 RCLCPP_ERROR(get_logger(), "Failed to parse JSON data");
//             }

//             // Sleep or do other work between receives
//             std::this_thread::sleep_for(1s);
//         }
//     }

// private:
//     int sockfd;

//     void processReceivedData(const std::string& currPos, const std::string& targetPos, int taskID) {
//         // Implement processing logic here
//         RCLCPP_INFO(get_logger(), "Received: Current Position = %s, Target Position = %s, Task ID = %d",
//                     currPos.c_str(), targetPos.c_str(), taskID);
        
//         // Optionally, publish processed data as ROS 2 messages
//         // Example: publishData(currPos, targetPos, taskID);
//     }
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<UDPOrderReceiverNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
