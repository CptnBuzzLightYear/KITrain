#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <thread>
#include <exception>

class UDPImageReceiver : public rclcpp::Node {
public:
    UDPImageReceiver()
        : Node("udp_image_receiver"), socket_fd(-1), buffer_size(8950), retry_count(3) {  // Match Unreal's PACK_SIZE
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("received_image", 10);

        socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = htons(9091);
        addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
            close(socket_fd);
            return;
        }

        receive_thread_ = std::thread(&UDPImageReceiver::receiveLoop, this);
    }

    ~UDPImageReceiver() {
        if (socket_fd >= 0) {
            close(socket_fd);
        }
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }

private:
    void receiveLoop() {
        while (rclcpp::ok()) {
            try {
                receiveAndDecodeImage();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in receiveLoop: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Use explicit chrono syntax
            }
        }
    }

    void receiveAndDecodeImage() {
        char size_buffer[sizeof(int)];
        struct sockaddr_in sender_addr;
        socklen_t sender_addr_len = sizeof(sender_addr);

        int received_bytes = recvfrom(socket_fd, size_buffer, sizeof(int), 0, 
                                      (struct sockaddr*)&sender_addr, &sender_addr_len);

        if (received_bytes != sizeof(int)) {
            RCLCPP_WARN(this->get_logger(), "Failed to receive image size");
            return;
        }

        int total_packets = 0;
        memcpy(&total_packets, size_buffer, sizeof(int));

        if (total_packets <= 0 || total_packets > 100000) {
            RCLCPP_WARN(this->get_logger(), "Invalid total_packets value: %d", total_packets);
            return;
        }

        std::vector<uchar> image_data;
        image_data.reserve(total_packets * buffer_size);

        uint8_t expected_sequence = 0;
        for (int i = 0; i < total_packets; ++i) {
            std::vector<uchar> packet_data(buffer_size + 1); // +1 for the sequence number
            int retry = retry_count;

            do {
                received_bytes = recvfrom(socket_fd, packet_data.data(), buffer_size + 1, 0, 
                                          (struct sockaddr*)&sender_addr, &sender_addr_len);
                retry--;
            } while (retry > 0 && received_bytes <= 0);

            if (received_bytes > 0 && packet_data[0] == expected_sequence++) {
                int CurrentPackSize = (i == total_packets - 1) ? (received_bytes - 1) : buffer_size;
                image_data.insert(image_data.end(), packet_data.begin() + 1, packet_data.begin() + 1 + CurrentPackSize);
            } else {
                RCLCPP_WARN(this->get_logger(), "Packet loss detected for packet %d", i);
                return;
            }
        }

        if (!image_data.empty()) {
            cv::Mat img = cv::imdecode(image_data, cv::IMREAD_UNCHANGED);
            if (!img.empty()) {
                publishImage(img);
                  image_data.clear();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to decode image");
                image_data.clear();
            }
        }
    }

    void publishImage(cv::Mat& img) {
        cv::Mat brightened_img;
        double alpha = 1.8;
        img.convertTo(brightened_img, -1, alpha, 0);

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";

        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", brightened_img).toImageMsg();
        image_publisher_->publish(*msg);
    }

    int socket_fd;
    const int buffer_size;
    int retry_count;
    std::thread receive_thread_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UDPImageReceiver>());
    rclcpp::shutdown();
    return 0;
}
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>
// #include <vector>
// #include <thread>
// #include <exception>

// class UDPImageReceiver : public rclcpp::Node {
// public:
//     UDPImageReceiver()
//         : Node("udp_image_receiver"), socket_fd(-1), buffer_size(4 * 4096), retry_count(3) {
//         image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("received_image", 100);

//         socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
//         if (socket_fd < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
//             return;
//         }

//         struct sockaddr_in addr;
//         addr.sin_family = AF_INET;
//         addr.sin_port = htons(9091);
//         addr.sin_addr.s_addr = htonl(INADDR_ANY);

//         if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
//             close(socket_fd);
//             return;
//         }

//         receive_thread_ = std::thread(&UDPImageReceiver::receiveLoop, this);
//     }

//     ~UDPImageReceiver() {
//         if (socket_fd >= 0) {
//             close(socket_fd);
//         }
//         if (receive_thread_.joinable()) {
//             receive_thread_.join();
//         }
//     }

// private:
//     void receiveLoop() {
//     while (rclcpp::ok()) {
//         try {
//             receiveAndDecodeImage();
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "Exception in receiveLoop: %s", e.what());
//             std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Use explicit chrono syntax
//         }
//     }
// }


//     void receiveAndDecodeImage() {
//         char size_buffer[sizeof(int)];
//         struct sockaddr_in sender_addr;
//         socklen_t sender_addr_len = sizeof(sender_addr);

//         int received_bytes = recvfrom(socket_fd, size_buffer, sizeof(int), 0, 
//                                       (struct sockaddr*)&sender_addr, &sender_addr_len);

//         if (received_bytes != sizeof(int)) {
//             RCLCPP_WARN(this->get_logger(), "Failed to receive image size");
//             return;
//         }

//         int total_packets = 0;
//         memcpy(&total_packets, size_buffer, sizeof(int));

//         if (total_packets <= 0 || total_packets > 100000) {
//             RCLCPP_WARN(this->get_logger(), "Invalid total_packets value: %d", total_packets);
//             return;
//         }

//         std::vector<uchar> image_data;
//         image_data.reserve(total_packets * buffer_size);

//         uint8_t expected_sequence = 0;
//         for (int i = 0; i < total_packets; ++i) {
//             std::vector<uchar> packet_data(buffer_size + 1); // +1 for the sequence number
//             int retry = retry_count;

//             do {
//                 received_bytes = recvfrom(socket_fd, packet_data.data(), buffer_size + 1, 0, 
//                                           (struct sockaddr*)&sender_addr, &sender_addr_len);
//                 retry--;
//             } while (retry > 0 && received_bytes <= 0);

//             if (received_bytes > 0 && packet_data[0] == expected_sequence++) {
//                 image_data.insert(image_data.end(), packet_data.begin() + 1, packet_data.begin() + received_bytes);
//             } else {
//                 RCLCPP_WARN(this->get_logger(), "Packet loss detected for packet %d", i);
//                 return;
//             }
//         }

//         if (!image_data.empty()) {
//             cv::Mat img = cv::imdecode(image_data, cv::IMREAD_UNCHANGED);
//             if (!img.empty()) {
//                 publishImage(img);
//             } else {
//                 RCLCPP_ERROR(this->get_logger(), "Failed to decode image");
//                 image_data.clear();
//             }
//         }
//     }

//     void publishImage(cv::Mat& img) {
//         cv::Mat brightened_img;
//         double alpha = 1.8;
//         img.convertTo(brightened_img, -1, alpha, 0);

//         std_msgs::msg::Header header;
//         header.stamp = this->now();
//         header.frame_id = "camera_frame";

//         sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", brightened_img).toImageMsg();
//         image_publisher_->publish(*msg);
//     }

//     int socket_fd;
//     const int buffer_size;
//     int retry_count;
//     std::thread receive_thread_;
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<UDPImageReceiver>());
//     rclcpp::shutdown();
//     return 0;
// }


// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>
// #include <vector>
// #include <thread>
// #include <exception>

// using namespace std::chrono_literals;

// class UDPImageReceiver : public rclcpp::Node {
// public:
//     UDPImageReceiver()
//         : Node("udp_image_receiver"), socket_fd(-1), buffer_size(4096) {
//         image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("received_image", 100);

//         socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
//         if (socket_fd < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
//             return;
//         }

//         struct sockaddr_in addr;
//         addr.sin_family = AF_INET;
//         addr.sin_port = htons(9091);
//         addr.sin_addr.s_addr = htonl(INADDR_ANY);

//         if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
//             close(socket_fd);
//             return;
//         }

//         RCLCPP_INFO(this->get_logger(), "UDPImageReceiver node has been started");

//         receive_thread_ = std::thread(&UDPImageReceiver::receiveLoop, this);
//     }

//     ~UDPImageReceiver() {
//         if (socket_fd >= 0) {
//             close(socket_fd);
//         }
//         if (receive_thread_.joinable()) {
//             receive_thread_.join();
//         }
//     }

// private:
//     void receiveLoop() {
//         while (rclcpp::ok()) {
//             try {
//                 receiveAndDecodeImage();
//             } catch (const std::exception& e) {
//                 RCLCPP_ERROR(this->get_logger(), "Exception in receiveLoop: %s", e.what());
//                 std::this_thread::sleep_for(100ms); // Sleep for a short while before retrying
//             }
//         }
//     }

//     void receiveAndDecodeImage() {
//         char size_buffer[sizeof(int)];
//         struct sockaddr_in sender_addr;
//         socklen_t sender_addr_len = sizeof(sender_addr);

//         int received_bytes = recvfrom(socket_fd, size_buffer, sizeof(int), 0, 
//                                       (struct sockaddr*)&sender_addr, &sender_addr_len);

//         if (received_bytes != sizeof(int)) {
//             RCLCPP_WARN(this->get_logger(), "Failed to receive image size");
//             return;
//         }

//         int total_packets = 0;
//         memcpy(&total_packets, size_buffer, sizeof(int));

//         if (total_packets <= 0 || total_packets > 100000) { // Sanity check on total_packets
//             RCLCPP_WARN(this->get_logger(), "Invalid total_packets value: %d", total_packets);
//             return;
//         }

//         std::vector<uchar> image_data;
//         try {
//             image_data.reserve(total_packets * buffer_size);
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to reserve memory for image_data: %s", e.what());
//             return;
//         }

//         for (int i = 0; i < total_packets; ++i) {
//             std::vector<uchar> packet_data(buffer_size);
//             received_bytes = recvfrom(socket_fd, packet_data.data(), buffer_size, 0, 
//                                       (struct sockaddr*)&sender_addr, &sender_addr_len);
//             if (received_bytes > 0) {
//                 image_data.insert(image_data.end(), packet_data.begin(), packet_data.begin() + received_bytes);
//             } else {
//                 RCLCPP_WARN(this->get_logger(), "Failed to receive packet %d", i);
//                 return;
//             }
//         }

//         if (!image_data.empty()) {
//             try {
//                 cv::Mat img = cv::imdecode(image_data, cv::IMREAD_UNCHANGED);
//                 if (!img.empty()) {
//                     publishImage(img);
//                 } else {
//                     RCLCPP_ERROR(this->get_logger(), "Failed to decode image");
//                     image_data.clear(); // Clear the buffer on error
//                 }
//             } catch (const cv::Exception& e) {
//                 RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
//                 image_data.clear(); // Clear the buffer on error
//             }
//         }
//     }

//     void publishImage(cv::Mat& img) {
//     // Adjust the brightness
//     cv::Mat brightened_img;
//     double alpha = 1.8;  // Brightness factor (1.0 means no change, >1.0 increases brightness)
//     img.convertTo(brightened_img, -1, alpha, 0);

//     std_msgs::msg::Header header;
//     header.stamp = this->now();
//     header.frame_id = "camera_frame";

//     sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", brightened_img).toImageMsg();
//     image_publisher_->publish(*msg);
//    //RCLCPP_INFO(this->get_logger(), "Brightened Image Published");
// }


//     int socket_fd;
//     const int buffer_size;
//     std::thread receive_thread_;
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<UDPImageReceiver>());
//     rclcpp::shutdown();
//     return 0;
// }

// ______________________________________________________________________________________________________-
//running with rgb images from openCV modified camerasender
//#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>

// using namespace std::chrono_literals;

// class UDPImageReceiver : public rclcpp::Node {
// public:
//     UDPImageReceiver()
//         : Node("udp_image_receiver"), socket_fd(-1), buffer_size(4096) {
//         image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("received_image", 100);

//         socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
//         if (socket_fd < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
//             return;
//         }

//         struct sockaddr_in addr;
//         addr.sin_family = AF_INET;
//         addr.sin_port = htons(9091);
//         addr.sin_addr.s_addr = htonl(INADDR_ANY);

//         if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
//             close(socket_fd);
//             return;
//         }

//         RCLCPP_INFO(this->get_logger(), "UDPImageReceiver node has been started");

//         receive_thread_ = std::thread(&UDPImageReceiver::receiveLoop, this);
//     }

//     ~UDPImageReceiver() {
//         if (socket_fd >= 0) {
//             close(socket_fd);
//         }
//         if (receive_thread_.joinable()) {
//             receive_thread_.join();
//         }
//     }

// private:
//     void receiveLoop() {
//         while (rclcpp::ok()) {
//             try {
//                 receiveAndDecodeImage();
//             } catch (const std::exception& e) {
//                 RCLCPP_ERROR(this->get_logger(), "Exception in receiveLoop: %s", e.what());
//                 // Continue loop after logging error
//             }
//         }
//     }

//     void receiveAndDecodeImage() {
//         char size_buffer[sizeof(int)];
//         struct sockaddr_in sender_addr;
//         socklen_t sender_addr_len = sizeof(sender_addr);

//         int received_bytes = recvfrom(socket_fd, size_buffer, sizeof(int), 0, 
//                                       (struct sockaddr*)&sender_addr, &sender_addr_len);

//         if (received_bytes != sizeof(int)) {
//             RCLCPP_WARN(this->get_logger(), "Failed to receive image size");
//             return;
//         }

//         int total_packets = 0;
//         memcpy(&total_packets, size_buffer, sizeof(int));

//         std::vector<uchar> image_data;
//         try {
//             image_data.reserve(total_packets * buffer_size*2);
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to reserve memory for image_data: %s", e.what());
//             return;
//         }

//         for (int i = 0; i < total_packets; ++i) {
//             std::vector<uchar> packet_data(buffer_size);
//             received_bytes = recvfrom(socket_fd, packet_data.data(), buffer_size, 0, 
//                                       (struct sockaddr*)&sender_addr, &sender_addr_len);
//             if (received_bytes > 0) {
//                 image_data.insert(image_data.end(), packet_data.begin(), packet_data.begin() + received_bytes);
//             } else {
//                 RCLCPP_WARN(this->get_logger(), "Failed to receive packet %d", i);
//                 return;
//             }
//         }

//         if (!image_data.empty()) {
//             try {
//                 cv::Mat img = cv::imdecode(image_data, cv::IMREAD_UNCHANGED);
//                 if (!img.empty()) {
//                     publishImage(img);
//                 } else {
//                     RCLCPP_ERROR(this->get_logger(), "Failed to decode image");
//                     // Reset buffers and state
//                     image_data.clear();
//                 }
//             } catch (const cv::Exception& e) {
//                 RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
//                 // Reset buffers and state
//                 image_data.clear();
//             }
//         }
//     }

//     void publishImage(const cv::Mat& img) {
//         std_msgs::msg::Header header;
//         header.stamp = this->now();
//         header.frame_id = "camera_frame";

//         sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
//         image_publisher_->publish(*msg);
//         RCLCPP_INFO(this->get_logger(), "Image Published");
//     }

//     int socket_fd;
//     const int buffer_size;
//     std::thread receive_thread_;
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<UDPImageReceiver>());
//     rclcpp::shutdown();
//     return 0;
// }


// __________________________________________________________________________________________________________________________________
//runs wih Open CV
//#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <arpa/inet.h>
// #include <unistd.h>
// #include <vector>

// using namespace std::chrono_literals;

// class ImageReceiverNode : public rclcpp::Node
// {
// public:
//     ImageReceiverNode()
//         : Node("image_receiver_node"), socket_fd_(-1)
//     {
//         // Initialize UDP socket
//         setup_udp_socket();

//         // Setup ROS 2 publisher
//         image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);

//         // Create a timer to periodically check for new data
//         timer_ = this->create_wall_timer(100ms, std::bind(&ImageReceiverNode::receive_data, this));
//     }

// private:
//     void setup_udp_socket()
//     {
//         // Create socket
//         socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
//         if (socket_fd_ < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
//             rclcpp::shutdown();
//         }

//         // Bind socket to a port
//         sockaddr_in server_addr;
//         server_addr.sin_family = AF_INET;
//         server_addr.sin_addr.s_addr = INADDR_ANY;
//         server_addr.sin_port = htons(9091);

//         if (bind(socket_fd_, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
//             close(socket_fd_);
//             rclcpp::shutdown();
//         }
//     }

//     void receive_data()
// {
//     if (socket_fd_ < 0) {
//         RCLCPP_ERROR(this->get_logger(), "Socket not initialized.");
//         return;
//     }
//     RCLCPP_INFO(this->get_logger(), "socket initialialized");
//     std::vector<uint8_t> recv_buffer(4096);
//     sockaddr_in sender_addr;
//     socklen_t sender_addr_len = sizeof(sender_addr);

//     int32_t bytes_received = recvfrom(socket_fd_, recv_buffer.data(), recv_buffer.size(), 0,
//                                       (sockaddr*)&sender_addr, &sender_addr_len);

//     if (bytes_received > 0) {
//         RCLCPP_INFO(this->get_logger(), "Byted received");
//         if (image_data_.empty()) {
//             // Extract total number of packets from the first packet
//             if (bytes_received >= static_cast<int32_t>(sizeof(int32_t))) {
//                 int32_t total_packets;
//                 std::memcpy(&total_packets, recv_buffer.data(), sizeof(total_packets));
//                 total_packets_ = total_packets;

//                 // Calculate expected image size
//                 if (total_packets > 0) {
//                     expected_image_size_ = static_cast<size_t>(total_packets) * 4096; // 4096 is the packet size
//                     try {
//                         image_data_.reserve(expected_image_size_);
//                     } catch (const std::length_error& e) {
//                         RCLCPP_ERROR(this->get_logger(), "Failed to reserve vector space: %s", e.what());
//                         return;
//                     }
//                 } else {
//                     RCLCPP_ERROR(this->get_logger(), "Invalid total packets count: %d", total_packets);
//                     return;
//                 }
//             } else {
//                 RCLCPP_ERROR(this->get_logger(), "First packet did not contain enough data for total packets.");
//                 return;
//             }
//         }
//         RCLCPP_INFO(this->get_logger(), "APPEND DATA IMAGE");

//         // Append received packet data
//         image_data_.insert(image_data_.end(), recv_buffer.begin(), recv_buffer.begin() + bytes_received);

//         // Check if we have received the expected amount of data
//         if (image_data_.size() >= expected_image_size_) {
//             if (!image_data_.empty()) {
//                 try {
//                     cv::Mat img = cv::imdecode(image_data_, cv::IMREAD_COLOR);

//                     if (!img.empty()) {
//                         auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
//                         RCLCPP_INFO(this->get_logger(), "PUBLISHING IMAGE");
//                         image_pub_->publish(*msg);
//                     } else {
//                         RCLCPP_ERROR(this->get_logger(), "Failed to decode image from data. Data size: %zu", image_data_.size());
//                     }

//                 } catch (const cv::Exception& e) {
//                     RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
//                 } catch (const std::bad_alloc& e) {
//                     RCLCPP_ERROR(this->get_logger(), "Memory allocation failed: %s", e.what());
//                 } catch (...) {
//                     RCLCPP_ERROR(this->get_logger(), "Unknown error occurred during image decoding.");
//                 }

//                 // Reset for next image
//                 image_data_.clear();
//                 expected_image_size_ = 0; // Reset expected size
//             } else {
//                 RCLCPP_ERROR(this->get_logger(), "Image data buffer is empty.");
//             }
//         }
//     } else {
//         RCLCPP_WARN(this->get_logger(), "No data received or error in receiving data.");
//     }
// }


//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     int socket_fd_;
//     std::vector<uint8_t> image_data_;
//     size_t expected_image_size_; // Set this according to the expected image size
//     int32_t total_packets_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ImageReceiverNode>());
//     rclcpp::shutdown();
//     return 0;
// }


//_______________________________________________________________________________________________________________________________
//Version 0, geringe Auflösung, Kommunikation mit VideoStream.cpp, ohne Open CV
// /*#include <sys/socket.h>
// #include <arpa/inet.h>
// #include <sensor_msgs/msg/image.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <vector>
// #include <unordered_map>
// #include <chrono>

// using namespace std::chrono_literals;

// struct PacketHeader {
//     int32_t ImageId;
//     int32_t SegmentIndex;
//     int32_t NumSegments;
//     int32_t TotalSize;
//     int32_t SegmentSize;
// };

// class RgbCameraReceiver : public rclcpp::Node {
// public:
//     RgbCameraReceiver() : Node("rgb_camera_receiver_main")  {
//         socketfd_ = socket(AF_INET, SOCK_DGRAM, 0);
//         if (socketfd_ < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
//             return;
//         }

//         sockaddr_in server_addr{};
//         server_addr.sin_family = AF_INET;
//         server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
//         server_addr.sin_port = htons(UDP_PORT);

//         if (bind(socketfd_, (const sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
//             return;
//         }

//         publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
//         timer_ = this->create_wall_timer(10ms, std::bind(&RgbCameraReceiver::receiveData, this));
//     }

//     ~RgbCameraReceiver() {
//         if (socketfd_ != -1) {
//             close(socketfd_);
//         }
//     }

// private:
//     void receiveData() {
//         constexpr size_t MAX_UDP_SIZE = 887500;
//         std::vector<uint8_t> buffer(MAX_UDP_SIZE);
//         sockaddr_in client_addr{};
//         socklen_t addr_len = sizeof(client_addr);
//         ssize_t len = recvfrom(socketfd_, buffer.data(), buffer.size(), 0, (struct sockaddr*)&client_addr, &addr_len);

//         if (len < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to receive data");
//             return;
//         }

//         if (len < sizeof(PacketHeader)) {
//             RCLCPP_ERROR(this->get_logger(), "Packet is too small");
//             return;
//         }

//         PacketHeader header;
//         memcpy(&header, buffer.data(), sizeof(PacketHeader));

//         int32_t image_id = header.ImageId;
//         int32_t segment_index = header.SegmentIndex;
//         int32_t num_segments = header.NumSegments;
//         int32_t total_size = header.TotalSize;
//         int32_t segment_size = header.SegmentSize;

//         size_t data_offset = sizeof(PacketHeader);
//         size_t data_length = len - data_offset;

//         // Add logging to debug header values
//         RCLCPP_INFO(this->get_logger(), "Received packet: image_id=%d, segment_index=%d, num_segments=%d, total_size=%d, segment_size=%d, data_length=%zu",
//                     image_id, segment_index, num_segments, total_size, segment_size, data_length);

//         if (data_length > segment_size) {
//             RCLCPP_ERROR(this->get_logger(), "Data length exceeds segment size");
//             return;
//         }

//         auto &image_buffer = images_[image_id];
//         image_buffer.packets_received++;
//         image_buffer.data.insert(image_buffer.data.end(), buffer.begin() + data_offset, buffer.begin() + data_offset + data_length);

//         if (image_buffer.packets_received == num_segments) {
//             publishImage(image_id, total_size);
//             images_.erase(image_id);
//         }
//     }

//     void publishImage(int32_t image_id, int32_t total_size) {
//         auto &image_buffer = images_[image_id];

//         if (image_buffer.data.size() != total_size) {
//             RCLCPP_ERROR(this->get_logger(), "Reassembled image size mismatch: expected=%d, actual=%zu",
//                          total_size, image_buffer.data.size());
//             return;
//         }

//         int width = IMAGE_WIDTH;
//         int height = IMAGE_HEIGHT;
//         int channels = 4; // Assuming RGBA

//         auto image_msg = sensor_msgs::msg::Image();
//         image_msg.header.stamp = this->now();
//         image_msg.header.frame_id = "camera_frame";
//         image_msg.height = height;
//         image_msg.width = width;
//         image_msg.encoding = "rgba8";
//         image_msg.is_bigendian = false;
//         image_msg.step = width * channels;
//         image_msg.data = std::move(image_buffer.data);

//         publisher_->publish(image_msg);
//     }

//     static constexpr int UDP_PORT = 12345;
//     static constexpr int IMAGE_WIDTH = 127;  // Example width, adjust as necessary
//     static constexpr int IMAGE_HEIGHT = 127; // Example height, adjust as necessary

//     struct ImageBuffer {
//         int packets_received = 0;
//         std::vector<uint8_t> data;
//     };

//     int socketfd_;
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     std::unordered_map<int32_t, ImageBuffer> images_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<RgbCameraReceiver>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
// */

// #include <sys/socket.h>
// #include <arpa/inet.h>
// #include <sensor_msgs/msg/image.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <vector>
// #include <unordered_map>
// #include <chrono>

// using namespace std::chrono_literals;

// struct PacketHeader {
//     int32_t ImageId;
//     int32_t SegmentIndex;
//     int32_t NumSegments;
//     int32_t TotalSize;
//     int32_t SegmentSize;
// };

// class RgbCameraReceiver : public rclcpp::Node {
// public:
//     RgbCameraReceiver() : Node("rgb_camera_receiver_main") {
//         socketfd_ = socket(AF_INET, SOCK_DGRAM, 0);
//         if (socketfd_ < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
//             return;
//         }

//         // Set the receive buffer size
//         int buffer_size = 2 * 1024 * 1024; // 2MB buffer size
//         if (setsockopt(socketfd_, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size)) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to set socket receive buffer size");
//             return;
//         }

//         sockaddr_in server_addr{};
//         server_addr.sin_family = AF_INET;
//         server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
//         server_addr.sin_port = htons(UDP_PORT);

//         if (bind(socketfd_, (const sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
//             return;
//         }

//         publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
//         timer_ = this->create_wall_timer(10ms, std::bind(&RgbCameraReceiver::receiveData, this));
//     }

//     ~RgbCameraReceiver() {
//         if (socketfd_ != -1) {
//             close(socketfd_);
//         }
//     }

// private:
//     void receiveData() {
//         constexpr size_t MAX_UDP_SIZE = 18928;
//         std::vector<uint8_t> buffer(MAX_UDP_SIZE);
//         sockaddr_in client_addr{};
//         socklen_t addr_len = sizeof(client_addr);
//         ssize_t len = recvfrom(socketfd_, buffer.data(), buffer.size(), 0, (struct sockaddr*)&client_addr, &addr_len);

//         if (len < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to receive data");
//             return;
//         }

//         if (len < sizeof(PacketHeader)) {
//             RCLCPP_ERROR(this->get_logger(), "Packet is too small");
//             return;
//         }

//         PacketHeader header;
//         memcpy(&header, buffer.data(), sizeof(PacketHeader));

//         int32_t image_id = header.ImageId;
//         int32_t segment_index = header.SegmentIndex;
//         int32_t num_segments = header.NumSegments;
//         int32_t total_size = header.TotalSize;
//         int32_t segment_size = header.SegmentSize;

//         size_t data_offset = sizeof(PacketHeader);
//         //size_t data_length = len - data_offset;

//         // Add logging to debug header values
//        // RCLCPP_INFO(this->get_logger(), "Received packet: image_id=%d, segment_index=%d, num_segments=%d, total_size=%d, segment_size=%d",
//           //         image_id, segment_index, num_segments, total_size, segment_size);

                    

//         // if (data_length > segment_size) {
//         //     RCLCPP_ERROR(this->get_logger(), "Data length exceeds segment size");
//         //     return;
//         // }

//         auto &image_buffer = images_[image_id];
//         image_buffer.packets_received++;
//         image_buffer.data.insert(image_buffer.data.end(), buffer.begin() + data_offset, buffer.begin() + data_offset + segment_size);

//         if (image_buffer.packets_received == num_segments) {
//             publishImage(image_id, total_size);
//             images_.erase(image_id);
//         }
//     }

//     void publishImage(int32_t image_id, int32_t total_size) {
//         auto &image_buffer = images_[image_id];

//         if (image_buffer.data.size() != total_size) {
//             RCLCPP_ERROR(this->get_logger(), "Reassembled image size mismatch: expected=%d, actual=%zu",
//                          total_size, image_buffer.data.size());
//             return;
//         }

//         int width = IMAGE_WIDTH;
//         int height = IMAGE_HEIGHT;
//         int channels = 4; // Assuming RGBA

//         auto image_msg = sensor_msgs::msg::Image();
//         image_msg.header.stamp = this->now();
//         image_msg.header.frame_id = "camera_frame";
//         image_msg.height = height;
//         image_msg.width = width;
//         image_msg.encoding = "rgba8";
//         image_msg.is_bigendian = false;
//         image_msg.step = width * channels;
//         image_msg.data = std::move(image_buffer.data);

//         //open CV integration, light up picture
        
//         publisher_->publish(image_msg);
//     }

//     static constexpr int UDP_PORT = 12345;
//     static constexpr int IMAGE_WIDTH = 256;  // Example width, adjust as necessary
//     static constexpr int IMAGE_HEIGHT = 256; // Example height, adjust as necessary

//     struct ImageBuffer {
//         int packets_received = 0;
//         std::vector<uint8_t> data;
//     };

//     int socketfd_;
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     std::unordered_map<int32_t, ImageBuffer> images_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<RgbCameraReceiver>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
