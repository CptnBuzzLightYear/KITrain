/*#include <sys/socket.h>
#include <arpa/inet.h>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <unordered_map>
#include <chrono>

using namespace std::chrono_literals;

struct PacketHeader {
    int32_t ImageId;
    int32_t SegmentIndex;
    int32_t NumSegments;
    int32_t TotalSize;
    int32_t SegmentSize;
};

class RgbCameraReceiver : public rclcpp::Node {
public:
    RgbCameraReceiver() : Node("rgb_camera_receiver_main")  {
        socketfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socketfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        server_addr.sin_port = htons(UDP_PORT);

        if (bind(socketfd_, (const sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
            return;
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&RgbCameraReceiver::receiveData, this));
    }

    ~RgbCameraReceiver() {
        if (socketfd_ != -1) {
            close(socketfd_);
        }
    }

private:
    void receiveData() {
        constexpr size_t MAX_UDP_SIZE = 887500;
        std::vector<uint8_t> buffer(MAX_UDP_SIZE);
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        ssize_t len = recvfrom(socketfd_, buffer.data(), buffer.size(), 0, (struct sockaddr*)&client_addr, &addr_len);

        if (len < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive data");
            return;
        }

        if (len < sizeof(PacketHeader)) {
            RCLCPP_ERROR(this->get_logger(), "Packet is too small");
            return;
        }

        PacketHeader header;
        memcpy(&header, buffer.data(), sizeof(PacketHeader));

        int32_t image_id = header.ImageId;
        int32_t segment_index = header.SegmentIndex;
        int32_t num_segments = header.NumSegments;
        int32_t total_size = header.TotalSize;
        int32_t segment_size = header.SegmentSize;

        size_t data_offset = sizeof(PacketHeader);
        size_t data_length = len - data_offset;

        // Add logging to debug header values
        RCLCPP_INFO(this->get_logger(), "Received packet: image_id=%d, segment_index=%d, num_segments=%d, total_size=%d, segment_size=%d, data_length=%zu",
                    image_id, segment_index, num_segments, total_size, segment_size, data_length);

        if (data_length > segment_size) {
            RCLCPP_ERROR(this->get_logger(), "Data length exceeds segment size");
            return;
        }

        auto &image_buffer = images_[image_id];
        image_buffer.packets_received++;
        image_buffer.data.insert(image_buffer.data.end(), buffer.begin() + data_offset, buffer.begin() + data_offset + data_length);

        if (image_buffer.packets_received == num_segments) {
            publishImage(image_id, total_size);
            images_.erase(image_id);
        }
    }

    void publishImage(int32_t image_id, int32_t total_size) {
        auto &image_buffer = images_[image_id];

        if (image_buffer.data.size() != total_size) {
            RCLCPP_ERROR(this->get_logger(), "Reassembled image size mismatch: expected=%d, actual=%zu",
                         total_size, image_buffer.data.size());
            return;
        }

        int width = IMAGE_WIDTH;
        int height = IMAGE_HEIGHT;
        int channels = 4; // Assuming RGBA

        auto image_msg = sensor_msgs::msg::Image();
        image_msg.header.stamp = this->now();
        image_msg.header.frame_id = "camera_frame";
        image_msg.height = height;
        image_msg.width = width;
        image_msg.encoding = "rgba8";
        image_msg.is_bigendian = false;
        image_msg.step = width * channels;
        image_msg.data = std::move(image_buffer.data);

        publisher_->publish(image_msg);
    }

    static constexpr int UDP_PORT = 12345;
    static constexpr int IMAGE_WIDTH = 127;  // Example width, adjust as necessary
    static constexpr int IMAGE_HEIGHT = 127; // Example height, adjust as necessary

    struct ImageBuffer {
        int packets_received = 0;
        std::vector<uint8_t> data;
    };

    int socketfd_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<int32_t, ImageBuffer> images_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RgbCameraReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
*/

#include <sys/socket.h>
#include <arpa/inet.h>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <unordered_map>
#include <chrono>

using namespace std::chrono_literals;

struct PacketHeader {
    int32_t ImageId;
    int32_t SegmentIndex;
    int32_t NumSegments;
    int32_t TotalSize;
    int32_t SegmentSize;
};

class RgbCameraReceiver : public rclcpp::Node {
public:
    RgbCameraReceiver() : Node("rgb_camera_receiver_main") {
        socketfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socketfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        // Set the receive buffer size
        int buffer_size = 2 * 1024 * 1024; // 2MB buffer size
        if (setsockopt(socketfd_, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set socket receive buffer size");
            return;
        }

        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        server_addr.sin_port = htons(UDP_PORT);

        if (bind(socketfd_, (const sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
            return;
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&RgbCameraReceiver::receiveData, this));
    }

    ~RgbCameraReceiver() {
        if (socketfd_ != -1) {
            close(socketfd_);
        }
    }

private:
    void receiveData() {
        constexpr size_t MAX_UDP_SIZE = 18928;
        std::vector<uint8_t> buffer(MAX_UDP_SIZE);
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        ssize_t len = recvfrom(socketfd_, buffer.data(), buffer.size(), 0, (struct sockaddr*)&client_addr, &addr_len);

        if (len < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive data");
            return;
        }

        if (len < sizeof(PacketHeader)) {
            RCLCPP_ERROR(this->get_logger(), "Packet is too small");
            return;
        }

        PacketHeader header;
        memcpy(&header, buffer.data(), sizeof(PacketHeader));

        int32_t image_id = header.ImageId;
        int32_t segment_index = header.SegmentIndex;
        int32_t num_segments = header.NumSegments;
        int32_t total_size = header.TotalSize;
        int32_t segment_size = header.SegmentSize;

        size_t data_offset = sizeof(PacketHeader);
        //size_t data_length = len - data_offset;

        // Add logging to debug header values
        RCLCPP_INFO(this->get_logger(), "Received packet: image_id=%d, segment_index=%d, num_segments=%d, total_size=%d, segment_size=%d",
                    image_id, segment_index, num_segments, total_size, segment_size);

                    

        // if (data_length > segment_size) {
        //     RCLCPP_ERROR(this->get_logger(), "Data length exceeds segment size");
        //     return;
        // }

        auto &image_buffer = images_[image_id];
        image_buffer.packets_received++;
        image_buffer.data.insert(image_buffer.data.end(), buffer.begin() + data_offset, buffer.begin() + data_offset + segment_size);

        if (image_buffer.packets_received == num_segments) {
            publishImage(image_id, total_size);
            images_.erase(image_id);
        }
    }

    void publishImage(int32_t image_id, int32_t total_size) {
        auto &image_buffer = images_[image_id];

        if (image_buffer.data.size() != total_size) {
            RCLCPP_ERROR(this->get_logger(), "Reassembled image size mismatch: expected=%d, actual=%zu",
                         total_size, image_buffer.data.size());
            return;
        }

        int width = IMAGE_WIDTH;
        int height = IMAGE_HEIGHT;
        int channels = 4; // Assuming RGBA

        auto image_msg = sensor_msgs::msg::Image();
        image_msg.header.stamp = this->now();
        image_msg.header.frame_id = "camera_frame";
        image_msg.height = height;
        image_msg.width = width;
        image_msg.encoding = "rgba8";
        image_msg.is_bigendian = false;
        image_msg.step = width * channels;
        image_msg.data = std::move(image_buffer.data);

        //open CV integration, light up picture
        
        publisher_->publish(image_msg);
    }

    static constexpr int UDP_PORT = 12345;
    static constexpr int IMAGE_WIDTH = 256;  // Example width, adjust as necessary
    static constexpr int IMAGE_HEIGHT = 256; // Example height, adjust as necessary

    struct ImageBuffer {
        int packets_received = 0;
        std::vector<uint8_t> data;
    };

    int socketfd_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<int32_t, ImageBuffer> images_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RgbCameraReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
