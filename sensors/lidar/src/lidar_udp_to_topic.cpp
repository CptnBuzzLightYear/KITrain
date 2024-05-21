#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

using std::placeholders::_1;

class LidarUDPPublisher : public rclcpp::Node
{
    public:
    LidarUDPPublisher() : Node("lidar_udp_publisher")
    {
        publisher_ = this ->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

        // Create UDP socket for receiving LIDAR data
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ <0){
            perror("Error: creating LIDAR UDP socket");
            return;
        }

        //Bind socket to address and port
            sockaddr_in servaddr;
            servaddr.sin_family = AF_INET;
            servaddr.sin_addr.s_addr = htonl(INADDR_ANY); //to any available interface
            servaddr.sin_port = htons(33571);             //PORT for LIDAR stream
            if(bind(sockfd_,(const sockaddr*)&servaddr, sizeof(servaddr)) < 0)
            {
                std::cerr << "Error: Failed to bind socket" << std::endl;
                close(sockfd_);
                return;
            }

            //Start receiving UDP packets in a separate thread
            receive_thread_ = std::thread(&LidarUDPPublisher::receiveLoop, this);
    }
    
    ~LidarUDPPublisher()
    {
        close(sockfd_);
    }

    private: 
     
    uint16_t previous_frame_id_; // ID of receiving upd pointcloud
    std::vector<double>pointcloud_buffer_; //accumulate Pointcloud
    
    void receiveLoop(){
        
        const uint16_t MAX_UDP_PACKET_SIZE = 1500;
        char buffer[MAX_UDP_PACKET_SIZE];
        
        while (rclcpp::ok()){
            //Receive UDP Data
            
            sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            size_t bytes_received = recvfrom(sockfd_, buffer, MAX_UDP_PACKET_SIZE,0 , (struct sockaddr *)&client_addr, &addr_len);


            if(bytes_received <= 0){
                perror("Error receiving LIDAR UDP data");
                continue;
            }
             RCLCPP_INFO(this->get_logger(), "Received LIDAR UDP");

            if(!convertToPointCloud(buffer, bytes_received, pointcloud_msg)){
                RCLCPP_ERROR(this->get_logger(), "Error converting UDP data to PointCloud2 message");
                continue;
            }
            publisher_->publish(pointcloud_msg);
        }
    }
    bool convertToPointCloud(const char *data, size_t size, sensor_msgs::msg::PointCloud2 &pointcloud_msg){
        //decoding UDP Stream from Unreal Engine LiDAR
        //Parse header
 /*Groß          
        if(size < sizeof(uint16_t)){
            RCLCPP_WARN(this->get_logger(), "Received UDP packet is too small to contain header");
           return false;
        }
    
          
        const uint16_t *header_ptr =reinterpret_cast<const uint16_t *>(data);
        uint16_t frame_id = ntohs(*header_ptr);

        //Parse payload
       if(size <sizeof(uint16_t)*6){
            RCLCPP_WARN(this->get_logger(), "Received UDP packet too small to contain payload");
            return false;
        }
    RCLCPP_INFO(this->get_logger(), "POINTS POINTS POINTS POINTS");
       /* const uint16_t *payload_ptr = reinterpret_cast<const uint16_t *> (data+sizeof(uint16_t));
     
        uint16_t nBeamsHor = ntohs(*payload_ptr);
        payload_ptr++;
        uint16_t nBeamsVert = ntohs(*payload_ptr);
        payload_ptr++;
        float threshIncLow = *reinterpret_cast<const float *>(payload_ptr);
        payload_ptr++;
        float threshIncHigh = *reinterpret_cast<const float *>(payload_ptr);
        payload_ptr++;
        float tX = *reinterpret_cast<const float *>(payload_ptr);
        payload_ptr++;
        float tY = *reinterpret_cast<const float *>(payload_ptr);
        payload_ptr++;
        float tZ = *reinterpret_cast<const float *>(payload_ptr);
        payload_ptr++;
        float roll = *reinterpret_cast<const float *>(payload_ptr);
        payload_ptr++;
        float pitch = *reinterpret_cast<const float *>(payload_ptr);
        payload_ptr++;
        float yaw = *reinterpret_cast<const float *>(payload_ptr);
        payload_ptr++;*/


        std::string received_data(data, size);
        RCLCPP_INFO(this->get_logger(),"Received string: %s", received_data.c_str());
        std::istringstream ss(received_data);
        std::string token;
        std::vector<std::string> tokens;
     
        
      //  size_t count = 0;
        while (std::getline(ss, token, '_')){
            tokens.push_back(token);
        }
        //Check first entry of UDP Stream 0-header, 1- payload
        if (tokens.empty()){
            RCLCPP_WARN(this->get_logger(), "Received LiDar Stream is empty");
            return false;
        }
        if(tokens[0]=="0"){
            //process header data
        }
        else if (tokens[0] =="1"){
            //process payload data
            uint16_t frame_id = std::stoi(tokens[1]);
            
            //Check if ID has changed
            if (frame_id != previous_frame_id_){

                //new ID, process existing buffer.
                if(!pointcloud_buffer_.empty()){
                    publishPointcloud(pointcloud_buffer_);
                    pointcloud_buffer_.clear();
                }
            }

            //Append data to buffer
            for(size_t i = 2; i + 2 < tokens.size(); i +=3){
                double x = std::stod(tokens[i]);
                pointcloud_buffer_.push_back(x);

                double y = std::stod(tokens[i + 1]);
                pointcloud_buffer_.push_back(y);

                double z = std::stod(tokens[i + 2]);
                pointcloud_buffer_.push_back(z);
            }
            //update previous frame ID
            previous_frame_id_ = frame_id;
            else{
                RCLCPP_WARN(this->get_logger(), "Unknown packet type received");
            }

        }

    }
    void publishPointcloud (const std::vector<double> &pointcloud_data){
        
        //construct Pointcloud Message
        sensor_msgs::msg::PointCloud2 pointcloud_msg;

        //Fill the header
        pointcloud_msg.header.frame_id = "lidar_frame"; //Set frame ID
        pointcloud_msg.header.stamp = rclcpp::Clock().now(); //Set timestamp

        //Define fields
        sensor_msgs::msg::PointField x_field, y_field, z_field; 
        x_field.name = "x";
        x_field.offset = 0;
        x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        x_field.count = 1;
        y_field.name = "y";
        y_field.offset = 4; //Offset of y in bytes (4bytes for x)
        y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        y_field.count = 1;
        z_field.name = "z";
        z_field.offset = 8; //Offset of z in bytes (4 bytes for x, 4 bytes for y -> sum =8)
        z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        z_field.count = 1;

        pointcloud_msg.fields = {x_field, y_field, z_field};

        //Resize data array and copy point cloud data
        pointcloud_msg.data.resize(pointcloud_data.size() * sizeof(float));
        for (size_t i=0; i < pointcloud_data.size(); i +=3){
            float x = static_cast<float>(pointcloud_data[i]);
            float y = static_cast<float>(pointcloud_data[i + 1]);
            float z = static_cast<float>(pointcloud_data[i + 2]);

        }

    }



        //calculate total number of points
      /*HERE 2  size_t num_points = nBeamsHor * nBeamsVert;

       
        
        

        //Set point cloud data
        pointcloud_msg.height = 128; // how many rows has the scna? replace 128
        pointcloud_msg.width = num_points; //Number of points
        pointcloud_msg.point_step = sizeof(float)*3; //Size of each point XYZ
        pointcloud_msg.row_step = pointcloud_msg.point_step * num_points; // Total size of data buffer

        //Resize data buffer
        pointcloud_msg.data.resize(pointcloud_msg.row_step);
        //Copy received data to pointcloud message data buffer
       // std::memcpy(pointcloud_msg.data.data(), data, size);

        //Fill in point cloud data
        float *data_ptr = reinterpret_cast<float *>(pointcloud_msg.data.data());
        for (size_t i = 0; i < num_points; i++){
            //calculate XYZ coordinates for each point
            float x = i;
            float y = i;
            float z = i;

            //store XYZ coordinated in point vloud data buffer
            *(data_ptr++) = x;
            *(data_ptr++) = y;
            *(data_ptr++) = z;

        }
*/
     
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    int sockfd_;
    std::thread receive_thread_;
    std::vector<char> udp_receive_buffer_;
};
int main(int argc, char **argv){
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarUDPPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}