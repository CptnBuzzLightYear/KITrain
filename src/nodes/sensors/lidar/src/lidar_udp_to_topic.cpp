#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include "lidar_msgs/msg/ground_truth.hpp"
#include "lidar_msgs/msg/ground_truth_array.hpp"
#include <fstream>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2/LinearMath/Quaternion.h>



using std::placeholders::_1;

class LidarUDPPublisher : public rclcpp::Node
{
    public:
    LidarUDPPublisher() : Node("lidar_udp_publisher")
    {
        publisher_ = this ->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

        groundTruthPublisher_ = this -> create_publisher<lidar_msgs::msg::GroundTruthArray>("LiDARGroundTruth", 10);

        boundingBoxPublisher_ = this-> create_publisher<visualization_msgs::msg::MarkerArray>("LiDAR_BoundingBoxes", 10);


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
            if(bind(sockfd_,(const sockaddr*)&servaddr, sizeof(servaddr)) < 0){
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
    uint16_t previous_frame_id_ =0; // ID of receiving upd pointcloud
    std::vector<double>pointcloud_buffer_; //accumulate Pointcloud
    
    void receiveLoop(){      
        const uint16_t MAX_UDP_PACKET_SIZE = 18928;
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
           //  RCLCPP_INFO(this->get_logger(), "Received LIDAR UDP");

            if(!convertToPointCloud(buffer, bytes_received)){
                RCLCPP_ERROR(this->get_logger(), "Error converting UDP data to PointCloud2 message");
                continue;
            }
        }
    }
    bool convertToPointCloud(const char *data, size_t size){       
        std::string received_data(data, size);
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
            RCLCPP_INFO(this->get_logger(),"Received HEADER");
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
            for(size_t i = 4; i + 6 < tokens.size(); i +=7){
                double x = std::stod(tokens[i])/100; // divide by 100 - scale factor from cm (UE) to real world (m)
                pointcloud_buffer_.push_back(x);

                double y = - std::stod(tokens[i + 1])/100; //180° y mirroring, because of UE initial coordinate system x, -y, z.
                pointcloud_buffer_.push_back(y);

                double z = std::stod(tokens[i + 2])/100;
                pointcloud_buffer_.push_back(z);
            }
            //update previous frame ID
            previous_frame_id_ = frame_id;
        }
      else if (tokens[0] =="2"){
            //proceed Ground Truth Bounding Box information, Derive Labelling file. 
           // RCLCPP_INFO(this->get_logger(), "GroundTruth Labelinstructions received");            
           
            //process payload data
            uint16_t frame_id = std::stoi(tokens[1]);
                 //Check if ID has changed
            if (frame_id != previous_frame_id_){
                if(!groundTruth_buffer_.empty()){
                    //publish complete label set of previous ID
                    publishGroundTruth(groundTruth_buffer_);
                    // exportGroundTruthToFile(groundTruth_buffer_);
                    publishBoundingBoxes(groundTruth_buffer_);
                    groundTruth_buffer_.clear();
                }
            }
            // Append data to buffer starting from tokens[4]
        for (size_t j = 4; j < tokens.size(); ++j) {
            std::vector<std::string> object_tokens;
            std::istringstream object_iss(tokens[j]);
            std::string object_token;
            while (std::getline(object_iss, object_token, ' ')) { // Assuming space is the secondary delimiter
                object_tokens.push_back(object_token);
            }      
            if (object_tokens.size() != 8) {
                RCLCPP_WARN(this->get_logger(), "Unexpected number of object tokens: %zu", object_tokens.size());
                continue;
            }
            // Skip objects with tag "Unknown"
            if (object_tokens[7] == "Unknown") {
                continue;
            }
            //Append data to buffer
                lidar_msgs::msg::GroundTruth groundtruth_msg;
                groundtruth_msg.id              =  frame_id;
                groundtruth_msg.centerx         =  std::stod(object_tokens[0]) / 100;
                groundtruth_msg.centery         = -std::stod(object_tokens[1]) / 100;  // 180° y mirroring
                groundtruth_msg.centerz         =  std::stod(object_tokens[2]) / 100;
                groundtruth_msg.length_x        =  std::stod(object_tokens[3]) / 50;
                groundtruth_msg.width_y         =  std::stod(object_tokens[4]) / 50;
                groundtruth_msg.height_z        =  std::stod(object_tokens[5]) / 50;
                groundtruth_msg.yaw             =  std::stod(object_tokens[6]);
                groundtruth_msg.tag             =  object_tokens[7];             // Correct handling for label

                groundTruth_buffer_.push_back(groundtruth_msg);
        }
      }
    else{
        RCLCPP_WARN(this->get_logger(), "Unknown packet type received");        
        }
        return true;
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
        for (size_t k=0; k < pointcloud_data.size(); k +=3){
            float x = static_cast<float>(pointcloud_data[k]);
            float y = static_cast<float>(pointcloud_data[k + 1]);
            float z = static_cast<float>(pointcloud_data[k + 2]);
            std::memcpy(&pointcloud_msg.data[k * sizeof(float)], &x, sizeof(float));
            std::memcpy(&pointcloud_msg.data[(k + 1) * sizeof(float)], &y, sizeof(float));
            std::memcpy(&pointcloud_msg.data[(k + 2) * sizeof(float)], &z, sizeof(float));
        }

    // fill in other fields
        pointcloud_msg.height = 1; // how many rows has the scna? replace 128
        pointcloud_msg.width = pointcloud_data.size()/3;; //Number of points, each consits of 3 coordinates
        pointcloud_msg.is_bigendian = false; // Endianness of data
        pointcloud_msg.point_step = sizeof(float)*3; //Size of each point XYZ
        pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width; // Total size of data buffer

        //pubish pointcloud

    publisher_->publish(pointcloud_msg);

    }

    void publishGroundTruth(const std::vector<lidar_msgs::msg::GroundTruth>& buffer) {
        lidar_msgs::msg::GroundTruthArray array_msg;
        array_msg.header.stamp = this->now();
        array_msg.header.frame_id = "frameidLiDARLabel";
        array_msg.labels = buffer;
        groundTruthPublisher_->publish(array_msg);       
    } 

    void publishBoundingBoxes(const std::vector<lidar_msgs::msg::GroundTruth>& buffer) {
    // Create a MarkerArray to hold the bounding boxes
    visualization_msgs::msg::MarkerArray boundingbox_array;
    
    // Keep track of marker IDs that are still valid in this frame
    std::set<int> valid_marker_ids;

    // Add new bounding boxes
    for (size_t l = 0; l < buffer.size(); ++l) {
        const auto& groundTruth = buffer[l]; 
        
        visualization_msgs::msg::Marker boundingbox;
        boundingbox.header.stamp = this->now();
        boundingbox.header.frame_id = "base_link";
        boundingbox.ns = "boxes";
        boundingbox.id = groundTruth.id;
        boundingbox.type = visualization_msgs::msg::Marker::CUBE;
        boundingbox.action = visualization_msgs::msg::Marker::ADD;
        boundingbox.pose.position.x = groundTruth.centerx;
        boundingbox.pose.position.y = groundTruth.centery;
        boundingbox.pose.position.z = groundTruth.centerz;

        tf2::Quaternion quat;
        quat.setRPY(0, 0, groundTruth.yaw);

        boundingbox.pose.orientation.x = quat.x();
        boundingbox.pose.orientation.y = quat.y();
        boundingbox.pose.orientation.z = quat.z();
        boundingbox.pose.orientation.w = quat.w();

        boundingbox.scale.x = groundTruth.length_x;
        boundingbox.scale.y = groundTruth.width_y;
        boundingbox.scale.z = groundTruth.height_z;
        
        if (groundTruth.tag == "Person") {   
        boundingbox.color.a = 1.0;  // Transparency
        boundingbox.color.r = 0.0;
        boundingbox.color.g = 0.9;
        boundingbox.color.b = 0.9;
        }
        else if (groundTruth.tag == "Waggon") {   
        boundingbox.color.a = 1.0;  // Transparency
        boundingbox.color.r = 0.8;
        boundingbox.color.g = 0.0;
        boundingbox.color.b = 0.0;
        }
        else if (groundTruth.tag == "Signal") {   
        boundingbox.color.a = 1.0;  // Transparency
        boundingbox.color.r = 0.8;
        boundingbox.color.g = 0.0;
        boundingbox.color.b = 0.9;
        }
        else if (groundTruth.tag == "Plant") {   
        boundingbox.color.a = 1.0;  // Transparency
        boundingbox.color.r = 0.8;
        boundingbox.color.g = 0.4;
        boundingbox.color.b = 0.9;
        }

        boundingbox_array.markers.push_back(boundingbox);
        valid_marker_ids.insert(groundTruth.id);  // Track valid IDs
    }

    // Remove old markers
    static std::set<int> previous_marker_ids;
    for (int id : previous_marker_ids) {
        if (valid_marker_ids.find(id) == valid_marker_ids.end()) {
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.header.stamp = this->now();
            delete_marker.header.frame_id = "base_link";
            delete_marker.ns = "boxes";
            delete_marker.id = id;
            delete_marker.action = visualization_msgs::msg::Marker::DELETE;

            boundingbox_array.markers.push_back(delete_marker);
        }
    }

    // Update the previous marker IDs with the current ones
    previous_marker_ids = valid_marker_ids;

    // Publish the MarkerArray
    boundingBoxPublisher_->publish(boundingbox_array);
}

    
    void exportGroundTruthToFile(const std::vector<lidar_msgs::msg::GroundTruth>& groundTruthBuffer) {
    // Open the file in append mode
    std::ofstream file("LiDAR_ground_truth.txt", std::ios::app);

    // Check if the file is open
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open ground_truth_output.txt for writing");
        return;
    }

    // Write each Box message to the file
    for (const auto& groundtruth : groundTruthBuffer) {
        file << groundtruth.centerx << ", "
             << groundtruth.centery << ", "
             << groundtruth.centerz << ", "
             << groundtruth.length_x << ", "
             << groundtruth.width_y << ", "
             << groundtruth.height_z << ", "
             << groundtruth.yaw << ", "
             << groundtruth.tag << "\n";
    }

    // Close the file
    file.close();

   // RCLCPP_INFO(this->get_logger(), "Ground truth data exported to lidar_ground_truth.txt");
}

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<lidar_msgs::msg::GroundTruthArray>::SharedPtr groundTruthPublisher_;
    std::vector<lidar_msgs::msg::GroundTruth> groundTruth_buffer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundingBoxPublisher_;
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
