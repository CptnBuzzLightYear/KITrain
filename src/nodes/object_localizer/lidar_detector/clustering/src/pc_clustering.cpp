#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "std_msgs/msg/string.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <vector>

#include <lidar_msgs/msg/ground_truth.hpp>
#include "lidar_msgs/msg/ground_truth_array.hpp"
#include <std_msgs/msg/header.hpp> 
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include "fast_euclidean_clustering.h"
#include <vector>
#include <unordered_map>
#include <algorithm>
using std::placeholders::_1;

class LidarFilter : public rclcpp::Node
{
public:
    LidarFilter()
        : Node("lidar_filter")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/VoxelFilterPub", 1, std::bind(&LidarFilter::topic_callback, this, _1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("LidarClusterdPub", 1);
        PublisherBbox_ = this->create_publisher<lidar_msgs::msg::GroundTruthArray>("BboxClusterPub", 10);
        PublisherBboxViz_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("LiDAR_Clustering_BoundingBoxes", 10);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        unsigned int num_points = msg->width;
        RCLCPP_INFO(this->get_logger(), "INPUT: The number of points in the pointcloud is %i", num_points);

        // Declaring the different pointcloud types
        sensor_msgs::msg::PointCloud2 cloud_out;
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
        pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZ> pass; // Declare the pass object
        float_t height_cut;

        // ROS2 Pointcloud2 to PCL Pointcloud2 Conversion
        pcl_conversions::toPCL(*msg, *cloud);

        // PCL Pointcloud2 to PCL Pointcloud Conversion
        pcl::fromPCLPointCloud2(*cloud, *cloud_xyz);

        height_cut = calc_height(cloud_xyz, std::to_string(num_points) + "_z_values.csv");

        // Ground and Sky segmentation
        pass.setInputCloud(cloud_xyz); // Use the pass object
        pass.setFilterFieldName("z");
        pass.setFilterLimits(height_cut, 5.0); // might need to be a dynamic parameter -1.25
        pass.filter(*cloud_xyz);

        // Clustering Algorithm
        cloud_filtered_xyzrgb = clustering(cloud_xyz, 0.3, cloud_xyz->size() / 1000, cloud_xyz->size() / 5, 0.5); // 1000,

        // PCL Pointcloud to PCL Pointcloud2 Conversion
        pcl::toPCLPointCloud2(*cloud_filtered_xyzrgb, *cloud_filtered);

        // PCL Pointcloud2 to ROS2 Pointcloud2 Conversion
        pcl_conversions::fromPCL(*cloud_filtered, cloud_out);

        unsigned int num_points_out = cloud_out.width;
        RCLCPP_INFO(this->get_logger(), "OUTPUT: The number of points in the pointcloud is %i", num_points_out);

        cloud_out.header.frame_id = msg->header.frame_id;
        cloud_out.header.stamp = msg->header.stamp;

        publisher_->publish(cloud_out);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float tolerance, int min_cluster_size, int max_cluster_size, float quality)
    {
        // Declaring all necessary variables
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_with_i(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZRGB>);
        FastEuclideanClustering<pcl::PointXYZI> fast_euclidean_clustering;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        lidar_msgs::msg::GroundTruth bbox;
        std::vector<lidar_msgs::msg::GroundTruth> bbox_arr;
        lidar_msgs::msg::GroundTruthArray array_msg;
        visualization_msgs::msg::Marker BboxViz;
        visualization_msgs::msg::MarkerArray BboxViz_arr;
        int clustersize = 0;
        std::vector<pcl::PointIndices> clusters;

        // adding Intensity to the pointcloud (aka zeros)
        pcl::copyPointCloud(*cloud, *cloud_with_i);

        // Clustering Parameters
        fast_euclidean_clustering.setInputCloud(cloud_with_i);
        fast_euclidean_clustering.setSearchMethod(tree);
        fast_euclidean_clustering.setClusterTolerance(tolerance);
        fast_euclidean_clustering.setMinClusterSize(min_cluster_size);
        fast_euclidean_clustering.setMaxClusterSize(max_cluster_size);
        fast_euclidean_clustering.setQuality(quality);
        fast_euclidean_clustering.segment(clusters);
        clustersize = clusters.size();
        RCLCPP_INFO(this->get_logger(), "Min_Clustersize: %d", min_cluster_size);
        RCLCPP_INFO(this->get_logger(), "Max_Clustersize: %d", max_cluster_size);
        RCLCPP_INFO(this->get_logger(), "Clustersize: %d", clustersize);

        // Assigning Colors to the clusters, so that they can be visualized
        pcl::copyPointCloud(*cloud_with_i, *cloud_clustered);
        auto label = 0;
        for (const auto &cluster : clusters)
        {
            // Generate a unique color for each BBOX using a HSV color map
            double hue = label * 360.0 / clustersize;
            double saturation = 1.0;
            double value = 1.0;

            // Convert HSV to RGB
            double c = value * saturation;
            double x = c * (1 - std::abs(std::fmod(hue / 60.0, 2) - 1));
            double m = value - c;

            double r, g, b;
            if (hue < 60)
            {
                r = c, g = x, b = 0;
            }
            else if (hue < 120)
            {
                r = x, g = c, b = 0;
            }
            else if (hue < 180)
            {
                r = 0, g = c, b = x;
            }
            else if (hue < 240)
            {
                r = 0, g = x, b = c;
            }
            else if (hue < 300)
            {
                r = x, g = 0, b = c;
            }
            else
            {
                r = c, g = 0, b = x;
            }

            uint8_t R = (r + m) * 255;
            uint8_t G = (g + m) * 255;
            uint8_t B = (b + m) * 255;

            // initialize min and max values with extreme values, so they will be overwritten
            float_t min_val[3] = {1000, 1000, 1000}, max_val[3] = {-1000, -1000, -1000};
            // for all initialized values: [0] = x, [1] = y, [2] = z

            // Assigning the color to the cluster & creating the bbox
            for (auto index : cluster.indices)
            {
                auto &point = cloud_clustered->at(index);
                point.r = R;
                point.g = G;
                point.b = B;

                if (point.x < min_val[0])
                {
                    min_val[0] = point.x;
                }
                if (point.y < min_val[1])
                {
                    min_val[1] = point.y;
                }
                if (point.z < min_val[2])
                {
                    min_val[2] = point.z;
                }
                if (point.x > max_val[0])
                {
                    max_val[0] = point.x;
                }
                if (point.y > max_val[1])
                {
                    max_val[1] = point.y;
                }
                if (point.z > max_val[2])
                {
                    max_val[2] = point.z;
                }
            }
            BboxViz = BboxViz_creation(min_val, max_val, R, G, B, label);
            bbox = bbox_creation(min_val, max_val, label);
            bbox_arr.push_back(bbox);
            BboxViz_arr.markers.push_back(BboxViz);
            label++;
        }

        //  Set the header for GroundTruthArray
        array_msg.header.stamp = this->now();
        array_msg.header.frame_id = "lidar_frame"; // Set appropriate frame_id here
        array_msg.labels = bbox_arr;

        PublisherBbox_->publish(array_msg);

        // Set the header for MarkerArray
        for (auto &marker : BboxViz_arr.markers) {
            marker.header.stamp = this->now();
            marker.header.frame_id = "lidar_frame"; // Ensure this matches your coordinate frame
        }
        PublisherBboxViz_->publish(BboxViz_arr);

        bbox_arr.clear();
        BboxViz_arr.markers.clear();
        
        return cloud_clustered;
    }

    // Marking filename as unused
    float_t calc_height(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &filename)
    {
        (void)filename;  // Explicitly mark the parameter as unused

        std::vector<float> z_values;
        float_t height_cut;
        z_values.reserve(cloud->points.size());

        for (const auto &point : cloud->points)
        {
            z_values.push_back(round(10*point.z)/10);
        }
        height_cut = findMostFrequent(z_values) + 0.5; //cut rails
        return height_cut;
    }

    float_t findMostFrequent(const std::vector<float_t>& arr) {
        std::unordered_map<float_t, int> countMap;
        // Count each element's occurrences
        for (const float_t& elem : arr) {
            ++countMap[elem];
        }

        // Find the element with the maximum occurrences
        auto mostFrequent = std::max_element(countMap.begin(), countMap.end(),
            [](const std::pair<float_t, int>& a, const std::pair<float_t, int>& b) {
                return a.second < b.second;
            });

        // Return the most frequent element
        return mostFrequent->first;
    }

    visualization_msgs::msg::Marker BboxViz_creation(float_t min_val[3], float_t max_val[3], uint8_t R, uint8_t G, uint8_t B, int label)
    {
        visualization_msgs::msg::Marker boundingbox;
        float_t bbox_dim[3] = {0, 0, 0}, center_coord[3] = {0, 0, 0};
        for (int i = 0; i < 3; i++)
        {
            bbox_dim[i] = max_val[i] - min_val[i];
            center_coord[i] = min_val[i] + bbox_dim[i] / 2;
        }
        boundingbox.ns = "boxes";
        boundingbox.id = label;
        boundingbox.type = visualization_msgs::msg::Marker::CUBE;
        boundingbox.action = visualization_msgs::msg::Marker::ADD;
        boundingbox.pose.position.x = center_coord[0];
        boundingbox.pose.position.y = center_coord[1];
        boundingbox.pose.position.z = center_coord[2];
        boundingbox.scale.x = bbox_dim[0];
        boundingbox.scale.y = bbox_dim[1];
        boundingbox.scale.z = bbox_dim[2];
        boundingbox.color.a = 1.0; // Transparency
        boundingbox.color.r = R;
        boundingbox.color.g = G;
        boundingbox.color.b = B;
        return boundingbox;
    }

    lidar_msgs::msg::GroundTruth bbox_creation(float_t min_val[3], float_t max_val[3], int label)
    {
        lidar_msgs::msg::GroundTruth bbox;
        float_t bbox_dim[3] = {0, 0, 0}, center_coord[3] = {0, 0, 0};

        for (int i = 0; i < 3; i++)
        {
            bbox_dim[i] = max_val[i] - min_val[i];
            center_coord[i] = min_val[i] + bbox_dim[i] / 2;
        }
        bbox.id = label;
        bbox.centerx = center_coord[0];
        bbox.centery = center_coord[1];
        bbox.centerz = center_coord[2];
        bbox.length_x = bbox_dim[0];
        bbox.width_y = bbox_dim[1];
        bbox.height_z = bbox_dim[2];
        bbox.yaw = 0;
        bbox.tag = "";
        return bbox;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<lidar_msgs::msg::GroundTruthArray>::SharedPtr PublisherBbox_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr PublisherBboxViz_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFilter>());
    rclcpp::shutdown();
    return 0;
}

// #include <memory>

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include "std_msgs/msg/string.hpp"
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/conversions.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/point_cloud.h>
// #include <pcl/filters/passthrough.h>
// #include <vector>

// #include <lidar_msgs/msg/ground_truth.hpp>
// #include "lidar_msgs/msg/ground_truth_array.hpp"
// #include <std_msgs/msg/header.hpp> 
// #include "visualization_msgs/msg/marker.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"

// #include <pcl_conversions/pcl_conversions.h>
// #include "fast_euclidean_clustering.h"
// #include <vector>
// #include <unordered_map>
// #include <algorithm>
// using std::placeholders::_1;

// class LidarFilter : public rclcpp::Node
// {
// public:
//     LidarFilter()
//         : Node("lidar_filter")
//     {
//         subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//             "/VoxelFilterPub", 1, std::bind(&LidarFilter::topic_callback, this, _1));
//         publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("LidarClusterdPub", 1);
//         PublisherBbox_ = this->create_publisher<lidar_msgs::msg::GroundTruthArray>("BboxClusterPub", 10);
//         PublisherBboxViz_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("LiDAR_Clustering_BoundingBoxes", 10);
//     }

// private:
//     void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//     {
//         unsigned int num_points = msg->width;
//         RCLCPP_INFO(this->get_logger(), "INPUT: The number of points in the pointcloud is %i", num_points);

//         // Declaring the different pointcloud types
//         sensor_msgs::msg::PointCloud2 cloud_out;
//         pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
//         pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
//         pcl::PassThrough<pcl::PointXYZ> pass; // Declare the pass object
//         float_t height_cut;

//         // ROS2 Pointcloud2 to PCL Pointcloud2 Conversion
//         pcl_conversions::toPCL(*msg, *cloud);

//         // PCL Pointcloud2 to PCL Pointcloud Conversion
//         pcl::fromPCLPointCloud2(*cloud, *cloud_xyz);

//         height_cut = calc_height(cloud_xyz, std::to_string(num_points) + "_z_values.csv");

//         // Ground and Sky segmentation
//         pass.setInputCloud(cloud_xyz); // Use the pass object
//         pass.setFilterFieldName("z");
//         pass.setFilterLimits(height_cut, 10.0); // might need to be an dynamic parameter -1.25
//         pass.filter(*cloud_xyz);

//         // Clustering Algorithm
//         cloud_filtered_xyzrgb = clustering(cloud_xyz, 0.3, cloud_xyz->size() / 1000, cloud_xyz->size() / 5, 0.5); // 1000,

//         // PCL Pointcloud to PCL Pointcloud2 Conversion
//         pcl::toPCLPointCloud2(*cloud_filtered_xyzrgb, *cloud_filtered);

//         // PCL Pointcloud2 to ROS2 Pointcloud2 Conversion
//         pcl_conversions::fromPCL(*cloud_filtered, cloud_out);

//         unsigned int num_points_out = cloud_out.width;
//         RCLCPP_INFO(this->get_logger(), "OUTPUT: The number of points in the pointcloud is %i", num_points_out);

//         cloud_out.header.frame_id = msg->header.frame_id;
//         cloud_out.header.stamp = msg->header.stamp;

//         publisher_->publish(cloud_out);
//     }

//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float tolerance, int min_cluster_size, int max_cluster_size, float quality)
//     {
//         // Declaring all necessary variables
//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_with_i(new pcl::PointCloud<pcl::PointXYZI>);
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZRGB>);
//         FastEuclideanClustering<pcl::PointXYZI> fast_euclidean_clustering;
//         pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
//         lidar_msgs::msg::GroundTruth bbox;
//         std::vector<lidar_msgs::msg::GroundTruth> bbox_arr;
//         lidar_msgs::msg::GroundTruthArray array_msg;
//         visualization_msgs::msg::Marker BboxViz;
//         visualization_msgs::msg::MarkerArray BboxViz_arr;
//         int clustersize = 0;
//         std::vector<pcl::PointIndices> clusters;

//         // adding Intensity to the pointcloud (aka zeros)
//         pcl::copyPointCloud(*cloud, *cloud_with_i);

//         // Clustering Parameters
//         fast_euclidean_clustering.setInputCloud(cloud_with_i);
//         fast_euclidean_clustering.setSearchMethod(tree);
//         fast_euclidean_clustering.setClusterTolerance(tolerance);
//         fast_euclidean_clustering.setMinClusterSize(min_cluster_size);
//         fast_euclidean_clustering.setMaxClusterSize(max_cluster_size);
//         fast_euclidean_clustering.setQuality(quality);
//         fast_euclidean_clustering.segment(clusters);
//         clustersize = clusters.size();
//         RCLCPP_INFO(this->get_logger(), "Min_Clustersize: %d", min_cluster_size);
//         RCLCPP_INFO(this->get_logger(), "Max_Clustersize: %d", max_cluster_size);
//         RCLCPP_INFO(this->get_logger(), "Clustersize: %d", clustersize);

//         // Assigning Colors to the clusters, so that they can be visualized
//         pcl::copyPointCloud(*cloud_with_i, *cloud_clustered);
//         auto label = 0;
//         for (const auto &cluster : clusters)
//         {
//             // Generate a unique color for each BBOX using a HSV color map
//             double hue = label * 360.0 / clustersize;
//             double saturation = 1.0;
//             double value = 1.0;

//             // Convert HSV to RGB
//             double c = value * saturation;
//             double x = c * (1 - std::abs(std::fmod(hue / 60.0, 2) - 1));
//             double m = value - c;

//             double r, g, b;
//             if (hue < 60)
//             {
//                 r = c, g = x, b = 0;
//             }
//             else if (hue < 120)
//             {
//                 r = x, g = c, b = 0;
//             }
//             else if (hue < 180)
//             {
//                 r = 0, g = c, b = x;
//             }
//             else if (hue < 240)
//             {
//                 r = 0, g = x, b = c;
//             }
//             else if (hue < 300)
//             {
//                 r = x, g = 0, b = c;
//             }
//             else
//             {
//                 r = c, g = 0, b = x;
//             }

//             uint8_t R = (r + m) * 255;
//             uint8_t G = (g + m) * 255;
//             uint8_t B = (b + m) * 255;

//             // initialize min and max values with extreme values, so they will be overwritten
//             float_t min_val[3] = {1000, 1000, 1000}, max_val[3] = {-1000, -1000, -1000};
//             // for all initilized values:[0] = x, [1] = y, [2] = z

//             // Assigning the color to the cluster & creating the bbox
//             for (auto index : cluster.indices)
//             {
//                 auto &point = cloud_clustered->at(index);
//                 point.r = R;
//                 point.g = G;
//                 point.b = B;

//                 if (point.x < min_val[0])
//                 {
//                     min_val[0] = point.x;
//                 }
//                 if (point.y < min_val[1])
//                 {
//                     min_val[1] = point.y;
//                 }
//                 if (point.z < min_val[2])
//                 {
//                     min_val[2] = point.z;
//                 }
//                 if (point.x > max_val[0])
//                 {
//                     max_val[0] = point.x;
//                 }
//                 if (point.y > max_val[1])
//                 {
//                     max_val[1] = point.y;
//                 }
//                 if (point.z > max_val[2])
//                 {
//                     max_val[2] = point.z;
//                 }
//             }
//             BboxViz = BboxViz_creation(min_val, max_val, R, G, B, label);
//             bbox = bbox_creation(min_val, max_val, label);
//             bbox_arr.push_back(bbox);
//             BboxViz_arr.markers.push_back(BboxViz);
//             label++;
//         }

//         //  Set the header for GroundTruthArray
//         array_msg.header.stamp = this->now();
//         array_msg.header.frame_id = "lidar_frame"; // Set appropriate frame_id here
//         array_msg.labels = bbox_arr;

//         PublisherBbox_->publish(array_msg);

//         // Set the header for MarkerArray
//         for (auto &marker : BboxViz_arr.markers) {
//             marker.header.stamp = this->now();
//             marker.header.frame_id = "lidar_frame"; // Ensure this matches your coordinate frame
//         }
//         PublisherBboxViz_->publish(BboxViz_arr);

//         bbox_arr.clear();
//         BboxViz_arr.markers.clear();
        
//         return cloud_clustered;
//     }

//     float_t calc_height(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &filename)
//     {
//         std::vector<float> z_values;
//         float_t height_cut;
//         z_values.reserve(cloud->points.size());

//         for (const auto &point : cloud->points)
//         {
//             z_values.push_back(round(10*point.z)/10);
//         }
//         height_cut = findMostFrequent(z_values) + 0.1;
//         return height_cut;
//     }

//     float_t findMostFrequent(const std::vector<float_t>& arr) {
//         std::unordered_map<float_t, int> countMap;
//         // Count each element's occurrences
//         for (const float_t& elem : arr) {
//             ++countMap[elem];
//         }

//         // Find the element with the maximum occurrences
//         auto mostFrequent = std::max_element(countMap.begin(), countMap.end(),
//             [](const std::pair<float_t, int>& a, const std::pair<float_t, int>& b) {
//                 return a.second < b.second;
//             });

//         // Return the most frequent element
//         return mostFrequent->first;
//     }

//     visualization_msgs::msg::Marker BboxViz_creation(float_t min_val[3], float_t max_val[3], uint8_t R, uint8_t G, uint8_t B, int label)
//     {
//         visualization_msgs::msg::Marker boundingbox;
//         float_t bbox_dim[3] = {0, 0, 0}, center_coord[3] = {0, 0, 0};
//         for (int i = 0; i < 3; i++)
//         {
//             bbox_dim[i] = max_val[i] - min_val[i];
//             center_coord[i] = min_val[i] + bbox_dim[i] / 2;
//         }
//         boundingbox.ns = "boxes";
//         boundingbox.id = label;
//         boundingbox.type = visualization_msgs::msg::Marker::CUBE;
//         boundingbox.action = visualization_msgs::msg::Marker::ADD;
//         boundingbox.pose.position.x = center_coord[0];
//         boundingbox.pose.position.y = center_coord[1];
//         boundingbox.pose.position.z = center_coord[2];
//         boundingbox.scale.x = bbox_dim[0];
//         boundingbox.scale.y = bbox_dim[1];
//         boundingbox.scale.z = bbox_dim[2];
//         boundingbox.color.a = 1.0; // Transparency
//         boundingbox.color.r = R;
//         boundingbox.color.g = G;
//         boundingbox.color.b = B;
//         return boundingbox;
//     }

//     lidar_msgs::msg::GroundTruth bbox_creation(float_t min_val[3], float_t max_val[3], int label)
//     {
//         lidar_msgs::msg::GroundTruth bbox;
//         float_t bbox_dim[3] = {0, 0, 0}, center_coord[3] = {0, 0, 0};

//         for (int i = 0; i < 3; i++)
//         {
//             bbox_dim[i] = max_val[i] - min_val[i];
//             center_coord[i] = min_val[i] + bbox_dim[i] / 2;
//         }
//         bbox.id = label;
//         bbox.centerx = center_coord[0];
//         bbox.centery = center_coord[1];
//         bbox.centerz = center_coord[2];
//         bbox.length_x = bbox_dim[0];
//         bbox.width_y = bbox_dim[1];
//         bbox.height_z = bbox_dim[2];
//         bbox.yaw = 0;
//         bbox.tag = "";
//         return bbox;
//     }

//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
//     rclcpp::Publisher<lidar_msgs::msg::GroundTruthArray>::SharedPtr PublisherBbox_;
//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr PublisherBboxViz_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<LidarFilter>());
//     rclcpp::shutdown();
//     return 0;
// }
