#include <memory>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <utility>
#include <numeric>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <lidar_msgs/msg/ground_truth.hpp>
#include <lidar_msgs/msg/ground_truth_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "fast_euclidean_clustering.h"

using std::placeholders::_1;

class LidarFilter : public rclcpp::Node
{
public:
    LidarFilter()
        : Node("lidar_filter")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/foursightm/points", 2, std::bind(&LidarFilter::topic_callback, this, _1)); //  /VoxelFilterPub
        sum_z_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/ground_height_arr", 10, std::bind(&LidarFilter::sum_z_callback, this, _1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("LidarClusterdPub", 2);
        PublisherBbox_ = this->create_publisher<lidar_msgs::msg::GroundTruthArray>("BboxClusterPub", 10);
        PublisherBboxViz_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("LiDAR_Clustering_BoundingBoxes", 10);
        Publisher_sum_z_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("ground_height_arr", 10);
        latest_ground_heights_ = std::vector<float_t>(20, 0.0);
    }
    
private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        unsigned int num_points = msg->width;
        // RCLCPP_INFO(this->get_logger(), "INPUT: The number of points in the pointcloud is %i", num_points);

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

        // Summing up points in the cardinal directions
        height_cut = calc_height(cloud_xyz, std::to_string(num_points) + "_z_values.csv", std::to_string(num_points) + "_y_values.csv", std::to_string(num_points) + "_x_values.csv");
        // correction for messed up z value
        if (height_cut == -99.0)
        {
            //RCLCPP_INFO(this->get_logger(), "old Z-Value: %f", height_cut);
            height_cut = std::accumulate(latest_ground_heights_.begin(), latest_ground_heights_.end(), 0.0) / latest_ground_heights_.size();
            //RCLCPP_INFO(this->get_logger(), "New Z-Value: %f", height_cut);
        }

        // Updating the ground height values
        latest_ground_heights_.push_back(height_cut);

        // Keep only the last 100 values so the array does not explode
        if (latest_ground_heights_.size() > 100)
        {
            latest_ground_heights_.erase(latest_ground_heights_.begin());
        }
        try
        {
            // Groundsegmentation value is the average of the last 10 values
            height_cut = std::accumulate(latest_ground_heights_.end() -10 , latest_ground_heights_.end(), 0.0) / 10;
            // fails if less then 10 elements are recorded
        }
        catch(const std::exception& e)
        {
           
        }

       
        height_cut = -0.9; // hast to be deteted later on

        // Ground and Sky segmentation
        pass.setInputCloud(cloud_xyz); // Use the pass object
        pass.setFilterFieldName("z");
        pass.setFilterLimits(height_cut, 5.0);
        pass.filter(*cloud_xyz);

        // Clustering Algorithm
        cloud_filtered_xyzrgb = clustering(cloud_xyz, 0.2, cloud_xyz->size() / 150,  cloud_xyz->size() / 2, 1); // 1000,
        RCLCPP_INFO(this->get_logger(), "OUTPUT: %i", cloud_xyz->size() / 150);
        // PCL Pointcloud to PCL Pointcloud2 Conversion
        pcl::toPCLPointCloud2(*cloud_filtered_xyzrgb, *cloud_filtered);

        // PCL Pointcloud2 to ROS2 Pointcloud2 Conversion
        pcl_conversions::fromPCL(*cloud_filtered, cloud_out);

        unsigned int num_points_out = cloud_out.width;
        // RCLCPP_INFO(this->get_logger(), "OUTPUT: The number of points in the pointcloud is %i", num_points_out);

        cloud_out.header.frame_id = msg->header.frame_id;
        cloud_out.header.stamp = msg->header.stamp;

        publisher_->publish(cloud_out);

        // Create and publish the ground height array
        std_msgs::msg::Float32MultiArray ground_height_array;
        ground_height_array.data = latest_ground_heights_;
        Publisher_sum_z_->publish(ground_height_array);
    }

    void sum_z_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Store the received ground height values
        latest_ground_heights_ = msg->data;
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
        // RCLCPP_INFO(this->get_logger(), "Min_Clustersize: %d", min_cluster_size);
        // RCLCPP_INFO(this->get_logger(), "Max_Clustersize: %d", max_cluster_size);
        // RCLCPP_INFO(this->get_logger(), "Clustersize: %d", clustersize);

        // Assigning Colors to the clusters, so that they can be visualized
        pcl::copyPointCloud(*cloud_with_i, *cloud_clustered);

        // Extract the points corresponding to the detected clusters
        pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        float_t dist_x = 0.0;
        std::string savestring;
        // only needed or testing purpusose, should be commented of for normal use
        for (const auto& cluster : clusters)
        {
            clustered_cloud->points.clear();
            for (const auto& index : cluster.indices)
            {
                // only save the points if tey are direcly in font of the camera aka y is between -1 and 1
                if (cloud_with_i->points[index].y < 1 && cloud_with_i->points[index].y > -1)
                {
                    pcl::PointXYZ point;
                    point.x = cloud_with_i->points[index].x;
                    point.y = cloud_with_i->points[index].y;
                    point.z = cloud_with_i->points[index].z;
                    clustered_cloud->points.push_back(point);
                    dist_x = cloud_with_i->points[index].x;
                }
            }
            // Set the width and height of the point cloud
            clustered_cloud->width = clustered_cloud->points.size();
            if (clustered_cloud->points.size() == 0)
            {
                continue;
            }
            else
            {
                // Adjust the x values of the points in clustered_cloud so the can be viewed easier
            for (auto& point : clustered_cloud->points)
            {
                point.x -= dist_x;
            }
                clustered_cloud->height = 1;
                clustered_cloud->is_dense = true;
                savestring = "/home/setter/Desktop/Clouds/clustered_cloud_" + std::to_string(dist_x) + "_" + std::to_string(clustered_cloud->points.size()) + ".pcd";
                saveDetectedObject(clustered_cloud, savestring);
            }
            
        }


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
        for (auto &marker : BboxViz_arr.markers)
        {
            marker.header.stamp = this->now();
            marker.header.frame_id = "lidar_frame"; // Ensure this matches your coordinate frame
        }
        PublisherBboxViz_->publish(BboxViz_arr);

        bbox_arr.clear();
        BboxViz_arr.markers.clear();

        return cloud_clustered;
    }

    float_t calc_height(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &filename_z, const std::string &filename_y, const std::string &filename_x)
    {
        (void)filename_z; // Explicitly mark the parameter as unused
        (void)filename_y;
        (void)filename_x;

        std::vector<float> z_values, y_values, x_values;

        z_values.reserve(cloud->points.size());
        y_values.reserve(cloud->points.size());
        x_values.reserve(cloud->points.size());
        for (const auto &point : cloud->points)
        {
            z_values.push_back(round(10 * point.z) / 10);
            y_values.push_back(round(10 * point.y) / 10);
            x_values.push_back(round(10 * point.x) / 10);
        }

        // Use structured bindings to unpack the pair returned by findMostFrequent
        auto [height_cut, cnt_z] = findMostFrequent(z_values);
        height_cut += 0.6; // cut rails

        uint8_t cnt_y = findMostFrequent(y_values).second;
        auto [dist_obj, cnt_x] = findMostFrequent(x_values);

        // looking if there is a value that is more frequent than the z value, if thats the case we re probably in front of a wagon and that messes up the z value
        if (cnt_z < cnt_x ||  dist_obj < 10|| cnt_z < cnt_y)
        {
            //RCLCPP_INFO(this->get_logger(), "Z-Values outnumbered: z:%i y:%i x:%i", cnt_z, cnt_y, cnt_x);
            height_cut = -99.0;
            return height_cut;
        }
        else
        {
            return height_cut;
        }
    }

    std::pair<float_t, uint8_t> findMostFrequent(const std::vector<float_t> &arr)
    {
        std::unordered_map<float_t, uint8_t> countMap;
        // Count each element's occurrences
        for (const float_t &elem : arr)
        {
            ++countMap[elem];
        }

        // Find the element with the maximum occurrences
        auto mostFrequent = std::max_element(countMap.begin(), countMap.end(),
                                             [](const std::pair<float_t, uint8_t> &a, const std::pair<float_t, uint8_t> &b)
                                             {
                                                 return a.second < b.second;
                                             });

        // Return the most frequent element and its count
        return *mostFrequent;
    }

    void saveDetectedObject(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& filename) {
        if (pcl::io::savePCDFileASCII(filename, *cloud) == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to save point cloud to file: %s", filename.c_str());
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Saved point cloud to file: %s", filename.c_str());
        }
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
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sum_z_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<lidar_msgs::msg::GroundTruthArray>::SharedPtr PublisherBbox_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr PublisherBboxViz_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr Publisher_sum_z_;

    std::vector<float_t> latest_ground_heights_; // Member variable to store the latest ground heights
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFilter>());
    rclcpp::shutdown();
    return 0;
}