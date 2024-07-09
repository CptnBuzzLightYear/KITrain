#include <memory>

/*
To Use This Template:
1. Change Class Name to the respective Pointcloud Processing Step
2. Change the topic_callback function to the respective processing function
3. Add the respective processing function
4. Add all necessary includes
5. add dependencies to CMakeLists.txt
6. change file name to the respective processing step
*/

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

#include <lidar_msgs/msg/ground_truth.hpp>
#include "lidar_msgs/msg/ground_truth_array.hpp"
//#include "visualization_msgs/msg/marker.hpp"
//#include "visualization_msgs/msg/marker_array.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include "fast_euclidean_clustering.h"
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
        publisherbbox_ = this->create_publisher<lidar_msgs::msg::GroundTruthArray>("BboxClusterPub", 10);
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

        // ROS2 Pointcloud2 to PCL Pointcloud2 Conversion
        pcl_conversions::toPCL(*msg, *cloud);

        // PCL Pointcloud2 to PCL Pointcloud Conversion
        pcl::fromPCLPointCloud2(*cloud, *cloud_xyz);

        // Ground and Sky segmentation
        pass.setInputCloud(cloud_xyz); // Use the pass object
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-1.26, 3.0); // might ne to be an dynamic parameter
        pass.filter(*cloud_xyz);

        // Clustering Algorithm
        cloud_filtered_xyzrgb = clustering(cloud_xyz, 1, 50, 100000, 0.5); // 1000,

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
            //for all initilized values:[0] = x, [1] = y, [2] = z

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
            label++;
            
            bbox = bbox_creation (min_val, max_val, R, G, B, label);
            bbox_arr.push_back(bbox);
            
            }


//  HIER ÜBERÖEGEN WIE DAS GUT GEHT MIT DEM PUBLISHEN, muss nicht hier sein, kann auch mit der pcd gepublished werden
        array_msg.header.stamp = this->now();
        array_msg.header.frame_id = "frameidLiDARLabel";
        array_msg.labels = bbox_arr;  
        bbox_arr.clear();
        publisherbbox_->publish(array_msg);
        RCLCPP_INFO(this->get_logger(), "BBox published");
        return cloud_clustered;
    }

    lidar_msgs::msg::GroundTruth bbox_creation (float_t min_val[3], float_t max_val[3], uint8_t R, uint8_t G, uint8_t B, int label)
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
        //bbox.color.a = 1;
        //bbox.color.r = R;
        //bbox.color.g = G;
        //bbox.color.b = B;
        return bbox;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr clustering_ec(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float tolerance, int min_cluster_size, int max_cluster_size, float quality)
    {
        // Declaring all necessary variables
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_with_i(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZRGB>);
        FastEuclideanClustering<pcl::PointXYZI> fast_euclidean_clustering;
        int clustersize = 0;
        std::vector<pcl::PointIndices> clusters;

        // adding Intensity to the pointcloud (aka zeros)
        pcl::copyPointCloud(*cloud, *cloud_with_i);

        // Clustering Parameters
        fast_euclidean_clustering.setInputCloud(cloud_with_i);
        fast_euclidean_clustering.setClusterTolerance(tolerance);
        fast_euclidean_clustering.setMinClusterSize(min_cluster_size);
        fast_euclidean_clustering.setMaxClusterSize(max_cluster_size);
        fast_euclidean_clustering.setQuality(quality);
        fast_euclidean_clustering.segment(clusters);
        clustersize = clusters.size();
        RCLCPP_INFO(this->get_logger(), "Clustersize: %d", clustersize);

        // Assigning Colors to the clusters, so that they can be visualized
        pcl::copyPointCloud(*cloud_with_i, *cloud_clustered);
        auto label = 0;
        for (const auto &cluster : clusters)
        {
            for (auto index : cluster.indices)
            {
                auto &point = cloud_with_i->at(index);
                point.intensity = static_cast<float>(label);
            }
            label++;
        }
        return cloud_with_i;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<lidar_msgs::msg::GroundTruthArray>::SharedPtr publisherbbox_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFilter>());
    rclcpp::shutdown();
    return 0;
}