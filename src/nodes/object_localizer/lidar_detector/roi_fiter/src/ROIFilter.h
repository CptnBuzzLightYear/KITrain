#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

#include "LineNode.h"


class ROIFilter : public rclcpp::Node
{
    public:

            ROIFilter() : Node("roifilter"), _tfBuffer(get_clock()), _tfListener(_tfBuffer)
            {
                _traintrackSub = this->create_subscription<nav_msgs::msg::Path>(_traintrackTopic, 10, std::bind(&ROIFilter::_rcvTraintrackCallback, this, std::placeholders::_1));
                _pointCloudSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(_pointCloudTopic, 10, std::bind(&ROIFilter::_rcvPointCloudCallback, this, std::placeholders::_1));
                _pointCloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>(_pointCloudFilteredTopic, 10);
            }

            void run() 
            {
                rclcpp::spin(shared_from_this());
            };

        private:
            void _rcvTraintrackCallback(nav_msgs::msg::Path::SharedPtr const& traintrackMsg); //Warum hier wechsel von ConstPtr zu shared_ptr?
            void _rcvPointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr const& pointCloudMsg); //Warum hier wechsel von ConstPtr zu shared_ptr?
        bool _createBinaryTree();

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _traintrackSub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _pointCloudSub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointCloudPub;

        tf2_ros::Buffer _tfBuffer;
        tf2_ros::TransformListener _tfListener;

        std::string _sensorFrame = "foursightm";
        std::string _traintrackFrame = "world";

        std::string const _traintrackTopic = "/rt40/traintrack"; //paths are subject to change
        std::string const _pointCloudTopic = "/rt40/floor_segmented";
        std::string const _pointCloudFilteredTopic = "/rt40/roi_filtered";

        int const _fov_lower = 0;
        int const _fov_upper = 110;

        double const _locomotiveProfile = 1.6;
 
        LineNode* _rootNode = nullptr;
        nav_msgs::msg::Path _traintrack;
};