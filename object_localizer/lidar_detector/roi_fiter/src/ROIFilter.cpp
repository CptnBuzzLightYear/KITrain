#include "ROIFilter.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <nav_msgs/msg/path.hpp> 

ROIFilter::ROIFilter() : rclcpp::Node("roi_filter"), _tfBuffer{this->get_clock()}, _tfListener{_tfBuffer} 
{
    _traintrackSub = this->create_subscription<nav_msgs::msg::Path>(_traintrackTopic, 10, std::bind(&ROIFilter::_rcvTraintrackCallback, this, std::placeholders::_1));
    _pointCloudSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(_pointCloudTopic, 10, std::bind(&ROIFilter::_rcvPointCloudCallback, this, std::placeholders::_1));
    _pointCloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>(_pointCloudFilteredTopic, 10);
}

void ROIFilter::_rcvTraintrackCallback(nav_msgs::msg::Path::SharedPtr const& traintrackMsg) //Warum hier Wechsel von ConstPtr zu SharedPtr?
{
    _traintrack = *traintrackMsg;
}

void ROIFilter::_rcvPointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr const& pointCloudMsg) //Warum hier Wechsel von ConstPtr zu SharedPtr?
{
    _traintrackFrame = _traintrack.header.frame_id;
    _sensorFrame = pointCloudMsg->header.frame_id;

    if(!_createBinaryTree())
    {
        _pointCloudPub->publish(*pointCloudMsg);
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> srcPointCloud;
    pcl::fromROSMsg(*pointCloudMsg, srcPointCloud);
    pcl::PointCloud<pcl::PointXYZ> outPointCloud;

    for(pcl::PointXYZ& point : srcPointCloud)
    {
        Eigen::Vector2d pointVec(point.x, point.y);

        if(_rootNode->pointOnLine(pointVec, _locomotiveProfile))
        {
            outPointCloud.push_back(point);
        }
    }

    sensor_msgs::msg::PointCloud2 outPointCloudMsg;
    pcl::toROSMsg(outPointCloud, outPointCloudMsg);
    outPointCloudMsg.header.frame_id = pointCloudMsg->header.frame_id;
    outPointCloudMsg.header.stamp = pointCloudMsg->header.stamp;
    

    _pointCloudPub->publish(outPointCloudMsg);
}

bool ROIFilter::_createBinaryTree()
{
    if(_rootNode != nullptr)
        delete _rootNode;
    
    _rootNode = nullptr;

    geometry_msgs::msg::TransformStamped sensorWorldTf;

    try
    {
        sensorWorldTf = _tfBuffer.lookupTransform(_sensorFrame, _traintrackFrame, tf2::TimePointZero);
    }
    catch(tf2::TransformException& ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());

        return false;
    }

    std::vector<Eigen::Vector2d> trackPoints;

    for(geometry_msgs::msg::PoseStamped const& trackPoint : _traintrack.poses)
    {
        geometry_msgs::msg::Point transformedPoint;
        tf2::doTransform(trackPoint.pose.position, transformedPoint, sensorWorldTf);

        Eigen::Vector2d point(transformedPoint.x, transformedPoint.y);

        if(transformedPoint.x < _fov_lower)
            trackPoints.clear();
            
        trackPoints.push_back(Eigen::Vector2d(transformedPoint.x, transformedPoint.y));
        
        if(transformedPoint.x > _fov_upper)
            break;
    }

    if(trackPoints.size() > 1)
    {
        _rootNode = new LineNode(trackPoints, 1, trackPoints.size() - 1);
    }
    else
        return false;

    return true;
}