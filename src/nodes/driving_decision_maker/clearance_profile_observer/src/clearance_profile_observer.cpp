#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ClearanceProfileObserver : public rclcpp::Node
{
public:
    ClearanceProfileObserver()
        : Node("clearance_profile_observer"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "LiDAR_Clustering_BoundingBoxes", 10, std::bind(&ClearanceProfileObserver::bbox_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "BoundingBoxesInFirstPersonView", 10);
    }

private:
    void bbox_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        auto transformed_bboxes = transform_bboxes_to_fpv(msg);

        if (!transformed_bboxes.empty())
        {
            visualization_msgs::msg::MarkerArray bbox_array;
            bbox_array.markers = transformed_bboxes;
            publisher_->publish(bbox_array);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No bounding boxes were transformed. Check transform availability and marker data.");
        }
    }

    std::vector<visualization_msgs::msg::Marker> transform_bboxes_to_fpv(const visualization_msgs::msg::MarkerArray::SharedPtr &bbox_array)
    {
        std::vector<visualization_msgs::msg::Marker> transformed_bboxes;

        for (const auto &bbox : bbox_array->markers)
        {
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_.lookupTransform("first_person_view", bbox.header.frame_id, rclcpp::Time(0));

                geometry_msgs::msg::PoseStamped pose_in;
                pose_in.header.frame_id = bbox.header.frame_id;
                pose_in.pose = bbox.pose;

                geometry_msgs::msg::PoseStamped pose_out;
                tf2::doTransform(pose_in, pose_out, transform);

                visualization_msgs::msg::Marker transformed_bbox = bbox;
                transformed_bbox.pose = pose_out.pose;
                transformed_bbox.header.frame_id = "first_person_view";

                transformed_bboxes.push_back(transformed_bbox);
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            }
        }

        return transformed_bboxes;
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClearanceProfileObserver>());
    rclcpp::shutdown();
    return 0;
}
