#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class StaticTransformBroadcasterNode : public rclcpp::Node {
public:
    StaticTransformBroadcasterNode()
        : Node("static_transform_broadcaster_node") {
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Broadcast static transforms
        broadcastStaticTransforms();
    }

private:
    void broadcastStaticTransforms() {
        geometry_msgs::msg::TransformStamped lidar_transform;
        lidar_transform.header.stamp = this->get_clock()->now();
        lidar_transform.header.frame_id = "first_person_view";
        lidar_transform.child_frame_id = "lidar_frame";
        lidar_transform.transform.translation.x = 6.876;
        lidar_transform.transform.translation.y = 0.0;
        lidar_transform.transform.translation.z = 2.14;
        tf2::Quaternion lidar_q;
        lidar_q.setRPY(0, 0, 0);
        lidar_transform.transform.rotation = tf2::toMsg(lidar_q);
        static_broadcaster_->sendTransform(lidar_transform);

        geometry_msgs::msg::TransformStamped camera_transform;
        camera_transform.header.stamp = this->get_clock()->now();
        camera_transform.header.frame_id = "first_person_view";
        camera_transform.child_frame_id = "camera_frame";
        camera_transform.transform.translation.x = 7.03;
        camera_transform.transform.translation.y = 0.0;
        camera_transform.transform.translation.z = 1.87;
        tf2::Quaternion camera_q;
        camera_q.setRPY(0, 0, 0);
        camera_transform.transform.rotation = tf2::toMsg(camera_q);
        static_broadcaster_->sendTransform(camera_transform);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticTransformBroadcasterNode>());
    rclcpp::shutdown();
    return 0;
}
