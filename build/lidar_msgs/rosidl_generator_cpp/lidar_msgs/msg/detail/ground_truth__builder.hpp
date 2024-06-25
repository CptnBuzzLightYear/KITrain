// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_msgs:msg/GroundTruth.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH__BUILDER_HPP_
#define LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lidar_msgs/msg/detail/ground_truth__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lidar_msgs
{

namespace msg
{

namespace builder
{

class Init_GroundTruth_tag
{
public:
  explicit Init_GroundTruth_tag(::lidar_msgs::msg::GroundTruth & msg)
  : msg_(msg)
  {}
  ::lidar_msgs::msg::GroundTruth tag(::lidar_msgs::msg::GroundTruth::_tag_type arg)
  {
    msg_.tag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_msgs::msg::GroundTruth msg_;
};

class Init_GroundTruth_yaw
{
public:
  explicit Init_GroundTruth_yaw(::lidar_msgs::msg::GroundTruth & msg)
  : msg_(msg)
  {}
  Init_GroundTruth_tag yaw(::lidar_msgs::msg::GroundTruth::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_GroundTruth_tag(msg_);
  }

private:
  ::lidar_msgs::msg::GroundTruth msg_;
};

class Init_GroundTruth_height_z
{
public:
  explicit Init_GroundTruth_height_z(::lidar_msgs::msg::GroundTruth & msg)
  : msg_(msg)
  {}
  Init_GroundTruth_yaw height_z(::lidar_msgs::msg::GroundTruth::_height_z_type arg)
  {
    msg_.height_z = std::move(arg);
    return Init_GroundTruth_yaw(msg_);
  }

private:
  ::lidar_msgs::msg::GroundTruth msg_;
};

class Init_GroundTruth_width_y
{
public:
  explicit Init_GroundTruth_width_y(::lidar_msgs::msg::GroundTruth & msg)
  : msg_(msg)
  {}
  Init_GroundTruth_height_z width_y(::lidar_msgs::msg::GroundTruth::_width_y_type arg)
  {
    msg_.width_y = std::move(arg);
    return Init_GroundTruth_height_z(msg_);
  }

private:
  ::lidar_msgs::msg::GroundTruth msg_;
};

class Init_GroundTruth_length_x
{
public:
  explicit Init_GroundTruth_length_x(::lidar_msgs::msg::GroundTruth & msg)
  : msg_(msg)
  {}
  Init_GroundTruth_width_y length_x(::lidar_msgs::msg::GroundTruth::_length_x_type arg)
  {
    msg_.length_x = std::move(arg);
    return Init_GroundTruth_width_y(msg_);
  }

private:
  ::lidar_msgs::msg::GroundTruth msg_;
};

class Init_GroundTruth_centerz
{
public:
  explicit Init_GroundTruth_centerz(::lidar_msgs::msg::GroundTruth & msg)
  : msg_(msg)
  {}
  Init_GroundTruth_length_x centerz(::lidar_msgs::msg::GroundTruth::_centerz_type arg)
  {
    msg_.centerz = std::move(arg);
    return Init_GroundTruth_length_x(msg_);
  }

private:
  ::lidar_msgs::msg::GroundTruth msg_;
};

class Init_GroundTruth_centery
{
public:
  explicit Init_GroundTruth_centery(::lidar_msgs::msg::GroundTruth & msg)
  : msg_(msg)
  {}
  Init_GroundTruth_centerz centery(::lidar_msgs::msg::GroundTruth::_centery_type arg)
  {
    msg_.centery = std::move(arg);
    return Init_GroundTruth_centerz(msg_);
  }

private:
  ::lidar_msgs::msg::GroundTruth msg_;
};

class Init_GroundTruth_centerx
{
public:
  explicit Init_GroundTruth_centerx(::lidar_msgs::msg::GroundTruth & msg)
  : msg_(msg)
  {}
  Init_GroundTruth_centery centerx(::lidar_msgs::msg::GroundTruth::_centerx_type arg)
  {
    msg_.centerx = std::move(arg);
    return Init_GroundTruth_centery(msg_);
  }

private:
  ::lidar_msgs::msg::GroundTruth msg_;
};

class Init_GroundTruth_id
{
public:
  Init_GroundTruth_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GroundTruth_centerx id(::lidar_msgs::msg::GroundTruth::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_GroundTruth_centerx(msg_);
  }

private:
  ::lidar_msgs::msg::GroundTruth msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_msgs::msg::GroundTruth>()
{
  return lidar_msgs::msg::builder::Init_GroundTruth_id();
}

}  // namespace lidar_msgs

#endif  // LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH__BUILDER_HPP_
