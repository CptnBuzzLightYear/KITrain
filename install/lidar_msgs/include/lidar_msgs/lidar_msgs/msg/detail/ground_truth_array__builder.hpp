// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_msgs:msg/GroundTruthArray.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH_ARRAY__BUILDER_HPP_
#define LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lidar_msgs/msg/detail/ground_truth_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lidar_msgs
{

namespace msg
{

namespace builder
{

class Init_GroundTruthArray_labels
{
public:
  explicit Init_GroundTruthArray_labels(::lidar_msgs::msg::GroundTruthArray & msg)
  : msg_(msg)
  {}
  ::lidar_msgs::msg::GroundTruthArray labels(::lidar_msgs::msg::GroundTruthArray::_labels_type arg)
  {
    msg_.labels = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_msgs::msg::GroundTruthArray msg_;
};

class Init_GroundTruthArray_header
{
public:
  Init_GroundTruthArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GroundTruthArray_labels header(::lidar_msgs::msg::GroundTruthArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GroundTruthArray_labels(msg_);
  }

private:
  ::lidar_msgs::msg::GroundTruthArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_msgs::msg::GroundTruthArray>()
{
  return lidar_msgs::msg::builder::Init_GroundTruthArray_header();
}

}  // namespace lidar_msgs

#endif  // LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH_ARRAY__BUILDER_HPP_
