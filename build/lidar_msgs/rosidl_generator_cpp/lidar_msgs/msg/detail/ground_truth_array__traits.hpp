// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lidar_msgs:msg/GroundTruthArray.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH_ARRAY__TRAITS_HPP_
#define LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "lidar_msgs/msg/detail/ground_truth_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'labels'
#include "lidar_msgs/msg/detail/ground_truth__traits.hpp"

namespace lidar_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GroundTruthArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: labels
  {
    if (msg.labels.size() == 0) {
      out << "labels: []";
    } else {
      out << "labels: [";
      size_t pending_items = msg.labels.size();
      for (auto item : msg.labels) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GroundTruthArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: labels
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.labels.size() == 0) {
      out << "labels: []\n";
    } else {
      out << "labels:\n";
      for (auto item : msg.labels) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GroundTruthArray & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace lidar_msgs

namespace rosidl_generator_traits
{

[[deprecated("use lidar_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const lidar_msgs::msg::GroundTruthArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  lidar_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use lidar_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const lidar_msgs::msg::GroundTruthArray & msg)
{
  return lidar_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<lidar_msgs::msg::GroundTruthArray>()
{
  return "lidar_msgs::msg::GroundTruthArray";
}

template<>
inline const char * name<lidar_msgs::msg::GroundTruthArray>()
{
  return "lidar_msgs/msg/GroundTruthArray";
}

template<>
struct has_fixed_size<lidar_msgs::msg::GroundTruthArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<lidar_msgs::msg::GroundTruthArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<lidar_msgs::msg::GroundTruthArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH_ARRAY__TRAITS_HPP_
