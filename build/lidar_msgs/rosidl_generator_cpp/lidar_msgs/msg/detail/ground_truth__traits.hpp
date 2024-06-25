// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lidar_msgs:msg/GroundTruth.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH__TRAITS_HPP_
#define LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "lidar_msgs/msg/detail/ground_truth__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace lidar_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GroundTruth & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: centerx
  {
    out << "centerx: ";
    rosidl_generator_traits::value_to_yaml(msg.centerx, out);
    out << ", ";
  }

  // member: centery
  {
    out << "centery: ";
    rosidl_generator_traits::value_to_yaml(msg.centery, out);
    out << ", ";
  }

  // member: centerz
  {
    out << "centerz: ";
    rosidl_generator_traits::value_to_yaml(msg.centerz, out);
    out << ", ";
  }

  // member: length_x
  {
    out << "length_x: ";
    rosidl_generator_traits::value_to_yaml(msg.length_x, out);
    out << ", ";
  }

  // member: width_y
  {
    out << "width_y: ";
    rosidl_generator_traits::value_to_yaml(msg.width_y, out);
    out << ", ";
  }

  // member: height_z
  {
    out << "height_z: ";
    rosidl_generator_traits::value_to_yaml(msg.height_z, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: tag
  {
    out << "tag: ";
    rosidl_generator_traits::value_to_yaml(msg.tag, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GroundTruth & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: centerx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "centerx: ";
    rosidl_generator_traits::value_to_yaml(msg.centerx, out);
    out << "\n";
  }

  // member: centery
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "centery: ";
    rosidl_generator_traits::value_to_yaml(msg.centery, out);
    out << "\n";
  }

  // member: centerz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "centerz: ";
    rosidl_generator_traits::value_to_yaml(msg.centerz, out);
    out << "\n";
  }

  // member: length_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "length_x: ";
    rosidl_generator_traits::value_to_yaml(msg.length_x, out);
    out << "\n";
  }

  // member: width_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width_y: ";
    rosidl_generator_traits::value_to_yaml(msg.width_y, out);
    out << "\n";
  }

  // member: height_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height_z: ";
    rosidl_generator_traits::value_to_yaml(msg.height_z, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: tag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tag: ";
    rosidl_generator_traits::value_to_yaml(msg.tag, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GroundTruth & msg, bool use_flow_style = false)
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
  const lidar_msgs::msg::GroundTruth & msg,
  std::ostream & out, size_t indentation = 0)
{
  lidar_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use lidar_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const lidar_msgs::msg::GroundTruth & msg)
{
  return lidar_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<lidar_msgs::msg::GroundTruth>()
{
  return "lidar_msgs::msg::GroundTruth";
}

template<>
inline const char * name<lidar_msgs::msg::GroundTruth>()
{
  return "lidar_msgs/msg/GroundTruth";
}

template<>
struct has_fixed_size<lidar_msgs::msg::GroundTruth>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<lidar_msgs::msg::GroundTruth>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<lidar_msgs::msg::GroundTruth>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH__TRAITS_HPP_
