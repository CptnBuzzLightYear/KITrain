// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lidar_msgs:msg/GroundTruth.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH__STRUCT_HPP_
#define LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__lidar_msgs__msg__GroundTruth __attribute__((deprecated))
#else
# define DEPRECATED__lidar_msgs__msg__GroundTruth __declspec(deprecated)
#endif

namespace lidar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GroundTruth_
{
  using Type = GroundTruth_<ContainerAllocator>;

  explicit GroundTruth_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->centerx = 0.0f;
      this->centery = 0.0f;
      this->centerz = 0.0f;
      this->length_x = 0.0f;
      this->width_y = 0.0f;
      this->height_z = 0.0f;
      this->yaw = 0.0f;
      this->tag = "";
    }
  }

  explicit GroundTruth_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : tag(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->centerx = 0.0f;
      this->centery = 0.0f;
      this->centerz = 0.0f;
      this->length_x = 0.0f;
      this->width_y = 0.0f;
      this->height_z = 0.0f;
      this->yaw = 0.0f;
      this->tag = "";
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _centerx_type =
    float;
  _centerx_type centerx;
  using _centery_type =
    float;
  _centery_type centery;
  using _centerz_type =
    float;
  _centerz_type centerz;
  using _length_x_type =
    float;
  _length_x_type length_x;
  using _width_y_type =
    float;
  _width_y_type width_y;
  using _height_z_type =
    float;
  _height_z_type height_z;
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _tag_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _tag_type tag;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__centerx(
    const float & _arg)
  {
    this->centerx = _arg;
    return *this;
  }
  Type & set__centery(
    const float & _arg)
  {
    this->centery = _arg;
    return *this;
  }
  Type & set__centerz(
    const float & _arg)
  {
    this->centerz = _arg;
    return *this;
  }
  Type & set__length_x(
    const float & _arg)
  {
    this->length_x = _arg;
    return *this;
  }
  Type & set__width_y(
    const float & _arg)
  {
    this->width_y = _arg;
    return *this;
  }
  Type & set__height_z(
    const float & _arg)
  {
    this->height_z = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__tag(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->tag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lidar_msgs::msg::GroundTruth_<ContainerAllocator> *;
  using ConstRawPtr =
    const lidar_msgs::msg::GroundTruth_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lidar_msgs::msg::GroundTruth_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lidar_msgs::msg::GroundTruth_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lidar_msgs::msg::GroundTruth_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lidar_msgs::msg::GroundTruth_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lidar_msgs::msg::GroundTruth_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lidar_msgs::msg::GroundTruth_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lidar_msgs::msg::GroundTruth_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lidar_msgs::msg::GroundTruth_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lidar_msgs__msg__GroundTruth
    std::shared_ptr<lidar_msgs::msg::GroundTruth_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lidar_msgs__msg__GroundTruth
    std::shared_ptr<lidar_msgs::msg::GroundTruth_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GroundTruth_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->centerx != other.centerx) {
      return false;
    }
    if (this->centery != other.centery) {
      return false;
    }
    if (this->centerz != other.centerz) {
      return false;
    }
    if (this->length_x != other.length_x) {
      return false;
    }
    if (this->width_y != other.width_y) {
      return false;
    }
    if (this->height_z != other.height_z) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->tag != other.tag) {
      return false;
    }
    return true;
  }
  bool operator!=(const GroundTruth_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GroundTruth_

// alias to use template instance with default allocator
using GroundTruth =
  lidar_msgs::msg::GroundTruth_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lidar_msgs

#endif  // LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH__STRUCT_HPP_
