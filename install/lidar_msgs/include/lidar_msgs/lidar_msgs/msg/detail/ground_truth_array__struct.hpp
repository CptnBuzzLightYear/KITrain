// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lidar_msgs:msg/GroundTruthArray.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH_ARRAY__STRUCT_HPP_
#define LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'labels'
#include "lidar_msgs/msg/detail/ground_truth__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lidar_msgs__msg__GroundTruthArray __attribute__((deprecated))
#else
# define DEPRECATED__lidar_msgs__msg__GroundTruthArray __declspec(deprecated)
#endif

namespace lidar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GroundTruthArray_
{
  using Type = GroundTruthArray_<ContainerAllocator>;

  explicit GroundTruthArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit GroundTruthArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _labels_type =
    std::vector<lidar_msgs::msg::GroundTruth_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<lidar_msgs::msg::GroundTruth_<ContainerAllocator>>>;
  _labels_type labels;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__labels(
    const std::vector<lidar_msgs::msg::GroundTruth_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<lidar_msgs::msg::GroundTruth_<ContainerAllocator>>> & _arg)
  {
    this->labels = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lidar_msgs::msg::GroundTruthArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const lidar_msgs::msg::GroundTruthArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lidar_msgs::msg::GroundTruthArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lidar_msgs::msg::GroundTruthArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lidar_msgs::msg::GroundTruthArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lidar_msgs::msg::GroundTruthArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lidar_msgs::msg::GroundTruthArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lidar_msgs::msg::GroundTruthArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lidar_msgs::msg::GroundTruthArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lidar_msgs::msg::GroundTruthArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lidar_msgs__msg__GroundTruthArray
    std::shared_ptr<lidar_msgs::msg::GroundTruthArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lidar_msgs__msg__GroundTruthArray
    std::shared_ptr<lidar_msgs::msg::GroundTruthArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GroundTruthArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->labels != other.labels) {
      return false;
    }
    return true;
  }
  bool operator!=(const GroundTruthArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GroundTruthArray_

// alias to use template instance with default allocator
using GroundTruthArray =
  lidar_msgs::msg::GroundTruthArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lidar_msgs

#endif  // LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH_ARRAY__STRUCT_HPP_
