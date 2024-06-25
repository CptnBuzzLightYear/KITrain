// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_msgs:msg/GroundTruthArray.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH_ARRAY__STRUCT_H_
#define LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'labels'
#include "lidar_msgs/msg/detail/ground_truth__struct.h"

/// Struct defined in msg/GroundTruthArray in the package lidar_msgs.
typedef struct lidar_msgs__msg__GroundTruthArray
{
  std_msgs__msg__Header header;
  lidar_msgs__msg__GroundTruth__Sequence labels;
} lidar_msgs__msg__GroundTruthArray;

// Struct for a sequence of lidar_msgs__msg__GroundTruthArray.
typedef struct lidar_msgs__msg__GroundTruthArray__Sequence
{
  lidar_msgs__msg__GroundTruthArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_msgs__msg__GroundTruthArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_MSGS__MSG__DETAIL__GROUND_TRUTH_ARRAY__STRUCT_H_
