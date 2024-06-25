// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from lidar_msgs:msg/GroundTruth.idl
// generated code does not contain a copyright notice
#include "lidar_msgs/msg/detail/ground_truth__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "lidar_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "lidar_msgs/msg/detail/ground_truth__struct.h"
#include "lidar_msgs/msg/detail/ground_truth__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // tag
#include "rosidl_runtime_c/string_functions.h"  // tag

// forward declare type support functions


using _GroundTruth__ros_msg_type = lidar_msgs__msg__GroundTruth;

static bool _GroundTruth__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GroundTruth__ros_msg_type * ros_message = static_cast<const _GroundTruth__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: centerx
  {
    cdr << ros_message->centerx;
  }

  // Field name: centery
  {
    cdr << ros_message->centery;
  }

  // Field name: centerz
  {
    cdr << ros_message->centerz;
  }

  // Field name: length_x
  {
    cdr << ros_message->length_x;
  }

  // Field name: width_y
  {
    cdr << ros_message->width_y;
  }

  // Field name: height_z
  {
    cdr << ros_message->height_z;
  }

  // Field name: yaw
  {
    cdr << ros_message->yaw;
  }

  // Field name: tag
  {
    const rosidl_runtime_c__String * str = &ros_message->tag;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _GroundTruth__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GroundTruth__ros_msg_type * ros_message = static_cast<_GroundTruth__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: centerx
  {
    cdr >> ros_message->centerx;
  }

  // Field name: centery
  {
    cdr >> ros_message->centery;
  }

  // Field name: centerz
  {
    cdr >> ros_message->centerz;
  }

  // Field name: length_x
  {
    cdr >> ros_message->length_x;
  }

  // Field name: width_y
  {
    cdr >> ros_message->width_y;
  }

  // Field name: height_z
  {
    cdr >> ros_message->height_z;
  }

  // Field name: yaw
  {
    cdr >> ros_message->yaw;
  }

  // Field name: tag
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->tag.data) {
      rosidl_runtime_c__String__init(&ros_message->tag);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->tag,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'tag'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lidar_msgs
size_t get_serialized_size_lidar_msgs__msg__GroundTruth(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GroundTruth__ros_msg_type * ros_message = static_cast<const _GroundTruth__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name centerx
  {
    size_t item_size = sizeof(ros_message->centerx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name centery
  {
    size_t item_size = sizeof(ros_message->centery);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name centerz
  {
    size_t item_size = sizeof(ros_message->centerz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name length_x
  {
    size_t item_size = sizeof(ros_message->length_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name width_y
  {
    size_t item_size = sizeof(ros_message->width_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name height_z
  {
    size_t item_size = sizeof(ros_message->height_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name yaw
  {
    size_t item_size = sizeof(ros_message->yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tag
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->tag.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _GroundTruth__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_lidar_msgs__msg__GroundTruth(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lidar_msgs
size_t max_serialized_size_lidar_msgs__msg__GroundTruth(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: centerx
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: centery
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: centerz
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: length_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: width_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: height_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: yaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: tag
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = lidar_msgs__msg__GroundTruth;
    is_plain =
      (
      offsetof(DataType, tag) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GroundTruth__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_lidar_msgs__msg__GroundTruth(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GroundTruth = {
  "lidar_msgs::msg",
  "GroundTruth",
  _GroundTruth__cdr_serialize,
  _GroundTruth__cdr_deserialize,
  _GroundTruth__get_serialized_size,
  _GroundTruth__max_serialized_size
};

static rosidl_message_type_support_t _GroundTruth__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GroundTruth,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, lidar_msgs, msg, GroundTruth)() {
  return &_GroundTruth__type_support;
}

#if defined(__cplusplus)
}
#endif
