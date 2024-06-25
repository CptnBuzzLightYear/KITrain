// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lidar_msgs:msg/GroundTruth.idl
// generated code does not contain a copyright notice
#include "lidar_msgs/msg/detail/ground_truth__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `tag`
#include "rosidl_runtime_c/string_functions.h"

bool
lidar_msgs__msg__GroundTruth__init(lidar_msgs__msg__GroundTruth * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // centerx
  // centery
  // centerz
  // length_x
  // width_y
  // height_z
  // yaw
  // tag
  if (!rosidl_runtime_c__String__init(&msg->tag)) {
    lidar_msgs__msg__GroundTruth__fini(msg);
    return false;
  }
  return true;
}

void
lidar_msgs__msg__GroundTruth__fini(lidar_msgs__msg__GroundTruth * msg)
{
  if (!msg) {
    return;
  }
  // id
  // centerx
  // centery
  // centerz
  // length_x
  // width_y
  // height_z
  // yaw
  // tag
  rosidl_runtime_c__String__fini(&msg->tag);
}

bool
lidar_msgs__msg__GroundTruth__are_equal(const lidar_msgs__msg__GroundTruth * lhs, const lidar_msgs__msg__GroundTruth * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // centerx
  if (lhs->centerx != rhs->centerx) {
    return false;
  }
  // centery
  if (lhs->centery != rhs->centery) {
    return false;
  }
  // centerz
  if (lhs->centerz != rhs->centerz) {
    return false;
  }
  // length_x
  if (lhs->length_x != rhs->length_x) {
    return false;
  }
  // width_y
  if (lhs->width_y != rhs->width_y) {
    return false;
  }
  // height_z
  if (lhs->height_z != rhs->height_z) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // tag
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->tag), &(rhs->tag)))
  {
    return false;
  }
  return true;
}

bool
lidar_msgs__msg__GroundTruth__copy(
  const lidar_msgs__msg__GroundTruth * input,
  lidar_msgs__msg__GroundTruth * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // centerx
  output->centerx = input->centerx;
  // centery
  output->centery = input->centery;
  // centerz
  output->centerz = input->centerz;
  // length_x
  output->length_x = input->length_x;
  // width_y
  output->width_y = input->width_y;
  // height_z
  output->height_z = input->height_z;
  // yaw
  output->yaw = input->yaw;
  // tag
  if (!rosidl_runtime_c__String__copy(
      &(input->tag), &(output->tag)))
  {
    return false;
  }
  return true;
}

lidar_msgs__msg__GroundTruth *
lidar_msgs__msg__GroundTruth__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_msgs__msg__GroundTruth * msg = (lidar_msgs__msg__GroundTruth *)allocator.allocate(sizeof(lidar_msgs__msg__GroundTruth), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lidar_msgs__msg__GroundTruth));
  bool success = lidar_msgs__msg__GroundTruth__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lidar_msgs__msg__GroundTruth__destroy(lidar_msgs__msg__GroundTruth * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lidar_msgs__msg__GroundTruth__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lidar_msgs__msg__GroundTruth__Sequence__init(lidar_msgs__msg__GroundTruth__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_msgs__msg__GroundTruth * data = NULL;

  if (size) {
    data = (lidar_msgs__msg__GroundTruth *)allocator.zero_allocate(size, sizeof(lidar_msgs__msg__GroundTruth), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lidar_msgs__msg__GroundTruth__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lidar_msgs__msg__GroundTruth__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
lidar_msgs__msg__GroundTruth__Sequence__fini(lidar_msgs__msg__GroundTruth__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      lidar_msgs__msg__GroundTruth__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

lidar_msgs__msg__GroundTruth__Sequence *
lidar_msgs__msg__GroundTruth__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_msgs__msg__GroundTruth__Sequence * array = (lidar_msgs__msg__GroundTruth__Sequence *)allocator.allocate(sizeof(lidar_msgs__msg__GroundTruth__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lidar_msgs__msg__GroundTruth__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lidar_msgs__msg__GroundTruth__Sequence__destroy(lidar_msgs__msg__GroundTruth__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lidar_msgs__msg__GroundTruth__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lidar_msgs__msg__GroundTruth__Sequence__are_equal(const lidar_msgs__msg__GroundTruth__Sequence * lhs, const lidar_msgs__msg__GroundTruth__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lidar_msgs__msg__GroundTruth__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lidar_msgs__msg__GroundTruth__Sequence__copy(
  const lidar_msgs__msg__GroundTruth__Sequence * input,
  lidar_msgs__msg__GroundTruth__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lidar_msgs__msg__GroundTruth);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    lidar_msgs__msg__GroundTruth * data =
      (lidar_msgs__msg__GroundTruth *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lidar_msgs__msg__GroundTruth__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          lidar_msgs__msg__GroundTruth__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!lidar_msgs__msg__GroundTruth__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
