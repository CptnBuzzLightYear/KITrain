// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lidar_msgs:msg/GroundTruthArray.idl
// generated code does not contain a copyright notice
#include "lidar_msgs/msg/detail/ground_truth_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `labels`
#include "lidar_msgs/msg/detail/ground_truth__functions.h"

bool
lidar_msgs__msg__GroundTruthArray__init(lidar_msgs__msg__GroundTruthArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    lidar_msgs__msg__GroundTruthArray__fini(msg);
    return false;
  }
  // labels
  if (!lidar_msgs__msg__GroundTruth__Sequence__init(&msg->labels, 0)) {
    lidar_msgs__msg__GroundTruthArray__fini(msg);
    return false;
  }
  return true;
}

void
lidar_msgs__msg__GroundTruthArray__fini(lidar_msgs__msg__GroundTruthArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // labels
  lidar_msgs__msg__GroundTruth__Sequence__fini(&msg->labels);
}

bool
lidar_msgs__msg__GroundTruthArray__are_equal(const lidar_msgs__msg__GroundTruthArray * lhs, const lidar_msgs__msg__GroundTruthArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // labels
  if (!lidar_msgs__msg__GroundTruth__Sequence__are_equal(
      &(lhs->labels), &(rhs->labels)))
  {
    return false;
  }
  return true;
}

bool
lidar_msgs__msg__GroundTruthArray__copy(
  const lidar_msgs__msg__GroundTruthArray * input,
  lidar_msgs__msg__GroundTruthArray * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // labels
  if (!lidar_msgs__msg__GroundTruth__Sequence__copy(
      &(input->labels), &(output->labels)))
  {
    return false;
  }
  return true;
}

lidar_msgs__msg__GroundTruthArray *
lidar_msgs__msg__GroundTruthArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_msgs__msg__GroundTruthArray * msg = (lidar_msgs__msg__GroundTruthArray *)allocator.allocate(sizeof(lidar_msgs__msg__GroundTruthArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lidar_msgs__msg__GroundTruthArray));
  bool success = lidar_msgs__msg__GroundTruthArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lidar_msgs__msg__GroundTruthArray__destroy(lidar_msgs__msg__GroundTruthArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lidar_msgs__msg__GroundTruthArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lidar_msgs__msg__GroundTruthArray__Sequence__init(lidar_msgs__msg__GroundTruthArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_msgs__msg__GroundTruthArray * data = NULL;

  if (size) {
    data = (lidar_msgs__msg__GroundTruthArray *)allocator.zero_allocate(size, sizeof(lidar_msgs__msg__GroundTruthArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lidar_msgs__msg__GroundTruthArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lidar_msgs__msg__GroundTruthArray__fini(&data[i - 1]);
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
lidar_msgs__msg__GroundTruthArray__Sequence__fini(lidar_msgs__msg__GroundTruthArray__Sequence * array)
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
      lidar_msgs__msg__GroundTruthArray__fini(&array->data[i]);
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

lidar_msgs__msg__GroundTruthArray__Sequence *
lidar_msgs__msg__GroundTruthArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_msgs__msg__GroundTruthArray__Sequence * array = (lidar_msgs__msg__GroundTruthArray__Sequence *)allocator.allocate(sizeof(lidar_msgs__msg__GroundTruthArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lidar_msgs__msg__GroundTruthArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lidar_msgs__msg__GroundTruthArray__Sequence__destroy(lidar_msgs__msg__GroundTruthArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lidar_msgs__msg__GroundTruthArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lidar_msgs__msg__GroundTruthArray__Sequence__are_equal(const lidar_msgs__msg__GroundTruthArray__Sequence * lhs, const lidar_msgs__msg__GroundTruthArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lidar_msgs__msg__GroundTruthArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lidar_msgs__msg__GroundTruthArray__Sequence__copy(
  const lidar_msgs__msg__GroundTruthArray__Sequence * input,
  lidar_msgs__msg__GroundTruthArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lidar_msgs__msg__GroundTruthArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    lidar_msgs__msg__GroundTruthArray * data =
      (lidar_msgs__msg__GroundTruthArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lidar_msgs__msg__GroundTruthArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          lidar_msgs__msg__GroundTruthArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!lidar_msgs__msg__GroundTruthArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
