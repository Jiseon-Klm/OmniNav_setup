// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from scout_mini_msgs:msg/DriverState.idl
// generated code does not contain a copyright notice
#include "scout_mini_msgs/msg/detail/driver_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "scout_mini_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "scout_mini_msgs/msg/detail/driver_state__struct.h"
#include "scout_mini_msgs/msg/detail/driver_state__functions.h"
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

#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_scout_mini_msgs
bool cdr_serialize_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_scout_mini_msgs
bool cdr_deserialize_std_msgs__msg__Header(
  eprosima::fastcdr::Cdr & cdr,
  std_msgs__msg__Header * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_scout_mini_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_scout_mini_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_scout_mini_msgs
bool cdr_serialize_key_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_scout_mini_msgs
size_t get_serialized_size_key_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_scout_mini_msgs
size_t max_serialized_size_key_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_scout_mini_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _DriverState__ros_msg_type = scout_mini_msgs__msg__DriverState;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_mini_msgs
bool cdr_serialize_scout_mini_msgs__msg__DriverState(
  const scout_mini_msgs__msg__DriverState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: driver_voltage
  {
    size_t size = 4;
    auto array_ptr = ros_message->driver_voltage;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: driver_temperature
  {
    size_t size = 4;
    auto array_ptr = ros_message->driver_temperature;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: communication_failure
  {
    size_t size = 4;
    auto array_ptr = ros_message->communication_failure;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: low_supply_voltage
  {
    size_t size = 4;
    auto array_ptr = ros_message->low_supply_voltage;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: motor_over_temperature
  {
    size_t size = 4;
    auto array_ptr = ros_message->motor_over_temperature;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: driver_over_current
  {
    size_t size = 4;
    auto array_ptr = ros_message->driver_over_current;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: driver_over_temperature
  {
    size_t size = 4;
    auto array_ptr = ros_message->driver_over_temperature;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_mini_msgs
bool cdr_deserialize_scout_mini_msgs__msg__DriverState(
  eprosima::fastcdr::Cdr & cdr,
  scout_mini_msgs__msg__DriverState * ros_message)
{
  // Field name: header
  {
    cdr_deserialize_std_msgs__msg__Header(cdr, &ros_message->header);
  }

  // Field name: driver_voltage
  {
    size_t size = 4;
    auto array_ptr = ros_message->driver_voltage;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: driver_temperature
  {
    size_t size = 4;
    auto array_ptr = ros_message->driver_temperature;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: communication_failure
  {
    size_t size = 4;
    auto array_ptr = ros_message->communication_failure;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: low_supply_voltage
  {
    size_t size = 4;
    auto array_ptr = ros_message->low_supply_voltage;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: motor_over_temperature
  {
    size_t size = 4;
    auto array_ptr = ros_message->motor_over_temperature;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: driver_over_current
  {
    size_t size = 4;
    auto array_ptr = ros_message->driver_over_current;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: driver_over_temperature
  {
    size_t size = 4;
    auto array_ptr = ros_message->driver_over_temperature;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_mini_msgs
size_t get_serialized_size_scout_mini_msgs__msg__DriverState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DriverState__ros_msg_type * ros_message = static_cast<const _DriverState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: driver_voltage
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->driver_voltage;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: driver_temperature
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->driver_temperature;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: communication_failure
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->communication_failure;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: low_supply_voltage
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->low_supply_voltage;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: motor_over_temperature
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->motor_over_temperature;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: driver_over_current
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->driver_over_current;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: driver_over_temperature
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->driver_over_temperature;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_mini_msgs
size_t max_serialized_size_scout_mini_msgs__msg__DriverState(
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

  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: driver_voltage
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: driver_temperature
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: communication_failure
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: low_supply_voltage
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: motor_over_temperature
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: driver_over_current
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: driver_over_temperature
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = scout_mini_msgs__msg__DriverState;
    is_plain =
      (
      offsetof(DataType, driver_over_temperature) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_mini_msgs
bool cdr_serialize_key_scout_mini_msgs__msg__DriverState(
  const scout_mini_msgs__msg__DriverState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_key_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: driver_voltage
  {
    size_t size = 4;
    auto array_ptr = ros_message->driver_voltage;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: driver_temperature
  {
    size_t size = 4;
    auto array_ptr = ros_message->driver_temperature;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: communication_failure
  {
    size_t size = 4;
    auto array_ptr = ros_message->communication_failure;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: low_supply_voltage
  {
    size_t size = 4;
    auto array_ptr = ros_message->low_supply_voltage;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: motor_over_temperature
  {
    size_t size = 4;
    auto array_ptr = ros_message->motor_over_temperature;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: driver_over_current
  {
    size_t size = 4;
    auto array_ptr = ros_message->driver_over_current;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: driver_over_temperature
  {
    size_t size = 4;
    auto array_ptr = ros_message->driver_over_temperature;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_mini_msgs
size_t get_serialized_size_key_scout_mini_msgs__msg__DriverState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DriverState__ros_msg_type * ros_message = static_cast<const _DriverState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_key_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: driver_voltage
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->driver_voltage;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: driver_temperature
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->driver_temperature;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: communication_failure
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->communication_failure;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: low_supply_voltage
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->low_supply_voltage;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: motor_over_temperature
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->motor_over_temperature;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: driver_over_current
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->driver_over_current;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: driver_over_temperature
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->driver_over_temperature;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_mini_msgs
size_t max_serialized_size_key_scout_mini_msgs__msg__DriverState(
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
  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: driver_voltage
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: driver_temperature
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: communication_failure
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: low_supply_voltage
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: motor_over_temperature
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: driver_over_current
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: driver_over_temperature
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = scout_mini_msgs__msg__DriverState;
    is_plain =
      (
      offsetof(DataType, driver_over_temperature) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _DriverState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const scout_mini_msgs__msg__DriverState * ros_message = static_cast<const scout_mini_msgs__msg__DriverState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_scout_mini_msgs__msg__DriverState(ros_message, cdr);
}

static bool _DriverState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  scout_mini_msgs__msg__DriverState * ros_message = static_cast<scout_mini_msgs__msg__DriverState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_scout_mini_msgs__msg__DriverState(cdr, ros_message);
}

static uint32_t _DriverState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_scout_mini_msgs__msg__DriverState(
      untyped_ros_message, 0));
}

static size_t _DriverState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_scout_mini_msgs__msg__DriverState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_DriverState = {
  "scout_mini_msgs::msg",
  "DriverState",
  _DriverState__cdr_serialize,
  _DriverState__cdr_deserialize,
  _DriverState__get_serialized_size,
  _DriverState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _DriverState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_DriverState,
  get_message_typesupport_handle_function,
  &scout_mini_msgs__msg__DriverState__get_type_hash,
  &scout_mini_msgs__msg__DriverState__get_type_description,
  &scout_mini_msgs__msg__DriverState__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scout_mini_msgs, msg, DriverState)() {
  return &_DriverState__type_support;
}

#if defined(__cplusplus)
}
#endif
