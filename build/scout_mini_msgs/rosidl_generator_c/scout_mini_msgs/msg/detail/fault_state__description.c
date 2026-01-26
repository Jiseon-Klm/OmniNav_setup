// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from scout_mini_msgs:msg/FaultState.idl
// generated code does not contain a copyright notice

#include "scout_mini_msgs/msg/detail/fault_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_scout_mini_msgs
const rosidl_type_hash_t *
scout_mini_msgs__msg__FaultState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5d, 0xb4, 0x5c, 0x1e, 0xb9, 0xc1, 0x14, 0x14,
      0x68, 0xa4, 0x09, 0xc7, 0x0a, 0x10, 0xd5, 0x5d,
      0x68, 0xdd, 0xe3, 0x36, 0xe6, 0x7d, 0x45, 0xf9,
      0x3d, 0xd2, 0xda, 0xae, 0xef, 0xb0, 0x07, 0x70,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char scout_mini_msgs__msg__FaultState__TYPE_NAME[] = "scout_mini_msgs/msg/FaultState";

// Define type names, field names, and default values
static char scout_mini_msgs__msg__FaultState__FIELD_NAME__battery_under_voltage_failure[] = "battery_under_voltage_failure";
static char scout_mini_msgs__msg__FaultState__FIELD_NAME__battery_under_voltage_alarm[] = "battery_under_voltage_alarm";
static char scout_mini_msgs__msg__FaultState__FIELD_NAME__loss_remote_control[] = "loss_remote_control";

static rosidl_runtime_c__type_description__Field scout_mini_msgs__msg__FaultState__FIELDS[] = {
  {
    {scout_mini_msgs__msg__FaultState__FIELD_NAME__battery_under_voltage_failure, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {scout_mini_msgs__msg__FaultState__FIELD_NAME__battery_under_voltage_alarm, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {scout_mini_msgs__msg__FaultState__FIELD_NAME__loss_remote_control, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
scout_mini_msgs__msg__FaultState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {scout_mini_msgs__msg__FaultState__TYPE_NAME, 30, 30},
      {scout_mini_msgs__msg__FaultState__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool battery_under_voltage_failure\n"
  "bool battery_under_voltage_alarm \n"
  "bool loss_remote_control";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
scout_mini_msgs__msg__FaultState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {scout_mini_msgs__msg__FaultState__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 93, 93},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
scout_mini_msgs__msg__FaultState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *scout_mini_msgs__msg__FaultState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
