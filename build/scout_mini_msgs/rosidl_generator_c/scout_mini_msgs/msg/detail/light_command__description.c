// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from scout_mini_msgs:msg/LightCommand.idl
// generated code does not contain a copyright notice

#include "scout_mini_msgs/msg/detail/light_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_scout_mini_msgs
const rosidl_type_hash_t *
scout_mini_msgs__msg__LightCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd5, 0x6d, 0x8f, 0x98, 0xf6, 0x66, 0xa9, 0x72,
      0x76, 0x85, 0x60, 0xdc, 0xb3, 0x3b, 0x86, 0xa6,
      0xda, 0xde, 0x9f, 0xe8, 0x50, 0xba, 0x8a, 0x5e,
      0x09, 0x63, 0x3c, 0xfb, 0x2b, 0x51, 0x77, 0xd3,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char scout_mini_msgs__msg__LightCommand__TYPE_NAME[] = "scout_mini_msgs/msg/LightCommand";

// Define type names, field names, and default values
static char scout_mini_msgs__msg__LightCommand__FIELD_NAME__mode[] = "mode";
static char scout_mini_msgs__msg__LightCommand__FIELD_NAME__brightness[] = "brightness";

static rosidl_runtime_c__type_description__Field scout_mini_msgs__msg__LightCommand__FIELDS[] = {
  {
    {scout_mini_msgs__msg__LightCommand__FIELD_NAME__mode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {scout_mini_msgs__msg__LightCommand__FIELD_NAME__brightness, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
scout_mini_msgs__msg__LightCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {scout_mini_msgs__msg__LightCommand__TYPE_NAME, 32, 32},
      {scout_mini_msgs__msg__LightCommand__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 NC = 0\n"
  "uint8 NO = 1\n"
  "uint8 BL = 2\n"
  "uint8 CUSTOM = 3\n"
  "\n"
  "uint8 mode        # Lighting Mode\n"
  "uint8 brightness  # Only for CUSTOM mode";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
scout_mini_msgs__msg__LightCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {scout_mini_msgs__msg__LightCommand__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 131, 131},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
scout_mini_msgs__msg__LightCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *scout_mini_msgs__msg__LightCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
