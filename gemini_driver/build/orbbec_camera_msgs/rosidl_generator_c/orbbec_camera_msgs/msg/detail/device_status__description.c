// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from orbbec_camera_msgs:msg/DeviceStatus.idl
// generated code does not contain a copyright notice

#include "orbbec_camera_msgs/msg/detail/device_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
const rosidl_type_hash_t *
orbbec_camera_msgs__msg__DeviceStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9c, 0x41, 0x9d, 0x67, 0x80, 0xe1, 0xd5, 0xb0,
      0x1e, 0x15, 0xf0, 0x5e, 0x8f, 0x6c, 0x10, 0xbf,
      0x04, 0x14, 0xca, 0xd5, 0x4c, 0x9a, 0x1e, 0x44,
      0x63, 0x1b, 0xd9, 0xec, 0xb4, 0x58, 0xf0, 0x31,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char orbbec_camera_msgs__msg__DeviceStatus__TYPE_NAME[] = "orbbec_camera_msgs/msg/DeviceStatus";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__header[] = "header";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_frame_rate_cur[] = "color_frame_rate_cur";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_frame_rate_avg[] = "color_frame_rate_avg";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_frame_rate_min[] = "color_frame_rate_min";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_frame_rate_max[] = "color_frame_rate_max";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_delay_ms_cur[] = "color_delay_ms_cur";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_delay_ms_avg[] = "color_delay_ms_avg";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_delay_ms_min[] = "color_delay_ms_min";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_delay_ms_max[] = "color_delay_ms_max";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_frame_rate_cur[] = "depth_frame_rate_cur";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_frame_rate_avg[] = "depth_frame_rate_avg";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_frame_rate_min[] = "depth_frame_rate_min";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_frame_rate_max[] = "depth_frame_rate_max";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_delay_ms_cur[] = "depth_delay_ms_cur";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_delay_ms_avg[] = "depth_delay_ms_avg";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_delay_ms_min[] = "depth_delay_ms_min";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_delay_ms_max[] = "depth_delay_ms_max";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__device_online[] = "device_online";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__connection_type[] = "connection_type";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__customer_calibration_ready[] = "customer_calibration_ready";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__calibration_from_factory[] = "calibration_from_factory";
static char orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__calibration_from_launch_param[] = "calibration_from_launch_param";

static rosidl_runtime_c__type_description__Field orbbec_camera_msgs__msg__DeviceStatus__FIELDS[] = {
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_frame_rate_cur, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_frame_rate_avg, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_frame_rate_min, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_frame_rate_max, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_delay_ms_cur, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_delay_ms_avg, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_delay_ms_min, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__color_delay_ms_max, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_frame_rate_cur, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_frame_rate_avg, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_frame_rate_min, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_frame_rate_max, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_delay_ms_cur, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_delay_ms_avg, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_delay_ms_min, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__depth_delay_ms_max, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__device_online, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__connection_type, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__customer_calibration_ready, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__calibration_from_factory, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__msg__DeviceStatus__FIELD_NAME__calibration_from_launch_param, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription orbbec_camera_msgs__msg__DeviceStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
orbbec_camera_msgs__msg__DeviceStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {orbbec_camera_msgs__msg__DeviceStatus__TYPE_NAME, 35, 35},
      {orbbec_camera_msgs__msg__DeviceStatus__FIELDS, 22, 22},
    },
    {orbbec_camera_msgs__msg__DeviceStatus__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "\n"
  "# --- Color stream ---\n"
  "float64 color_frame_rate_cur\n"
  "float64 color_frame_rate_avg\n"
  "float64 color_frame_rate_min\n"
  "float64 color_frame_rate_max\n"
  "\n"
  "float64 color_delay_ms_cur\n"
  "float64 color_delay_ms_avg\n"
  "float64 color_delay_ms_min\n"
  "float64 color_delay_ms_max\n"
  "\n"
  "# --- Depth stream ---\n"
  "float64 depth_frame_rate_cur\n"
  "float64 depth_frame_rate_avg\n"
  "float64 depth_frame_rate_min\n"
  "float64 depth_frame_rate_max\n"
  "\n"
  "float64 depth_delay_ms_cur\n"
  "float64 depth_delay_ms_avg\n"
  "float64 depth_delay_ms_min\n"
  "float64 depth_delay_ms_max\n"
  "\n"
  "# --- Device info ---\n"
  "bool device_online\n"
  "string connection_type   # e.g. \"USB2.0\", \"USB3.0\", \"GigE\"\n"
  "\n"
  "# --- Calibration status ---\n"
  "bool customer_calibration_ready\n"
  "bool calibration_from_factory\n"
  "bool calibration_from_launch_param";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
orbbec_camera_msgs__msg__DeviceStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {orbbec_camera_msgs__msg__DeviceStatus__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 749, 749},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
orbbec_camera_msgs__msg__DeviceStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *orbbec_camera_msgs__msg__DeviceStatus__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
