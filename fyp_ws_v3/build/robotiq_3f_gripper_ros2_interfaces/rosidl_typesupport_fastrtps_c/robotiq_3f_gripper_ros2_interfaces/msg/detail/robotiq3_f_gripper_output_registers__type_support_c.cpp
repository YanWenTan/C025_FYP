// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from robotiq_3f_gripper_ros2_interfaces:msg/Robotiq3FGripperOutputRegisters.idl
// generated code does not contain a copyright notice
#include "robotiq_3f_gripper_ros2_interfaces/msg/detail/robotiq3_f_gripper_output_registers__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "robotiq_3f_gripper_ros2_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robotiq_3f_gripper_ros2_interfaces/msg/detail/robotiq3_f_gripper_output_registers__struct.h"
#include "robotiq_3f_gripper_ros2_interfaces/msg/detail/robotiq3_f_gripper_output_registers__functions.h"
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


// forward declare type support functions


using _Robotiq3FGripperOutputRegisters__ros_msg_type = robotiq_3f_gripper_ros2_interfaces__msg__Robotiq3FGripperOutputRegisters;

static bool _Robotiq3FGripperOutputRegisters__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Robotiq3FGripperOutputRegisters__ros_msg_type * ros_message = static_cast<const _Robotiq3FGripperOutputRegisters__ros_msg_type *>(untyped_ros_message);
  // Field name: r_act
  {
    cdr << ros_message->r_act;
  }

  // Field name: r_mod
  {
    cdr << ros_message->r_mod;
  }

  // Field name: r_gto
  {
    cdr << ros_message->r_gto;
  }

  // Field name: r_atr
  {
    cdr << ros_message->r_atr;
  }

  // Field name: r_pra
  {
    cdr << ros_message->r_pra;
  }

  // Field name: r_spa
  {
    cdr << ros_message->r_spa;
  }

  // Field name: r_fra
  {
    cdr << ros_message->r_fra;
  }

  return true;
}

static bool _Robotiq3FGripperOutputRegisters__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Robotiq3FGripperOutputRegisters__ros_msg_type * ros_message = static_cast<_Robotiq3FGripperOutputRegisters__ros_msg_type *>(untyped_ros_message);
  // Field name: r_act
  {
    cdr >> ros_message->r_act;
  }

  // Field name: r_mod
  {
    cdr >> ros_message->r_mod;
  }

  // Field name: r_gto
  {
    cdr >> ros_message->r_gto;
  }

  // Field name: r_atr
  {
    cdr >> ros_message->r_atr;
  }

  // Field name: r_pra
  {
    cdr >> ros_message->r_pra;
  }

  // Field name: r_spa
  {
    cdr >> ros_message->r_spa;
  }

  // Field name: r_fra
  {
    cdr >> ros_message->r_fra;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robotiq_3f_gripper_ros2_interfaces
size_t get_serialized_size_robotiq_3f_gripper_ros2_interfaces__msg__Robotiq3FGripperOutputRegisters(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Robotiq3FGripperOutputRegisters__ros_msg_type * ros_message = static_cast<const _Robotiq3FGripperOutputRegisters__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name r_act
  {
    size_t item_size = sizeof(ros_message->r_act);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name r_mod
  {
    size_t item_size = sizeof(ros_message->r_mod);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name r_gto
  {
    size_t item_size = sizeof(ros_message->r_gto);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name r_atr
  {
    size_t item_size = sizeof(ros_message->r_atr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name r_pra
  {
    size_t item_size = sizeof(ros_message->r_pra);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name r_spa
  {
    size_t item_size = sizeof(ros_message->r_spa);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name r_fra
  {
    size_t item_size = sizeof(ros_message->r_fra);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Robotiq3FGripperOutputRegisters__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_robotiq_3f_gripper_ros2_interfaces__msg__Robotiq3FGripperOutputRegisters(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robotiq_3f_gripper_ros2_interfaces
size_t max_serialized_size_robotiq_3f_gripper_ros2_interfaces__msg__Robotiq3FGripperOutputRegisters(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: r_act
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: r_mod
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: r_gto
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: r_atr
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: r_pra
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: r_spa
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: r_fra
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _Robotiq3FGripperOutputRegisters__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_robotiq_3f_gripper_ros2_interfaces__msg__Robotiq3FGripperOutputRegisters(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Robotiq3FGripperOutputRegisters = {
  "robotiq_3f_gripper_ros2_interfaces::msg",
  "Robotiq3FGripperOutputRegisters",
  _Robotiq3FGripperOutputRegisters__cdr_serialize,
  _Robotiq3FGripperOutputRegisters__cdr_deserialize,
  _Robotiq3FGripperOutputRegisters__get_serialized_size,
  _Robotiq3FGripperOutputRegisters__max_serialized_size
};

static rosidl_message_type_support_t _Robotiq3FGripperOutputRegisters__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Robotiq3FGripperOutputRegisters,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robotiq_3f_gripper_ros2_interfaces, msg, Robotiq3FGripperOutputRegisters)() {
  return &_Robotiq3FGripperOutputRegisters__type_support;
}

#if defined(__cplusplus)
}
#endif
