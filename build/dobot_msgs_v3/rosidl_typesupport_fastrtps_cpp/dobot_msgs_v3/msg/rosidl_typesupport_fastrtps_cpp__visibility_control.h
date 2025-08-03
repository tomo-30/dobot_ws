// generated from
// rosidl_typesupport_fastrtps_cpp/resource/rosidl_typesupport_fastrtps_cpp__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V3__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP__VISIBILITY_CONTROL_H_
#define DOBOT_MSGS_V3__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP__VISIBILITY_CONTROL_H_

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_dobot_msgs_v3 __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_dobot_msgs_v3 __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_dobot_msgs_v3 __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_dobot_msgs_v3 __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_FASTRTPS_CPP_BUILDING_DLL_dobot_msgs_v3
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dobot_msgs_v3 ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_dobot_msgs_v3
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dobot_msgs_v3 ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_dobot_msgs_v3
  #endif
#else
  #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_dobot_msgs_v3 __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_dobot_msgs_v3
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dobot_msgs_v3 __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dobot_msgs_v3
  #endif
#endif

#if __cplusplus
}
#endif

#endif  // DOBOT_MSGS_V3__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP__VISIBILITY_CONTROL_H_
