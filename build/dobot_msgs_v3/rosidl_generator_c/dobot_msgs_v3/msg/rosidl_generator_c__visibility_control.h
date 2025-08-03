// generated from rosidl_generator_c/resource/rosidl_generator_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V3__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
#define DOBOT_MSGS_V3__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_C_EXPORT_dobot_msgs_v3 __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_C_IMPORT_dobot_msgs_v3 __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_C_EXPORT_dobot_msgs_v3 __declspec(dllexport)
    #define ROSIDL_GENERATOR_C_IMPORT_dobot_msgs_v3 __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_C_BUILDING_DLL_dobot_msgs_v3
    #define ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v3 ROSIDL_GENERATOR_C_EXPORT_dobot_msgs_v3
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v3 ROSIDL_GENERATOR_C_IMPORT_dobot_msgs_v3
  #endif
#else
  #define ROSIDL_GENERATOR_C_EXPORT_dobot_msgs_v3 __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_C_IMPORT_dobot_msgs_v3
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v3 __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v3
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // DOBOT_MSGS_V3__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
