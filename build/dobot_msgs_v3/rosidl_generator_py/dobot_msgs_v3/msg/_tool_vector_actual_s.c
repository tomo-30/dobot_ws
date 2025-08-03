// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from dobot_msgs_v3:msg/ToolVectorActual.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "dobot_msgs_v3/msg/detail/tool_vector_actual__struct.h"
#include "dobot_msgs_v3/msg/detail/tool_vector_actual__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool dobot_msgs_v3__msg__tool_vector_actual__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[55];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("dobot_msgs_v3.msg._tool_vector_actual.ToolVectorActual", full_classname_dest, 54) == 0);
  }
  dobot_msgs_v3__msg__ToolVectorActual * ros_message = _ros_message;
  {  // x
    PyObject * field = PyObject_GetAttrString(_pymsg, "x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y
    PyObject * field = PyObject_GetAttrString(_pymsg, "y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z
    PyObject * field = PyObject_GetAttrString(_pymsg, "z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rx
    PyObject * field = PyObject_GetAttrString(_pymsg, "rx");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rx = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ry
    PyObject * field = PyObject_GetAttrString(_pymsg, "ry");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ry = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rz
    PyObject * field = PyObject_GetAttrString(_pymsg, "rz");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rz = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * dobot_msgs_v3__msg__tool_vector_actual__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ToolVectorActual */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("dobot_msgs_v3.msg._tool_vector_actual");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ToolVectorActual");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  dobot_msgs_v3__msg__ToolVectorActual * ros_message = (dobot_msgs_v3__msg__ToolVectorActual *)raw_ros_message;
  {  // x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rx
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ry
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ry);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ry", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rz
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rz);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
