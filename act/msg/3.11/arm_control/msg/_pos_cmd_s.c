// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from arm_control:msg/PosCmd.idl
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
#include "arm_control/msg/detail/pos_cmd__struct.h"
#include "arm_control/msg/detail/pos_cmd__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool arm_control__msg__pos_cmd__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[32];
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
    assert(strncmp("arm_control.msg._pos_cmd.PosCmd", full_classname_dest, 31) == 0);
  }
  arm_control__msg__PosCmd * ros_message = _ros_message;
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
  {  // roll
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // gripper
    PyObject * field = PyObject_GetAttrString(_pymsg, "gripper");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->gripper = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // quater_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "quater_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->quater_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // quater_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "quater_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->quater_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // quater_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "quater_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->quater_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // quater_w
    PyObject * field = PyObject_GetAttrString(_pymsg, "quater_w");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->quater_w = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // chx
    PyObject * field = PyObject_GetAttrString(_pymsg, "chx");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->chx = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // chy
    PyObject * field = PyObject_GetAttrString(_pymsg, "chy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->chy = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // chz
    PyObject * field = PyObject_GetAttrString(_pymsg, "chz");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->chz = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vel_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vel_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vel_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vel_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // height
    PyObject * field = PyObject_GetAttrString(_pymsg, "height");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->height = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // head_pit
    PyObject * field = PyObject_GetAttrString(_pymsg, "head_pit");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->head_pit = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // head_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "head_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->head_yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // temp_float_data
    PyObject * field = PyObject_GetAttrString(_pymsg, "temp_float_data");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 6;
      double * dest = ros_message->temp_float_data;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // temp_int_data
    PyObject * field = PyObject_GetAttrString(_pymsg, "temp_int_data");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_INT32);
      Py_ssize_t size = 6;
      int32_t * dest = ros_message->temp_int_data;
      for (Py_ssize_t i = 0; i < size; ++i) {
        int32_t tmp = *(npy_int32 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(int32_t));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // mode1
    PyObject * field = PyObject_GetAttrString(_pymsg, "mode1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mode1 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // mode2
    PyObject * field = PyObject_GetAttrString(_pymsg, "mode2");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mode2 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // time_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->time_count = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * arm_control__msg__pos_cmd__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PosCmd */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("arm_control.msg._pos_cmd");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PosCmd");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  arm_control__msg__PosCmd * ros_message = (arm_control__msg__PosCmd *)raw_ros_message;
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
  {  // roll
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gripper
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->gripper);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gripper", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // quater_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->quater_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quater_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // quater_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->quater_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quater_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // quater_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->quater_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quater_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // quater_w
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->quater_w);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quater_w", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // chx
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->chx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "chx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // chy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->chy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "chy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // chz
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->chz);
    {
      int rc = PyObject_SetAttrString(_pymessage, "chz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vel_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vel_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // height
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "height", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // head_pit
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->head_pit);
    {
      int rc = PyObject_SetAttrString(_pymessage, "head_pit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // head_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->head_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "head_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // temp_float_data
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "temp_float_data");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->temp_float_data[0]);
    memcpy(dst, src, 6 * sizeof(double));
    Py_DECREF(field);
  }
  {  // temp_int_data
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "temp_int_data");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_INT32);
    assert(sizeof(npy_int32) == sizeof(int32_t));
    npy_int32 * dst = (npy_int32 *)PyArray_GETPTR1(seq_field, 0);
    int32_t * src = &(ros_message->temp_int_data[0]);
    memcpy(dst, src, 6 * sizeof(int32_t));
    Py_DECREF(field);
  }
  {  // mode1
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->mode1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mode1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mode2
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->mode2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mode2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // time_count
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->time_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
