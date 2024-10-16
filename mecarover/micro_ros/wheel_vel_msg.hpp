#pragma once

#include <mecarover/mrtypes.h>
#include <std_msgs/msg/float64_multi_array.h>

#include <cstring>

struct wheel_vel_msg {
  wheel_vel_msg() {
    std_msgs__msg__Float64MultiArray__init(&this->msg);

    this->msg.layout.data_offset = 0;
    this->dim.label.data = const_cast<char*>(label);
    this->dim.label.size = strlen(label);

    this->dim.size = this->SIZE;  // number of elements in the array
    this->dim.stride = this->STRIDE;

    this->msg.layout.dim.data = &this->dim;
    this->msg.layout.dim.size = 1;
    this->msg.layout.dim.capacity = 1;

    this->msg.data.data = new real_t[this->dim.size]{};
    this->msg.data.size = this->dim.size;
    this->msg.data.capacity = this->dim.size;
  }

  wheel_vel_msg(real_t o0, real_t o1, real_t o2, real_t o3) : wheel_vel_msg() {
    this->msg.data.data[0] = o0;
    this->msg.data.data[1] = o1;
    this->msg.data.data[2] = o2;
    this->msg.data.data[3] = o3;
  }

  ~wheel_vel_msg() { delete[] this->msg.data.data; }

  real_t& operator[](size_t i) { return this->msg.data.data[i]; }

  const real_t& operator[](size_t i) const {
    return const_cast<wheel_vel_msg*>(this)->operator[](i);
  }

  constexpr static uint8_t SIZE = 4;
  constexpr static uint8_t STRIDE = 1;

  std_msgs__msg__Float64MultiArray msg;
  std_msgs__msg__MultiArrayDimension dim;
  const char* label = "mecanum wheel velocity array";
};
