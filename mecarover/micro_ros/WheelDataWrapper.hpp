#pragma once

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float64_multi_array.h>

#include <cmath>
#include <cstring>
#include <mecarover/robot_params.hpp>
#include <type_traits>

template <typename T>
concept FltNum = std::is_same_v<T, float> || std::is_same_v<T, double>;
template <FltNum T>
struct WheelDataWrapper {
  using MsgType =
      typename std::conditional<std::is_same<T, float>::value,
                                std_msgs__msg__Float32MultiArray,
                                std_msgs__msg__Float64MultiArray>::type;

  WheelDataWrapper() {
    this->msg.layout.data_offset = 0;
    this->dim.label.data = const_cast<char*>(label);
    this->dim.label.size = strlen(label);
    // set up the array dimension
    this->dim.size = this->size;  // number of elements in the array
    this->dim.stride = this->stride;
    this->msg.layout.dim.data = &this->dim;
    this->msg.layout.dim.size = 1;
    this->msg.layout.dim.capacity = 1;

    this->msg.data.data = new T[this->dim.size]{};
    this->msg.data.size = this->dim.size;
    this->msg.data.capacity = this->dim.size;
  }

  WheelDataWrapper(const std::array<T, N_WHEEL>& data) : WheelDataWrapper() {
    std::copy(begin(data), end(data), this->msg.data.data);
  }

  WheelDataWrapper(const WheelDataWrapper&) = delete;
  WheelDataWrapper(WheelDataWrapper&&) = delete;
  WheelDataWrapper& operator=(const WheelDataWrapper&) = delete;
  WheelDataWrapper& operator=(WheelDataWrapper&&) = delete;

  ~WheelDataWrapper() { delete[] this->msg.data.data; }

  static const rosidl_message_type_support_t* get_msg_type_support() {
    if constexpr (std::is_same<T, float>::value)
      return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray);
    else
      return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray);
  }

  T& operator[](size_t i) { return this->msg.data.data[i]; }

  const T& operator[](size_t i) const {
    return const_cast<WheelDataWrapper*>(this)->operator[](i);
  }

  const uint8_t size = N_WHEEL;
  const uint8_t stride = 1;
  const char* label = "Wheel generic data";

  MsgType msg;
  std_msgs__msg__MultiArrayDimension dim;
};
