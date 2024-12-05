#pragma once

#include <tim.h>

inline constexpr uint16_t ARR_VALUE = 65535;

class Encoder {
 public:
  Encoder() = default;

  void init(TIM_HandleTypeDef* htim) {
    this->offset = -(ARR_VALUE / 2);

    __HAL_TIM_SET_COUNTER(htim, ARR_VALUE / 2);
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(htim);

    this->counter = __HAL_TIM_GET_COUNTER(htim);
    this->htim = htim;  // set `htim` to finalize the initialization
  }

  int32_t get_val() {
    this->counter = __HAL_TIM_GET_COUNTER(this->htim);
    return this->counter + this->offset;
  }

  void update_offset() {
    if (!this->htim)
      return;  // need to check because the interrupt callback
               // `HAL_TIM_PeriodElapsedCallback` gets triggered immediately
               // after `HAL_TIM_Base_Start_IT`, which calls this function while
               // the whole thing not fully initialized yet and also the handle
               // is still nullptr

    auto cnt = __HAL_TIM_GET_COUNTER(this->htim);
    if (cnt == this->counter) return;
    if (cnt < this->counter)
      this->offset += ARR_VALUE + 1;
    else
      this->offset -= ARR_VALUE - 1;
  }

 private:
  TIM_HandleTypeDef* htim = nullptr;
  /* 64-bit integer for not over or underflowing easily */
  volatile int64_t offset{};
  int64_t counter{};
};
