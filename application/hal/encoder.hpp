#pragma once

#include <tim.h>

inline constexpr uint16_t ARR_VALUE = 65535;

class Encoder {
 public:
  Encoder() = default;

  void init(TIM_HandleTypeDef* htim) {
    this->offset = -(ARR_VALUE / 2);
    htim->Instance->CNT = this->counter = ARR_VALUE / 2;

    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(htim);

    this->htim = htim;  // set `htim` to finalize the initialization
  }

  int32_t get_val() {
    this->counter = this->htim->Instance->CNT;
    return this->counter + this->offset;
  }

  void update_offset() {
    if (!this->htim)
      return;  // `HAL_TIM_PeriodElapsedCallback` gets triggered immediately
               // after calling `HAL_TIM_Base_Start_IT()` from the ctor, which
               // calls this function while the whole thing not fully
               // initialized yet and also the handle is still nullptr

    auto cnt = this->htim->Instance->CNT;
    if (cnt == this->counter) return;
    if (cnt < this->counter)
      this->offset += ARR_VALUE + 1;
    else
      this->offset -= ARR_VALUE - 1;

    this->counter = cnt;
  }

 private:
  TIM_HandleTypeDef* htim = nullptr;
  /* 64-bit integer for not over or underflowing easily */
  volatile int64_t offset{};
  int64_t counter{};
};
