#pragma once

#include <application/real_t.h>
#include <tim.h>

#include <algorithm>

class Motor {
 public:
  Motor() = default;
  void init(TIM_HandleTypeDef* htim, uint32_t pwm_channel_a,
            uint32_t pwm_channel_b, int direction)
  {
    this->htim = htim;
    this->pwm_channel_a = pwm_channel_a;
    this->pwm_channel_b = pwm_channel_b;
    this->direction = direction;
    this->ARR_VALUE = static_cast<real_t>(htim->Instance->ARR);

    HAL_TIM_PWM_Start(this->htim, this->pwm_channel_a);
    HAL_TIM_PWM_Start(this->htim, this->pwm_channel_b);
    this->set_pwm(0);
  }

  void set_pwm(real_t duty_cycle) {
    duty_cycle = std::clamp(duty_cycle, -this->ARR_VALUE, this->ARR_VALUE) *
                 this->direction;
    __HAL_TIM_SET_COMPARE(htim, this->pwm_channel_a,
                          duty_cycle > 0 ? duty_cycle : 0);
    __HAL_TIM_SET_COMPARE(htim, this->pwm_channel_b,
                          duty_cycle < 0 ? -duty_cycle : 0);
  }

 private:
  TIM_HandleTypeDef* htim = nullptr;
  uint32_t pwm_channel_a;
  uint32_t pwm_channel_b;
  int direction;
  real_t ARR_VALUE;
};
