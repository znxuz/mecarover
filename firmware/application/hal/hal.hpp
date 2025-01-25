#pragma once

#include <tim.h>

#ifdef __cplusplus
#include <application/robot_params.hpp>
#include <array>

void hal_init();
void hal_wheel_set_pwm(
    const std::array<real_t, robot_params::N_WHEEL>& duty_cycle);
std::array<real_t, robot_params::N_WHEEL> hal_wheel_delta_phi();

extern "C" {
#endif

void encoder_tim_cb(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif
