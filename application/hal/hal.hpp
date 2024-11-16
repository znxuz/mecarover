#pragma once

#include <tim.h>

#ifdef __cplusplus
#include <application/robot_params.hpp>
#include <array>

void hal_init();
void hal_wheel_vel_set_pwm(
    const std::array<real_t, robot_params::N_WHEEL>& duty_cycle);
std::array<real_t, robot_params::N_WHEEL> hal_encoder_delta_rad();
std::array<uint32_t, robot_params::N_WHEEL> hal_encoder_val();

extern "C" {
#endif

void encoder_tim_cb(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif
