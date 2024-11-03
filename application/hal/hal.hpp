#pragma once

#include <tim.h>

#ifdef __cplusplus
#include <application/robot_params.hpp>
#include <array>

void hal_init();
void hal_wheel_vel_set_pwm(const std::array<real_t, N_WHEEL>& duty_cycle);
std::array<real_t, N_WHEEL> hal_encoder_delta_rad();
bool hal_get_estop();

extern "C" {
#endif

void stm_encoder_cb(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif
