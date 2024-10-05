#pragma once

#include <tim.h>

#include <mecarover/mrtypes.h>

#ifdef __cplusplus
#include <array>
#include <mecarover/robot_params.hpp>

static constexpr int NUM_MOTORS = 4;

void hal_init(const robot_param_t* robot_params);
void hal_wheel_vel_set_pwm(const std::array<real_t, N_WHEEL>& duty_cycle);
std::array<real_t, NUM_MOTORS> hal_encoder_delta();
bool hal_get_estop();

extern "C"
{
#endif

void stm_encoder_cb(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif
