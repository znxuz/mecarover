#pragma once

#include <tim.h>

#include <mecarover/mrtypes.h>

#ifdef __cplusplus
#include <array>

static constexpr int NUM_MOTORS = 4;

void hal_init(const Fahrzeug_t* fz);
int hal_wheel_vel_set_pwm(real_t* duty);
std::array<real_t, NUM_MOTORS> hal_encoder_delta();
int hal_encoder_read(real_t* DeltaPhi);
bool hal_get_estop();

extern "C"
{
#endif

void stm_encoder_cb(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif
