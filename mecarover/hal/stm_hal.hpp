#pragma once

#include <array>

#include <mecarover/mrtypes.h>

void hal_init(const Fahrzeug_t* fz);
int hal_wheel_vel_set(real_t* w);
int hal_encoder_read(real_t* DeltaPhi);
int mr_hal_wheel_vel_set_pwm(float* duty);
bool hal_get_estop();

static constexpr int NumMotors = 4;

std::array<uint64_t, NumMotors> hal_encoder_getval();
