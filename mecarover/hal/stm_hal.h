#pragma once

#include <mrtypes.h>

void hal_init(Fahrzeug_t *fz);
int hal_wheel_vel_set(real_t *w);
int hal_encoder_read(real_t *DeltaPhi);
int mr_hal_wheel_vel_set_pwm(float *duty);
bool hal_get_estop();

static constexpr int NumMotors = 4;
