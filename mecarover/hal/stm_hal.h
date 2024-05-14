#pragma once

#include <tim.h>
#include <stm32f7xx_hal_tim.h>
#include <mrtypes.h>

#if __cplusplus
extern "C"
{
#endif

bool hal_init(Fahrzeug_t *fz);
int hal_wheel_vel_set(real_t *w);
int hal_encoder_read(real_t *DeltaPhi);
int mr_hal_wheel_vel_set_pwm(float *duty);
bool hal_get_estop();

void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

const int NumMotors = 4;
#if __cplusplus
}
#endif
