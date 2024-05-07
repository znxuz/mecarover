/*
 * hal.h
 *
 *  Created on: 13.11.2021
 *      Author: Fabia
 */
#pragma once
#ifndef HAL_HAL_H_
#define HAL_HAL_H_

#include "mrtypes.h"            // for Fahrzeug_t
#include "main.h"
#include "system_config.h"			// for real_t
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "cmsis_os.h"
//#include "lwip.h"
//#include "dma.h"


#if __cplusplus
extern "C"{
#endif

// HAL


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


#endif /* HAL_HAL_H_ */
