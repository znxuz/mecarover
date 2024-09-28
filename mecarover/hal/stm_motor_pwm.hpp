/*
 * STM_MCPWM_DC.h
 *
 *  Created on: Nov 13, 2021
 *      Author: Fabia
 */

#pragma once

#include <tim.h>
#include "stm32f7xx_hal_tim.h"

class STMMotorPWM {
public:
	STMMotorPWM() = default;
	bool init(int motor, TIM_HandleTypeDef *htim, uint32_t Channel_PwmA,
		uint32_t Channel_PwmB);
	void setPWM(float duty_cicle, TIM_HandleTypeDef *timer, int ChannelA,
		int ChannelB);

private:
	bool is_init = false;
};
