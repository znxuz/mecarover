/*
 * STM_MCPWM_DC.h
 *
 *  Created on: Nov 13, 2021
 *      Author: Fabia
 */

#pragma once

#include "stm32f7xx_hal.h"

class STM_MCPWM_DC
{
public:
	STM_MCPWM_DC() = default;
	bool init(int motor, TIM_HandleTypeDef *htim, uint32_t Channel_PwmA,
			uint32_t Channel_PwmB);
	void setPWM(float duty_cicle, TIM_HandleTypeDef *timer, int ChannelA,
			int ChannelB); // duty cicle in +/- percent

private:
	bool is_init = false;
	int index = 0;
	int Num_Motors = 4;
};
