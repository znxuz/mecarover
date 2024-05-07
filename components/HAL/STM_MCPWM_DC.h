/*
 * STM_MCPWM_DC.h
 *
 *  Created on: Nov 13, 2021
 *      Author: Fabia
 */

#ifndef HAL_STM_MCPWM_DC_H_
#define HAL_STM_MCPWM_DC_H_

#include "stm32f7xx_hal.h"

class STM_MCPWM_DC {
public:
	STM_MCPWM_DC() {
		is_init = false;
	}
	bool init(int motor, TIM_HandleTypeDef *htim, uint32_t Channel_PwmA,
			uint32_t Channel_PwmB);
	void setPWM(float duty_cicle, TIM_HandleTypeDef *timer, int ChannelA,
			int ChannelB); // duty cicle in +/- percent

private:

	bool is_init;
	int index = 0;
	int Num_Motors = 4;

};

#endif /* HAL_STM_MCPWM_DC_H_ */
