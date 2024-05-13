/*
 * STM_MCPWM_DC.cc
 *
 *  Created on: Nov 13, 2021
 *      Author: Fabia
 */

#include "stm_motor_pwm.h"

//init des PWM Signals
bool STMMotorPWM::init(int motor, TIM_HandleTypeDef *htim,
		uint32_t Channel_PwmA, uint32_t Channel_PwmB)
{
	//Start Timer
	HAL_TIM_PWM_Start(htim, Channel_PwmA);
	HAL_TIM_PWM_Start(htim, Channel_PwmB);

	__HAL_TIM_SET_COMPARE(htim, Channel_PwmA, 0);
	__HAL_TIM_SET_COMPARE(htim, Channel_PwmB, 0);

	is_init = true;
	return is_init;
}

void STMMotorPWM::setPWM(float duty_cicle, TIM_HandleTypeDef *timer,
		int ChannelA, int ChannelB)
{
	if (is_init) {
		// pwm must be in the range from -100.0 to +100.0
		if (duty_cicle > 100.0) {
			duty_cicle = 100.0;
		} else if (duty_cicle < -100.0) {
			duty_cicle = -100.0;
		}

		//		printf("Dutycicle setPWM: %.4f\n\r", duty_cicle);

		if (duty_cicle == 0.0) {

			__HAL_TIM_SET_COMPARE(timer, ChannelA, 0); //set dutycicle = 0
			__HAL_TIM_SET_COMPARE(timer, ChannelB, 0);

		} else if (duty_cicle > 0.0) {

			__HAL_TIM_SET_COMPARE(timer, ChannelB, 0);
			__HAL_TIM_SET_COMPARE(timer, ChannelA, duty_cicle);

		} else {

			__HAL_TIM_SET_COMPARE(timer, ChannelA, 0);
			__HAL_TIM_SET_COMPARE(timer, ChannelB, -duty_cicle);

		}
	}
}
