/*
 * STM_Counter.cc
 *
 *  Created on: Nov 13, 2021
 *      Author: Fabia
 */

#include "stm_counter.h"
#include "stm_hal.h"

#define New_Zero 32767
#define Max_Value 65535
//#define Max_Value 100
#define Min_Value 0
//#define New_Zero 50

/*
 *
 * STM32F767ZI insg: 14 Counter/Timer
 * TIM1 6 Channel
 * TIM2 bis TIM5 - TIM8 4 Channel
 * TIM9 - TIM12 2 Channel
 * TIM6 bis TIM7 - TIM10 - TIM11 - TIM13 - TIM14 One Pulse Mode
 *
 */

int64_t flow[NumMotors] = { -New_Zero, -New_Zero, -New_Zero, -New_Zero };
int64_t akt_pos[NumMotors] = {0,0,0,0};
TIM_HandleTypeDef timer[NumMotors];
static bool hal_is_init;

//init der Encoder
bool STMCounter::init(TIM_HandleTypeDef *htim, int id)
{

	__HAL_TIM_SET_COUNTER(htim, New_Zero);

		HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_1);
		HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_2);
		HAL_TIM_Base_Start_IT(htim); //Start update Interrupt the trigger by overflow/underflow
		akt_pos[id] = __HAL_TIM_GET_COUNTER(htim);


		timer[id] = *htim;

	is_init = true;
	return is_init;
}

uint64_t STMCounter::getCount(int id)
{
	if (is_init) {

		 enc_value = __HAL_TIM_GET_COUNTER(&timer[id]);

		 akt_pos[id] = enc_value;
		 counter = akt_pos[id] + flow[id];

			return counter;

	}
	return 0;

}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM12) {
		HAL_IncTick();
	}

	/* USER CODE BEGIN Callback 1 */

	if(hal_is_init == true){

		//hinzu: ob Rad sich vor- oder rückwärts dreht
	if (htim->Instance == TIM3) {
		//Overflow
		if (__HAL_TIM_GET_COUNTER(htim) < akt_pos[0]) {
			flow[0] += Max_Value + 1;
		}
		//Underflow
		if (__HAL_TIM_GET_COUNTER(htim) > akt_pos[0]) {
			flow[0] -= Max_Value - 1;
		}

	}

	if (htim->Instance == TIM1) {
		//Overflow
			if (__HAL_TIM_GET_COUNTER(htim) < akt_pos[1]) {
				flow[1] += Max_Value + 1;
			}
			//Underflow
			if (__HAL_TIM_GET_COUNTER(htim) > akt_pos[1]) {
				flow[1] -= Max_Value - 1;
			}
	}

	if (htim->Instance == TIM2) {
		//Overflow
			if (__HAL_TIM_GET_COUNTER(htim) < akt_pos[2]) {
				flow[2] += Max_Value + 1;
			}
			//Underflow
			if (__HAL_TIM_GET_COUNTER(htim) > akt_pos[2]) {
				flow[2] -= Max_Value - 1;
			}
	}

	if (htim->Instance == TIM4) {
		//Overflow
			if (__HAL_TIM_GET_COUNTER(htim) < akt_pos[3]) {
				flow[3] += Max_Value + 1;
			}
			//Underflow
			if (__HAL_TIM_GET_COUNTER(htim) > akt_pos[3]) {
				flow[3] -= Max_Value - 1;
			}
	}

	}

	/* USER CODE END Callback 1 */
}
