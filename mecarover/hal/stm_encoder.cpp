#include "stm_encoder.hpp"

#include <mecarover/mrlogger/mrlogger.h>

/*
 *
 * STM32F767ZI insg: 14 Counter/Timer
 * TIM1 6 Channel
 * TIM2 bis TIM5 - TIM8 4 Channel
 * TIM9 - TIM12 2 Channel
 * TIM6 bis TIM7 - TIM10 - TIM11 - TIM13 - TIM14 One Pulse Mode
 *
 */

bool STMEncoder::init(TIM_HandleTypeDef* htim)
{
	this->offset = -(ARR_VALUE / 2);

	__HAL_TIM_SET_COUNTER(htim, ARR_VALUE / 2);
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(htim);

	this->counter = __HAL_TIM_GET_COUNTER(htim);
	this->htim = htim;

	is_init = true;
	return is_init;
}

int32_t STMEncoder::get_val()
{
	// i dont know if this check if really necessary...
	// TODO: only one way to find out
	if (!is_init) [[unlikely]] {
		log_message(log_error, "encoder not initialized");
		return 0;
	}

	this->counter = __HAL_TIM_GET_COUNTER(this->htim);
	return this->counter + this->offset;
}

void STMEncoder::update_offset()
{
	if (__HAL_TIM_GET_COUNTER(this->htim) < this->counter)
		this->offset += ARR_VALUE + 1;
	else
		this->offset -= ARR_VALUE - 1;
}
