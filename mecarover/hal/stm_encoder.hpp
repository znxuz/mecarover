#pragma once

#include "mecarover/mrlogger/mrlogger.h"
#include <tim.h>

#define ARR_VALUE 65535

class STMEncoder {
public:
	STMEncoder() = default;

	bool init(TIM_HandleTypeDef* htim)
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

	int32_t get_val()
	{
		if (!is_init) [[unlikely]] {
			log_message(log_error, "encoder not initialized");
			return 0;
		}

		this->counter = __HAL_TIM_GET_COUNTER(this->htim);
		return this->counter + this->offset;
	}

	void update_offset()
	{
		if (__HAL_TIM_GET_COUNTER(this->htim) < this->counter)
			this->offset += ARR_VALUE + 1;
		else
			this->offset -= ARR_VALUE - 1;
	}

private:
	TIM_HandleTypeDef* htim;
	/* 64-bit integer for not over or underflowing easily */
	int64_t counter;
	int64_t offset;
	bool is_init = false; // i dont know if this check if really necessary...
						  // TODO: only one way to find out
};
