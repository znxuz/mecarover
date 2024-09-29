#pragma once

#include <tim.h>

#define ARR_VALUE 65535

class STMEncoder {
public:
	STMEncoder() = default;
	bool init(TIM_HandleTypeDef* htim);
	int32_t get_val();
	void update_offset();

private:
	TIM_HandleTypeDef* htim;
	int32_t counter;
	int64_t offset; /* 64-bit integer for not over or underflowing easily */
	bool is_init = false;
};
