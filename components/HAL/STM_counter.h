/*
 * STM_counter.h
 *
 *  Created on: Nov 13, 2021
 *      Author: Fabia
 */
#pragma once

#include "hal.h"

extern "C"
{
class STM_Counter
{
public:
	STM_Counter() = default;
	bool init(TIM_HandleTypeDef *htim, int id);
	uint64_t getCount(int id);

private:
	bool is_init = false;
	uint64_t enc_value;
	int64_t counter;
};
}
