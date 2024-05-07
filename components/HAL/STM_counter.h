/*
 * STM_counter.h
 *
 *  Created on: Nov 13, 2021
 *      Author: Fabia
 */
#ifndef STM_COUNTER_H
#define STM_COUNTER_H

#include "hal.h"

extern "C" {

class STM_Counter {
public:
	STM_Counter() {
		is_init = false;
	}

	bool init(TIM_HandleTypeDef *htim, int id);

	uint64_t getCount(int id);



private:

	bool is_init;
	uint64_t enc_value;
	 int64_t counter;

};

}

#endif /* STM_COUNTER_H */

