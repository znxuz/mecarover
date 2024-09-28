#pragma once

#include <tim.h>

#ifdef __cplusplus
class STMCounter {
public:
	STMCounter() = default;
	bool init(TIM_HandleTypeDef* htim, int id);
	uint64_t getCount(int id);

private:
	bool is_init = false;
};

extern "C"
{
#endif

void stm_counter_cb(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif
