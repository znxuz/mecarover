#include "mecarover/rtos_config.h"
#include <mecarover/controls/ControllerTask.h>
#include <mecarover/hal/stm_hal.hpp>
#include <mecarover/lidar/lidar.h>
#include <mecarover/micro_ros/micro_ros.hpp>
#include <mecarover/micro_ros/micro_ros_legacy.hpp>
#include <mecarover/mrlogger/mrlogger.h>
#include <mecarover/retarget.h>
#include <mecarover/robot_params.hpp>

#include <tim.h>
#include <usart.h>

LaserScanner laser_scanner;

extern "C"
{

volatile unsigned long ulHighFrequencyTimerTicks;

void configureTimerForRunTimeStats(void)
{
	ulHighFrequencyTimerTicks = 0;
	HAL_TIM_Base_Start_IT(&htim13);
}

unsigned long getRunTimeCounterValue(void) { return ulHighFrequencyTimerTicks; }

int _gettimeofday(struct timeval* tv, void* tzvp)
{
	return 0; // not used, thus unimplemented to satisfy the compiler
}

void mecarover_start(void)
{
	retarget_init(&huart3);
	logger_init();
	hal_init();

	log_message(log_info, "mecarover start");

	auto* controller_task = new imsl::vehiclecontrol::ControllerTask<real_t>();
	laser_scanner.init();

	const osThreadAttr_t executor_attributes = {
		.name = "micro_ros",
		.stack_size = MAIN_TASK_STACK_SIZE,
		.priority = (osPriority_t)MICRO_ROS_TASK_PRIORITY,
	};
	osThreadNew(micro_ros_legacy, controller_task, &executor_attributes);

	osKernelStart();
	Error_Handler(); // because osKernelStart should never return
}
}
