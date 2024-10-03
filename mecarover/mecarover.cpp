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

LaserScanner ls;

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
	hal_init(&fz);

	log_message(log_info, "mecarover start");
	if (fz.type != MRC_VEHICLETYPE_MECANUM) {
		log_message(log_error, "Vehicle type is not mecanum");
		Error_Handler();
	}

	auto* controller_task = new imsl::vehiclecontrol::ControllerTask<real_t>();
	controller_task->Init(&fz, Regler, Ta);
	ls.init_LaserScanner();

	const osThreadAttr_t executor_attributes = {
		.name = "micro_ros",
		.stack_size = STACK_SIZE,
		.priority = (osPriority_t)MICRO_ROS_TASK_PRIORITY,
	};
	osThreadNew(micro_ros_legacy, controller_task, &executor_attributes);

	osKernelStart();
	Error_Handler(); // because osKernelStart should never return
}
}
