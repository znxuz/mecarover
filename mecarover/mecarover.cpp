#include <tim.h>
#include <usart.h>

#include <mecarover/controls/MecanumControllerTask.h>
#include <mecarover/hal/stm_hal.hpp>
#include <mecarover/lidar/lidar.h>
#include <mecarover/micro_ros/micro_ros.hpp>
#include <mecarover/retarget.h>
#include <mecarover/robot_params.hpp>

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

	if (fz.type != MRC_VEHICLETYPE_MECANUM) {
		log_message(log_error, "Vehicle type is not mecanum");
		Error_Handler();
	}

	osKernelInitialize();

	auto* controller_task
		= new imsl::vehiclecontrol::MecanumControllerTask<real_t>();
	controller_task->Init(&fz, Regler, Ta);
	ls.init_LaserScanner();

	const osThreadAttr_t executor_attributes = {
		.name = "micro_ros",
		.stack_size = STACK_SIZE,
		.priority = (osPriority_t)MICRO_ROS_TASK_PRIORITY,
	};
	osThreadNew(micro_ros, NULL, &executor_attributes);

	osKernelStart();
}
}
