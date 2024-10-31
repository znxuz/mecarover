#include <cmsis_os2.h>
#include <mecarover/controls/ControllerTask.h>
#include <mecarover/lidar/lidar.h>
#include <mecarover/retarget.h>
#include <tim.h>
#include <ulog.h>
#include <usart.h>

#include <mecarover/hal/stm_hal.hpp>
#include <mecarover/micro_ros/micro_ros.hpp>
#include <mecarover/micro_ros/micro_ros_legacy.hpp>
#include <mecarover/robot_params.hpp>

#include "mecarover/rtos_config.h"

LaserScanner laser_scanner;

extern "C" {

volatile unsigned long ulHighFrequencyTimerTicks;

// TODO: tim13 or tim12?
void configureTimerForRunTimeStats(void) {
  ulHighFrequencyTimerTicks = 0;
  HAL_TIM_Base_Start_IT(&htim13);
}

unsigned long getRunTimeCounterValue(void) { return ulHighFrequencyTimerTicks; }

int _gettimeofday(struct timeval* tv, void* tzvp) {
  return 0;  // not used, thus unimplemented to satisfy the compiler
}

void my_console_logger(ulog_level_t severity, char* msg) {
  printf("[%s]: %s\n",
         // get_timestamp(),    // user defined function
         ulog_level_name(severity), msg);
}

void mecarover_start(void) {
  retarget_init(&huart3);
  ULOG_INIT();
  ULOG_SUBSCRIBE(my_console_logger, ULOG_DEBUG_LEVEL);
  hal_init();

  laser_scanner.init();

  // auto* controller_task = new imsl::vehiclecontrol::ControllerTask<real_t>();
  // xTaskCreate(micro_ros_legacy, "micro_ros", MAIN_TASK_STACK_SIZE,
  //             controller_task, MICRO_ROS_TASK_PRIORITY, NULL);

  xTaskCreate(micro_ros, "micro_ros", MAIN_TASK_STACK_SIZE, NULL,
              MICRO_ROS_TASK_PRIORITY, NULL);

  osKernelStart();
  Error_Handler();  // because osKernelStart should never return
}
}
