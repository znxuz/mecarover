#include <cmsis_os2.h>
#include <mecarover/controls/ControllerTask.h>
#include <mecarover/lidar/lidar.h>
#include <mecarover/retarget.h>
#include <rtc.h>
#include <tim.h>
#include <ulog.h>
#include <usart.h>

#include <mecarover/micro_ros/micro_ros.hpp>
#include <mecarover/micro_ros/micro_ros_legacy.hpp>
#include <mecarover/robot_params.hpp>

#include "mecarover/rtos_config.h"

LaserScanner laser_scanner;

extern "C" {

volatile unsigned long ulHighFrequencyTimerTicks;

void configureTimerForRunTimeStats(void) {
  ulHighFrequencyTimerTicks = 0;
  HAL_TIM_Base_Start_IT(&htim13);
}

unsigned long getRunTimeCounterValue(void) { return ulHighFrequencyTimerTicks; }

int _gettimeofday(struct timeval* tv, void* tzvp) {
  return 0;  // not used, thus unimplemented to satisfy the compiler
}

void my_console_logger(ulog_level_t severity, char* msg) {
  static RTC_TimeTypeDef sTime;
  static RTC_DateTypeDef sDate;

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  /* must also get the date *after* getting the time to unlock the time values,
   * otherwise GetTime() won't work */
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  printf("%02d:%02d:%02d [%s]: %s\n", sTime.Hours, sTime.Minutes, sTime.Seconds,
         ulog_level_name(severity), msg);
}

void mecarover_start(void) {
  retarget_init(&huart3);
  ULOG_INIT();
  ULOG_SUBSCRIBE(my_console_logger, ULOG_DEBUG_LEVEL);

  laser_scanner.init();

  // auto* controller_task = new imsl::vehiclecontrol::ControllerTask<real_t>();
  // xTaskCreate(micro_ros_legacy, "micro_ros", MAIN_TASK_STACK_SIZE,
  //             controller_task, MICRO_ROS_TASK_PRIORITY, NULL);

  xTaskCreate(micro_ros, "micro_ros", 3000, NULL, MICRO_ROS_TASK_PRIORITY,
              NULL);

  osKernelStart();
  Error_Handler();  // because osKernelStart should never return
}
}
