#include <application/legacy/controls/ControllerTask.h>
#include <application/legacy/lidar/lidar.h>
#include <application/legacy/mrlogger/mrlogger.h>
#include <application/retarget.h>
#include <application/rtos_config.h>
#include <cmsis_os2.h>
#include <rtc.h>
#include <tim.h>
#include <ulog.h>
#include <usart.h>

#include <application/hal/hal.hpp>
#include <application/legacy/micro_ros_legacy.hpp>
#include <application/micro_ros/micro_ros.hpp>
#include <application/robot_params.hpp>

// LaserScanner lidar;

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

void application_start(void) {
  retarget_init(&huart3);
  ULOG_INIT();
  ULOG_SUBSCRIBE(my_console_logger, ULOG_DEBUG_LEVEL);

#ifdef LEGACY
  logger_init();
  hal_init();
  lidar.init();
  auto* controller = new imsl::vehiclecontrol::ControllerTask<real_t>();
  xTaskCreate(micro_ros_legacy, "micro_ros", MAIN_TASK_STACK_SIZE, controller,
              MICRO_ROS_TASK_PRIORITY, NULL);
#else
  xTaskCreate(micro_ros, "micro_ros", 3000, NULL, MICRO_ROS_TASK_PRIORITY,
              NULL);
#endif

  osKernelStart();
  Error_Handler();  // because osKernelStart should never return
}
}
