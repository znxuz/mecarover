#include "application.hpp"

#include <cmsis_os2.h>
#include <errno.h>
#include <rtc.h>
#include <stdio.h>
#include <tim.h>
#include <ulog.h>
#include <unistd.h>
#include <usart.h>

#include <application/freertos/init.hpp>

extern "C" {

volatile unsigned long ulHighFrequencyTimerTicks;

int _write(int file, char* ptr, int len) {
  if (file == STDOUT_FILENO || file == STDERR_FILENO) {
    taskENTER_CRITICAL();
    HAL_StatusTypeDef status =
        HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    taskEXIT_CRITICAL();

    if (status == HAL_OK)
      return len;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

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
}

void application_start(void) {
  ULOG_INIT();
  ULOG_SUBSCRIBE(my_console_logger, ULOG_DEBUG_LEVEL);

  freertos::init();

  ULOG_INFO("kernel start");
  osKernelStart();
  Error_Handler();  // because osKernelStart should never return
}
