#include <cmsis_os2.h>
#include <printf.h>
#include <rtc.h>
#include <tim.h>
#include <ulog.h>

#include <application/freertos/init.hpp>
#include <threadsafe_sink.hpp>

using namespace freertos;

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

// tsink::write_blocking writing the whole string directly into the sink, thus
// unused
/* void _putchar(char c) { tsink::write_or_fail(c); } */

void my_console_logger(ulog_level_t severity, char* msg) {
  static char buf[100];
  static RTC_TimeTypeDef sTime;
  static RTC_DateTypeDef sDate;

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  /* must also get the date *after* getting the time to unlock the time values,
   * otherwise GetTime() won't work */
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  tsink::write_blocking(
      buf,
      snprintf(buf, sizeof(buf), "%02d:%02d:%02d [%s]: %s\n", sTime.Hours,
               sTime.Minutes, sTime.Seconds, ulog_level_name(severity), msg));
}

void application_start(void) {
  ULOG_INIT();
  ULOG_SUBSCRIBE(my_console_logger, ULOG_DEBUG_LEVEL);

  init();

  ULOG_INFO("kernel start");
  osKernelStart();
  Error_Handler();  // because osKernelStart should never return
}
}
