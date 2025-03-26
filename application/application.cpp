#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <rtc.h>
#include <stm32f7xx_hal_def.h>
#include <stm32f7xx_hal_uart.h>
#include <task.h>
#include <tim.h>
#include <ulog.h>
#include <usart.h>

#include <application/freertos/task_runtime_stats.hpp>
#include <application/hal/hal.hpp>
#include <application/micro_ros/micro_ros.hpp>
#include <application/robot_params.hpp>
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance != huart3.Instance) return;

  tsink_consume_complete<TSINK_CALL_FROM::ISR>();
}

void application_start(void) {
  // ulog not enabled via macro for profiling
  ULOG_INIT();
  ULOG_SUBSCRIBE(my_console_logger, ULOG_DEBUG_LEVEL);

  auto tsink_consume = [](const uint8_t* buf, size_t size) static {
    auto flush_cache_aligned = [](uintptr_t addr, size_t size) static {
      constexpr auto align_addr = [](uintptr_t addr) { return addr & ~0x1F; };
      constexpr auto align_size = [](uintptr_t addr, size_t size) {
        return size + ((addr) & 0x1F);
      };

      SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(align_addr(addr)),
                              align_size(addr, size));
    };

    flush_cache_aligned(reinterpret_cast<uintptr_t>(buf), size);
    HAL_UART_Transmit_DMA(&huart3, buf, size);
  };

  tsink_init(tsink_consume, osPriorityAboveNormal);
  task_runtime_stats_init();
  xTaskCreate(micro_ros, "uros", 3000, NULL, osPriorityNormal, NULL);

  osKernelStart();
  Error_Handler();  // because osKernelStart should never return
}
}
