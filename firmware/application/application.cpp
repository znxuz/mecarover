#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <rtc.h>
#include <unistd.h>
#include <errno.h>
#include <stm32f7xx_hal_def.h>
#include <stm32f7xx_hal_uart.h>
#include <task.h>
#include <tim.h>
#include <ulog.h>
#include <usart.h>

#include <application/hal/hal.hpp>
#include <application/robot_params.hpp>

extern "C" {

static uint8_t uart_rx_buf[10];

// redirect stdout stdout/stderr to uart
int _write(int file, char* ptr, int len) {
  HAL_StatusTypeDef hstatus;

  if (file == STDOUT_FILENO || file == STDERR_FILENO) {
    hstatus = HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);

    if (hstatus == HAL_OK)
      return len;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart3.Instance) {
    // TODO use the buf
    HAL_UART_Transmit_IT(&huart3, uart_rx_buf, sizeof(uart_rx_buf));
    HAL_UART_Receive_IT(&huart3, uart_rx_buf, sizeof(uart_rx_buf));
  }
}

void application_start(void) {
  ULOG_INIT();
  ULOG_SUBSCRIBE(my_console_logger, ULOG_DEBUG_LEVEL);

  HAL_UART_Receive_IT(&huart3, uart_rx_buf, sizeof(uart_rx_buf));

  ULOG_INFO("app start");
  while (true) {
    HAL_Delay(1000);
  }

  osKernelStart();
  Error_Handler();  // because osKernelStart should never return
}
}
