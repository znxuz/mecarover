#include "application.hpp"

#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <crc.h>
#include <errno.h>
#include <rtc.h>
#include <stm32f7xx_hal_def.h>
#include <stm32f7xx_hal_uart.h>
#include <task.h>
#include <tim.h>
#include <ulog.h>
#include <unistd.h>
#include <usart.h>

#include <application/hal/hal.hpp>
#include <application/robot_params.hpp>
#include <application/vel2d_frame.hpp>
#include <optional>

extern "C" {

volatile unsigned long ulHighFrequencyTimerTicks;

static std::optional<Vel2dFrame> frame;
static uint8_t uart_rx_buf[VEL2D_FRAME_LEN];
static size_t crc_err;

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

static TaskHandle_t task1_handle;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == huart3.Instance) {
    frame = *reinterpret_cast<const Vel2dFrame*>(uart_rx_buf);

    configASSERT(task1_handle != NULL);
    BaseType_t xHigherPriorityTaskWoken;
    vTaskNotifyGiveFromISR(task1_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    HAL_UART_Receive_IT(&huart3, uart_rx_buf, sizeof(uart_rx_buf));
  }
}

void task1(void*) {
  task1_handle = xTaskGetCurrentTaskHandle();
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    taskENTER_CRITICAL();
    auto v = *frame;
    taskEXIT_CRITICAL();

    uint8_t* data = reinterpret_cast<uint8_t*>(&v.vel);
    if (!v.compare(HAL_CRC_Calculate(&hcrc, reinterpret_cast<uint32_t*>(data),
                                     sizeof(v.vel)))) {
      ULOG_ERROR("crc mismatch!");
      ++crc_err;
      continue;
    }

    printf("%f %f %f\n", v.vel.x, v.vel.y, v.vel.z);
  }
}
}

void application_start(void) {
  ULOG_INIT();
  ULOG_SUBSCRIBE(my_console_logger, ULOG_DEBUG_LEVEL);

  HAL_UART_Receive_IT(&huart3, uart_rx_buf, sizeof(uart_rx_buf));

  ULOG_INFO("app start");

  xTaskCreate(task1, "task1", 128 * 4, NULL, osPriorityNormal, &task1_handle);

  osKernelStart();
  Error_Handler();  // because osKernelStart should never return
}
