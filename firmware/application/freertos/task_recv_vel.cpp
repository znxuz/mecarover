#include "task_recv_vel.hpp"

#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <crc.h>
#include <stdio.h>
#include <task.h>
#include <ulog.h>
#include <usart.h>

#include <optional>

#include "vel2d_frame.hpp"

static std::optional<Vel2dFrame> frame;
static uint8_t uart_rx_buf[VEL2D_FRAME_LEN];

static TaskHandle_t task_handle;
static QueueHandle_t vel_sp_queue;
static size_t crc_err;

extern "C" {
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == huart3.Instance) {
    configASSERT(task_handle != NULL);

    frame = *reinterpret_cast<const Vel2dFrame*>(uart_rx_buf);

    BaseType_t xHigherPriorityTaskWoken;
    vTaskNotifyGiveFromISR(task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    HAL_UART_Receive_IT(&huart3, uart_rx_buf, sizeof(uart_rx_buf));
  }
}

static void task_impl(void*) {
  constexpr TickType_t NO_BLOCK = 0;

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

    xQueueSend(vel_sp_queue, &v.vel, NO_BLOCK);
  }
}
}

namespace freertos {

void task_vel_recv_init(QueueHandle_t vel_sp_queue_out) {
  vel_sp_queue = vel_sp_queue_out;

  HAL_UART_Receive_IT(&huart3, uart_rx_buf, sizeof(uart_rx_buf));

  configASSERT(xTaskCreate(task_impl, "task_recv_vel", 128 * 4, NULL,
                           osPriorityNormal, &task_handle) == pdPASS);
}

}  // namespace freertos
