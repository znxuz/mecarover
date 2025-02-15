#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <crc.h>
#include <queue.h>
#include <stdio.h>
#include <task.h>
#include <ulog.h>
#include <usart.h>

#include "shared.hpp"
#include "vel2d_frame.hpp"

static uint8_t uart_rx_buf[VEL2D_FRAME_LEN];

static TaskHandle_t task_handle;
static size_t crc_err;

extern "C" {
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == huart3.Instance) {
    configASSERT(task_handle != NULL);

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
    auto frame = *reinterpret_cast<const Vel2dFrame*>(uart_rx_buf);
    taskEXIT_CRITICAL();

    auto* vel = reinterpret_cast<uint8_t*>(&frame.vel);
    if (!frame.compare(HAL_CRC_Calculate(
            &hcrc, reinterpret_cast<uint32_t*>(vel), sizeof(frame.vel)))) {
      ULOG_ERROR("crc mismatch!");
      ++crc_err;
      continue;
    }

    ULOG_INFO("recv vel [%f %f %f]", frame.vel.x, frame.vel.y, frame.vel.z);
    xQueueSend(freertos::vel_sp_queue, &frame.vel, NO_BLOCK);
  }
}
}

namespace freertos {

void task_vel_recv_init() {
  HAL_UART_Receive_IT(&huart3, uart_rx_buf, sizeof(uart_rx_buf));

  configASSERT(xTaskCreate(task_impl, "task_recv_vel", 128 * 4, NULL,
                           osPriorityNormal1, &task_handle) == pdPASS);
}

}  // namespace freertos
