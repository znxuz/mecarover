#include "recv_vel_task.hpp"

#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <crc.h>
#include <stdio.h>
#include <task.h>
#include <ulog.h>
#include <usart.h>

#include <application/vel2d_frame.hpp>
#include <optional>

static std::optional<Vel2dFrame> frame;
static uint8_t uart_rx_buf[VEL2D_FRAME_LEN];

static TaskHandle_t taskRecvVelHandle;
static size_t crc_err;

extern "C" {
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == huart3.Instance) {
    configASSERT(taskRecvVelHandle != NULL);

    frame = *reinterpret_cast<const Vel2dFrame*>(uart_rx_buf);

    BaseType_t xHigherPriorityTaskWoken;
    vTaskNotifyGiveFromISR(taskRecvVelHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    HAL_UART_Receive_IT(&huart3, uart_rx_buf, sizeof(uart_rx_buf));
  }
}

void task1(void*) {
  taskRecvVelHandle = xTaskGetCurrentTaskHandle();
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

namespace freertos {

void recv_vel_task_create() {
  HAL_UART_Receive_IT(&huart3, uart_rx_buf, sizeof(uart_rx_buf));
  xTaskCreate(task1, "recv_vel_task", 128 * 4, NULL, osPriorityNormal,
              &taskRecvVelHandle);
}

}  // namespace freertos
