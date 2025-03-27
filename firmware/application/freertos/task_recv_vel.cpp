#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <crc.h>
#include <queue.h>
#include <task.h>
#include <ulog.h>
#include <usart.h>

#include <application/pose_types.hpp>

#include "shared.hpp"
#include "vel2d_frame.hpp"

static uint8_t uart_rx_buf[VEL2D_FRAME_LEN];
volatile static uint16_t rx_len;

static TaskHandle_t task_handle;
static size_t crc_err;

extern "C" {
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
  if (huart->Instance != huart3.Instance) return;

  rx_len = size;

  static BaseType_t xHigherPriorityTaskWoken;
  configASSERT(task_handle != NULL);
  vTaskNotifyGiveFromISR(task_handle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

  HAL_UARTEx_ReceiveToIdle_IT(&huart3, uart_rx_buf, sizeof(uart_rx_buf));
}

static void task_impl(void*) {
  constexpr TickType_t NO_BLOCK = 0;
  size_t len = 0;

  // check for sending Vel2d directly as vPose for pose control task
  configASSERT(sizeof(Vel2d) == sizeof(imsl::vPose));

  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    len = rx_len;  // atomic access shouldn't need to protected

    if (len != VEL2D_FRAME_LEN) {
      ULOG_ERROR("parsing velocity failed: insufficient bytes received");
      continue;
    }

    auto frame = *reinterpret_cast<const Vel2dFrame*>(uart_rx_buf);
    auto* vel_data = reinterpret_cast<uint8_t*>(&frame.vel);
    if (!frame.compare(HAL_CRC_Calculate(
            &hcrc, reinterpret_cast<uint32_t*>(vel_data), sizeof(frame.vel)))) {
      ULOG_ERROR("crc mismatch!");
      ++crc_err;
      continue;
    }

    frame.vel.x *= 1000;  // m to mm
    frame.vel.y *= 1000;  // m to mm

    ULOG_INFO("[recv vel] [%f %f %f]", frame.vel.x, frame.vel.y, frame.vel.z);

    xQueueSend(freertos::vel_sp_queue, &frame.vel, NO_BLOCK);
  }
}
}

namespace freertos {

void task_vel_recv_init() {
  HAL_UARTEx_ReceiveToIdle_IT(&huart3, uart_rx_buf, sizeof(uart_rx_buf));

  constexpr size_t STACK_SIZE = configMINIMAL_STACK_SIZE * 4;
#ifdef FREERTOS_STATIC_INIT
  static StackType_t taskStack[STACK_SIZE];
  static StaticTask_t taskBuffer;
  configASSERT((task_handle = xTaskCreateStatic(
                    task_impl, "recv_vel", STACK_SIZE, NULL, osPriorityNormal,
                    taskStack, &taskBuffer)) != NULL);
#else
  configASSERT(xTaskCreate(task_impl, "recv_vel", STACK_SIZE, NULL,
                           osPriorityNormal, &task_handle) == pdPASS);
#endif
}

}  // namespace freertos
