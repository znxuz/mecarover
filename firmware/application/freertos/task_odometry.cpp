#include "task_odometry.hpp"

#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <queue.h>

#include <application/hal/hal.hpp>

extern "C" {

TaskHandle_t task_handle;
QueueHandle_t enc_delta_queue;
QueueHandle_t odom_queue;

static void task_impl(void*) {
  while (true) {
    vTaskDelay(portMAX_DELAY);
  }
}
}

namespace freertos {

void task_odometry_init(QueueHandle_t enc_delta_queue_out,
                        QueueHandle_t odom_queue_out) {
  enc_delta_queue = enc_delta_queue_out;
  odom_queue = odom_queue_out;

  configASSERT(xTaskCreate(task_impl, "task_odometry", 128 * 4, NULL,
                           osPriorityNormal, &task_handle) == pdPASS);
}

}  // namespace freertos
