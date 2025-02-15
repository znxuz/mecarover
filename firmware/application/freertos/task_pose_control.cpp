#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <task.h>
#include <ulog.h>

#include "shared.hpp"
#include "vel2d_frame.hpp"

static TaskHandle_t task_handle;
static Vel2d vel_sp;

extern "C" {

static void task_impl(void*) {
  while (true) {
    xQueueReceive(freertos::vel_sp_queue, &vel_sp, portMAX_DELAY);
    ULOG_INFO("vel from queue [%f %f %f]", vel_sp.x, vel_sp.y, vel_sp.z);
  }
}
}

namespace freertos {

void task_pose_control_init() {
  configASSERT(xTaskCreate(task_impl, "task_pose_control", 128 * 4, NULL,
                           osPriorityNormal, &task_handle) == pdPASS);
}
}  // namespace freertos
