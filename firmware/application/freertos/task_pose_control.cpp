#include "task_pose_control.hpp"

#include <cmsis_os2.h>
#include <ulog.h>

#include "vel2d_frame.hpp"

static TaskHandle_t task_handle;
static QueueHandle_t vel_sp_queue;
static QueueHandle_t odom_queue;
static QueueHandle_t wheel_vel_queue;
static Vel2d vel_sp;

extern "C" {

void task_impl(void*) {
  while (true) {
    xQueueReceive(vel_sp_queue, &vel_sp, portMAX_DELAY);
    ULOG_INFO("vel from queue [%f %f %f]", vel_sp.x, vel_sp.y,
              vel_sp.z);
  }
}
}

namespace freertos {

void task_pose_control_init(QueueHandle_t vel_sp_queue_in,
                            QueueHandle_t odom_queue_in,
                            QueueHandle_t wheel_vel_queue_out) {
  vel_sp_queue = vel_sp_queue_in;
  odom_queue = odom_queue_in;
  wheel_vel_queue = wheel_vel_queue_out;

  configASSERT(xTaskCreate(task_impl, "task_pose_control", 128 * 4, NULL,
                           osPriorityNormal, &task_handle) == pdPASS);
}
}  // namespace freertos
