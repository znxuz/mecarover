#include "init.hpp"

#include <FreeRTOS.h>
#include <queue.h>
#include <ulog.h>

#include <application/hal/hal.hpp>

#include "shared.hpp"

namespace freertos {

void task_odometry_init();
void task_pose_control_init();
void task_vel_recv_init();
void task_get_enc_delta_init();

void init() {
  ULOG_INFO("freertos init");

  hal_init();
  queues_init();
  task_get_enc_delta_init();
  task_vel_recv_init();
  task_pose_control_init();
  task_odometry_init();

  ULOG_INFO("freertos init finished");
}

}  // namespace freertos
