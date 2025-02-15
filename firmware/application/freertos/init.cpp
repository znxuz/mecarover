#include "init.hpp"

#include <FreeRTOS.h>
#include <queue.h>
#include <ulog.h>

#include <application/hal/hal.hpp>

#include "shared.hpp"

namespace freertos {

void task_odom_init();
void task_pose_ctrl_init();
void task_vel_recv_init();
void task_get_enc_delta_init();
void task_wheel_ctrl_init();

void init() {
  hal_init();
  queues_init();
  task_get_enc_delta_init();
  task_vel_recv_init();
  task_pose_ctrl_init();
  task_wheel_ctrl_init();
  task_odom_init();
}

}  // namespace freertos
