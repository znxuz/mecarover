#pragma once

#include <FreeRTOS.h>
#include <queue.h>

namespace freertos {

void task_pose_control_init(QueueHandle_t vel_sp_queue_in,
                       QueueHandle_t odometry_queue_in,
                       QueueHandle_t wheel_vel_queue_out);
}
