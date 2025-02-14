#pragma once

#include <FreeRTOS.h>
#include <queue.h>

namespace freertos {

void task_odometry_init(QueueHandle_t enc_delta_queue_out,
                        QueueHandle_t odom_queue_out);
}
