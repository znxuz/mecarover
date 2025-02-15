#pragma once

#include <FreeRTOS.h>
#include <queue.h>

namespace freertos {

extern QueueHandle_t vel_sp_queue;
extern QueueHandle_t enc_delta_odom_queue;
extern QueueHandle_t enc_delta_wheel_ctrl_queue;
extern QueueHandle_t odom_queue;
extern QueueHandle_t vel_wheel_queue;

void queues_init();

}  // namespace freertos
