#pragma once

#include <FreeRTOS.h>
#include <queue.h>

namespace freertos {

void task_vel_recv_init(QueueHandle_t vel_sp_queue_out);

}
