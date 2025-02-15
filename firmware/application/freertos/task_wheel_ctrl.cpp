#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <queue.h>
#include <task.h>
#include <ulog.h>

#include <application/robot_params.hpp>

#include "four_wheel_data.hpp"
#include "shared.hpp"

using namespace freertos;

TaskHandle_t task_handle;

static FourWheelData vel_wheel;
static FourWheelData enc_delta;

extern "C" {
void task_impl(void*) {
  constexpr auto dt = robot_params::WHEEL_CTRL_PERIOD_S.count();
  const TickType_t xFrequency = pdMS_TO_TICKS(dt * 1000);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    xQueueReceive(vel_wheel_queue, &vel_wheel, 0);
    xQueueReceive(enc_delta_wheel_ctrl_queue, &enc_delta, 0);

    ULOG_INFO("[wheel_ctrl] vel: [%.2f, %.2f, %.2f, %.2f]",
              vel_wheel.front_right, vel_wheel.front_left, vel_wheel.back_left,
              vel_wheel.back_right);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
}

namespace freertos {

void task_wheel_ctrl_init() {
  configASSERT(xTaskCreate(task_impl, "task_pose_control", 128 * 4, NULL,
                           osPriorityNormal, &task_handle) == pdPASS);
}
}  // namespace freertos
