#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <task.h>
#include <ulog.h>

#include <application/hal/hal.hpp>

#include "four_wheel_data.hpp"
#include "shared.hpp"

static TaskHandle_t task_handle;

using namespace robot_params;

using freertos::FourWheelData;
using freertos::FourWheelDataType;

extern "C" {

static void task_impl(void*) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(WHEEL_CTRL_PERIOD_MS.count());

  while (true) {
    auto enc_delta = FourWheelData(hal_encoder_delta_rad(),
                                   FourWheelDataType::ENC_DELTA_RAD);
    xQueueSend(freertos::enc_delta_wheel_ctrl_queue, &enc_delta, 0);
    xQueueOverwrite(freertos::enc_delta_odom_queue, &enc_delta);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
}

namespace freertos {

void task_get_enc_delta_init() {
  configASSERT(xTaskCreate(task_impl, "task_pose_control", 128 * 4, NULL,
                           osPriorityNormal, &task_handle) == pdPASS);
}
}  // namespace freertos
