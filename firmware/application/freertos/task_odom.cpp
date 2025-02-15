#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <queue.h>
#include <ulog.h>

#include <application/hal/hal.hpp>
#include <application/pose_types.hpp>
#include <application/robot_params.hpp>
#include <application/transformations.hpp>

#include "four_wheel_data.hpp"
#include "shared.hpp"

using namespace imsl;
using namespace robot_params;

static TaskHandle_t task_handle;
static Pose odom;

extern "C" {

static void task_impl(void*) {
  const TickType_t xFrequency = pdMS_TO_TICKS(POSE_CTRL_PERIOD_MS.count());
  TickType_t xLastWakeTime = xTaskGetTickCount();

  freertos::FourWheelData enc_delta;
  while (true) {
    xQueueReceive(freertos::enc_delta_odom_queue, &enc_delta, 0);

    const auto dpose_rf_mtx = transform::forward_transform(
        VelWheel(enc_delta.front_right, enc_delta.front_left,
                 enc_delta.back_left, enc_delta.back_right));
    const auto dpose_rf =
        Pose{dpose_rf_mtx(0), dpose_rf_mtx(1), dpose_rf_mtx(2)};

    odom += pRF2pWF(dpose_rf,
                    (static_cast<double>(odom.theta) * 2 + dpose_rf.theta) / 2);
    // TODO do epsilon

    xQueueSend(freertos::odom_queue, &odom, 0);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
}

namespace freertos {

void task_odom_init() {
  configASSERT(xTaskCreate(task_impl, "task_odometry", 128 * 4, NULL,
                           osPriorityNormal, &task_handle) == pdPASS);
}

}  // namespace freertos
