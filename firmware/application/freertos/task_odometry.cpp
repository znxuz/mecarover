#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <queue.h>
#include <ulog.h>

#include <application/hal/hal.hpp>
#include <application/jacobi_transformation.hpp>
#include <application/pose_types.hpp>
#include <application/robot_params.hpp>

#include "four_wheel_data.hpp"
#include "shared.hpp"

using namespace imsl;
using namespace robot_params;

static TaskHandle_t task_handle;
static Pose pose_wf;

extern "C" {

static void task_impl(void*) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency =
      pdMS_TO_TICKS(robot_params::POSE_CTRL_PERIOD_S * S_TO_MS);

  freertos::FourWheelData enc_delta;
  while (true) {
    xQueueReceive(freertos::enc_delta_odom_queue, &enc_delta, 0);

    const auto dpose_rf_mtx =
        forward_transform(VelWheel(enc_delta.front_right, enc_delta.front_left,
                                   enc_delta.back_left, enc_delta.back_right));
    const auto dpose_rf =
        Pose{dpose_rf_mtx(0), dpose_rf_mtx(1), dpose_rf_mtx(2)};

    // TODO rotate_z
    pose_wf +=
        pRF2pWF(dpose_rf,
                (static_cast<double>(pose_wf.theta) * 2 + dpose_rf.theta) / 2);
    // TODO do epsilon

    ULOG_INFO("pose wf: [%.2f, %.2f, %.2f]", pose_wf.x, pose_wf.y,
              pose_wf.theta);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
}

namespace freertos {

void task_odometry_init() {
  configASSERT(xTaskCreate(task_impl, "task_odometry", 128 * 4, NULL,
                           osPriorityNormal, &task_handle) == pdPASS);
}

}  // namespace freertos
