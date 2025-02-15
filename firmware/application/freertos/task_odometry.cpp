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
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);

  while (true) {
    // FourWheelData enc_delta;
    // xQueueReceive(enc_delta_odom_queue, &enc_delta, 0);
    //
    // const auto dpose_rf_mtx =
    //     forward_transform(VelWheel(enc_delta.front_right,
    //     enc_delta.front_left,
    //                                enc_delta.back_left,
    //                                enc_delta.back_right));
    // const auto dpose_rf =
    //     Pose{dpose_rf_mtx(0), dpose_rf_mtx(1), dpose_rf_mtx(2)};
    //
    // pose_wf +=
    //     pRF2pWF(dpose_rf,
    //             (static_cast<double>(pose_wf.theta) * 2 + dpose_rf.theta) /
    //             2);
    //
    ULOG_INFO("odom");
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
