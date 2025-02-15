#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <task.h>
#include <ulog.h>

#include <application/robot_params.hpp>
#include <application/pose_types.hpp>

#include "shared.hpp"
#include "vel2d_frame.hpp"

using namespace robot_params;
using namespace imsl;

static TaskHandle_t task_handle;
static Vel2d vel_sp;
static Pose pose_sp;
static Pose odom;

extern "C" {

static void task_impl(void*) {
  const TickType_t xFrequency = pdMS_TO_TICKS(POSE_CTRL_PERIOD_MS.count());
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    xQueueReceive(freertos::vel_sp_queue, &vel_sp, 0);
    xQueueReceive(freertos::odom_queue, &odom, 0);

    ULOG_INFO("pose_ctrl: vel [%f %f %f]", vel_sp.x, vel_sp.y, vel_sp.z);
    ULOG_INFO("pose_ctrl: odom [%.2f, %.2f, %.2f]", odom.x, odom.y, odom.theta);

    pose_sp += Pose();

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
}

namespace freertos {

void task_pose_control_init() {
  configASSERT(xTaskCreate(task_impl, "task_pose_control", 128 * 4, NULL,
                           osPriorityNormal, &task_handle) == pdPASS);
}
}  // namespace freertos
