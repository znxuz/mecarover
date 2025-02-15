#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <task.h>
#include <ulog.h>

#include <application/robot_params.hpp>
#include <application/pose_types.hpp>

#include "application/freertos/vel2d_frame.hpp"
#include "shared.hpp"

using namespace robot_params;
using namespace imsl;

static TaskHandle_t task_handle;
static vPose vel_target;
static Pose pose_sp;
static Pose odom;

static constexpr vPose velocity_smoothen(const vPose& vel_target,
                                         const vPose& vel_cur) {
  constexpr auto MAX_DIFF_LINEAR = MAX_VELOCITY_WHEEL_LINEAR * 0.03;
  constexpr auto MAX_DIFF_ANGULAR = MAX_DIFF_LINEAR / L_W_HALF;

  auto vel_diff = vel_target - vel_cur;
  using std::clamp;
  vel_diff.vx = clamp(vel_diff.vx, -MAX_DIFF_LINEAR, MAX_DIFF_LINEAR);
  vel_diff.vy = clamp(vel_diff.vy, -MAX_DIFF_LINEAR, MAX_DIFF_LINEAR);
  vel_diff.omega = clamp(vel_diff.omega, -MAX_DIFF_ANGULAR, MAX_DIFF_ANGULAR);

  return vel_cur + vel_diff;
}

extern "C" {

static void task_impl(void*) {
  constexpr auto dt = POSE_CTRL_PERIOD_S.count();
  const TickType_t xFrequency = pdMS_TO_TICKS(dt * 1000);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // check for receiving Vel2d directly as vPose
  configASSERT(sizeof(vPose) == sizeof(Vel2d));

  auto vel_prev = vPose{};

  while (true) {
    xQueueReceive(freertos::vel_sp_queue, &vel_target, 0);
    xQueueReceive(freertos::odom_queue, &odom, 0);

    ULOG_INFO("pose_ctrl: vel [%f %f %f]", vel_target.vx, vel_target.vy,
              vel_target.omega);
    ULOG_INFO("pose_ctrl: odom [%.2f, %.2f, %.2f]", odom.x, odom.y, odom.theta);

    auto vel_sp = velocity_smoothen(vel_target, vel_prev);
    vel_prev = vel_sp;

    pose_sp += Pose(vRF2vWF(vel_sp, pose_sp.theta) * dt);

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
