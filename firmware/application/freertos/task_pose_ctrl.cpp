#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <task.h>
#include <ulog.h>

#include <application/pose_types.hpp>
#include <application/robot_params.hpp>
#include <application/transformations.hpp>
#include <utility>

#include "four_wheel_data.hpp"
#include "shared.hpp"

using namespace robot_params;
using namespace imsl;

static TaskHandle_t task_handle;

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

static Pose pose_ctrl(const Pose& pose_sp, const Pose& pose_cur, double dt) {
  static constexpr double MAX_INTEGRAL_LINEAR = 200;
  static constexpr double MAX_INTEGRAL_ANGULAR = M_PI;
  static constexpr double MAX_POSE_DEVIATION_LINEAR = 300;
  static constexpr double MAX_POSE_DEVIATION_ANGULAR = pi;
  static constexpr double K_P = 0.07;
  static constexpr double K_I = 0.001;
  static constexpr double K_D = 0.00001;
  static auto integral = Pose{}, prev_err = Pose{};

  const auto err = pose_sp - pose_cur;
  using std::abs;
  if (abs(err.x) > MAX_POSE_DEVIATION_LINEAR ||
      abs(err.y) > MAX_POSE_DEVIATION_LINEAR ||
      abs(err.theta) > MAX_POSE_DEVIATION_ANGULAR) [[unlikely]]
    ULOG_WARNING("[pose_ctrl] sanity check: pose deviation too large");
  // ULOG_DEBUG("[pose_ctrl] delta: [x: %.2f, y: %.2f, theta: %.2f]", err.x, err.y,
  //            static_cast<double>(err.theta));

  integral += err * dt;
  integral.x =
      std::clamp(integral.x, -MAX_INTEGRAL_LINEAR, MAX_INTEGRAL_LINEAR);
  integral.y =
      std::clamp(integral.y, -MAX_INTEGRAL_LINEAR, MAX_INTEGRAL_LINEAR);
  integral.theta = std::clamp(static_cast<double>(integral.theta),
                              -MAX_INTEGRAL_ANGULAR, MAX_INTEGRAL_ANGULAR);

  const auto derivative = err - std::exchange(prev_err, err);

  return err * K_P + integral * K_I + derivative / dt * K_D;
}

extern "C" {

static void task_impl(void*) {
  const TickType_t xFrequency = pdMS_TO_TICKS(POSE_CTRL_PERIOD_MS.count());
  constexpr auto dt = POSE_CTRL_PERIOD_MS.count() / 1000.0;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  vPose rvel_target;
  vPose rvel_prev;
  Pose pose_sp;
  Pose odom;

  while (true) {
    xQueueReceive(freertos::vel_sp_queue, &rvel_target, 0);
    xQueueReceive(freertos::odom_queue, &odom, 0);

    auto rvel_sp = velocity_smoothen(rvel_target, rvel_prev);
    rvel_prev = rvel_sp;

    pose_sp += Pose(vRF2vWF(rvel_sp, pose_sp.theta) * dt);

    // TODO epsilon
    constexpr double K_eps = -0.2;
    const auto eps = 0;

    const auto rvel_cv =
        rvel_sp + vWF2vRF(vPose(pose_ctrl(pose_sp, odom, dt) / dt), odom.theta);

    const auto vel_wheel =
        freertos::FourWheelData{transform::backward_transform(
            VelRF{rvel_cv.vx, rvel_cv.vy, rvel_cv.omega, eps * K_eps})};

    xQueueSend(freertos::vel_wheel_queue, &vel_wheel, 0);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
}

namespace freertos {

void task_pose_ctrl_init() {
  configASSERT(xTaskCreate(task_impl, "pose_control", 128 * 4, NULL,
                           osPriorityNormal, &task_handle) == pdPASS);
}
}  // namespace freertos
