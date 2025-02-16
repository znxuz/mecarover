#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <queue.h>
#include <task.h>
#include <ulog.h>

#include <algorithm>
#include <application/hal/hal.hpp>
#include <application/robot_params.hpp>
#include <utility>

#include "four_wheel_data.hpp"
#include "shared.hpp"

using namespace freertos;
using namespace robot_params;

TaskHandle_t task_handle;

static FourWheelData vel_wheel_buf;
static FourWheelData enc_delta_buf;

static std::array<double, N_WHEEL> vel_to_duty_cycle(const VelWheel& vel) {
  static constexpr double PERCENT = 100.0;
  auto ret = std::array<double, N_WHEEL>{};

  std::transform(vel.data(), vel.data() + vel.size(), begin(ret),
                 [](double val) {
                   return std::clamp(val / MAX_VELOCITY_WHEEL_ANGULAR * PERCENT,
                                     -PERCENT, PERCENT);
                 });
  return ret;
}

static VelWheel pid_ctrl(const VelWheel& vel_wheel_sp,
                         const VelWheel& vel_wheel_pv, const double dt) {
  static constexpr double K_P = 0.025, K_I = 0.015, K_D = 0, MAX_INTEGRAL = 10;
  static VelWheel integral = VelWheel::Zero(), prev_err = VelWheel::Zero();

  auto err = vel_wheel_sp - vel_wheel_pv;
  ULOG_WARNING("[wheel_ctrl]: wheel vel err: [%0.2f, %.02f, %.02f, %.02f]",
               err(0), err(1), err(2), err(3));

  integral += err * dt;
  integral = integral.unaryExpr(
      [&](double val) { return std::clamp(val, -MAX_INTEGRAL, MAX_INTEGRAL); });
  if (std::any_of(std::begin(integral), std::end(integral),
                  [](double val) { return val >= 0.8 * MAX_INTEGRAL; }))
    ULOG_WARNING("[wheel_ctrl]: PID integral: [%0.2f, %.02f, %.02f, %.02f]",
                 integral(0), integral(1), integral(2), integral(3));

  VelWheel derivative = (err - std::exchange(prev_err, err)) / dt;  // unused

  return vel_wheel_sp + K_P * err + K_I * integral + K_D * derivative;
}

extern "C" {
void task_impl(void*) {
  constexpr auto dt = robot_params::WHEEL_CTRL_PERIOD_S.count();
  const TickType_t xFrequency = pdMS_TO_TICKS(dt * 1000);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    xQueueReceive(vel_wheel_queue, &vel_wheel_buf, 0);
    xQueueReceive(enc_delta_wheel_ctrl_queue, &enc_delta_buf, 0);

    VelWheel vel_wheel_sp = vel_wheel_buf;
    VelWheel vel_wheel_pv = enc_delta_buf;
    vel_wheel_pv /= dt;

    auto vel_cv = pid_ctrl(vel_wheel_sp, vel_wheel_pv, dt);
    ULOG_DEBUG("[wheel_ctrl] vel_cv: [%.02f, %.02f, %.02f, %.02f]", vel_cv(0),
               vel_cv(1), vel_cv(2), vel_cv(3));
    auto duty_cycle = vel_to_duty_cycle(vel_cv);
    ULOG_DEBUG("[wheel_ctrl] duty: [%.02f, %.02f, %.02f, %.02f]", duty_cycle[0],
               duty_cycle[1], duty_cycle[2], duty_cycle[3]);
    hal_wheel_vel_set_pwm(duty_cycle);

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
