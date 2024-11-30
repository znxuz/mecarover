#include "interpolation.hpp"

#include <application/real_t.h>
#include <geometry_msgs/msg/pose2_d.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl/allocator.h>
#include <rcl/node.h>
#include <rcl/time.h>
#include <rclc/executor.h>
#include <rclc/executor_handle.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <rclc/timer.h>
#include <rclc/types.h>
#include <ulog.h>

#include <application/pose_types.hpp>
#include <application/robot_params.hpp>
#include <cmath>

#include "drive_state_wrapper.hpp"
#include "jacobi_transformation.hpp"
#include "rcl_guard.hpp"

using namespace imsl;
using namespace robot_params;

static constexpr uint8_t N_EXEC_HANDLES = 3;
static constexpr uint16_t TIMER_TIMEOUT_MS =
    UROS_FREQ_MOD_INTERPOLATION_SEC * S_TO_MS;

extern "C" {
static rclc_executor_t exe;

static rcl_subscription_t sub_cmd_vel;
static geometry_msgs__msg__Twist msg_cmd_vel{};

static rcl_subscription_t sub_odometry;
static geometry_msgs__msg__Pose2D msg_odom_pose{};

static auto timer = rcl_get_zero_initialized_timer();
static rcl_publisher_t pub_wheel_vel;

static auto vel_rf_target = vPose<real_t>{};
static auto pose_actual = Pose<real_t>{};

real_t epsilon = 0;

static void cmd_vel_cb(const void* arg) {
  const auto* msg = reinterpret_cast<const geometry_msgs__msg__Twist*>(arg);
  vel_rf_target.vx = msg->linear.x * S_TO_MS;
  vel_rf_target.vy = msg->linear.y * S_TO_MS;
  vel_rf_target.omega = msg->angular.z;  // rad/s
}

static void pose_cb(const void* arg) {
  const auto* msg = reinterpret_cast<const geometry_msgs__msg__Pose2D*>(arg);
  pose_actual = {msg->x, msg->y, msg->theta};
}

static constexpr vPose<real_t> velocity_smoothen(
    const vPose<real_t>& vel_target, const vPose<real_t>& vel_cur) {
  constexpr auto FACTOR = 2;
  constexpr auto MAX_DIFF_LINEAR =
      MAX_VELOCITY_WHEEL_LINEAR * UROS_FREQ_MOD_INTERPOLATION_SEC / FACTOR;
  constexpr auto MAX_DIFF_ANGULAR = MAX_DIFF_LINEAR / L_W_HALF;

  auto vel_diff = vel_target - vel_cur;
  using std::clamp;
  vel_diff.vx = clamp(vel_diff.vx, -MAX_DIFF_LINEAR, MAX_DIFF_LINEAR);
  vel_diff.vy = clamp(vel_diff.vy, -MAX_DIFF_LINEAR, MAX_DIFF_LINEAR);
  vel_diff.omega = clamp(vel_diff.omega, -MAX_DIFF_ANGULAR, MAX_DIFF_ANGULAR);

  return vel_cur + vel_diff;
}

static Pose<real_t> pose_ctrl(const Pose<real_t>& pose_sp,
                              const Pose<real_t>& pose_cur, real_t dt) {
  static constexpr real_t K_P = 0.02;
  static constexpr real_t K_I = 0.01;
  static constexpr real_t K_D = 0.01;
  static auto integral = Pose<real_t>{}, prev_err = Pose<real_t>{};
  static constexpr real_t MAX_INTEGRAL_LINEAR = 200;
  static constexpr real_t MAX_INTEGRAL_ANGULAR = M_PI;

  const auto err = pose_sp - pose_cur;
  using std::abs;
  if (abs(err.x) > MAX_POSE_DEVIATION_LINEAR ||
      abs(err.y) > MAX_POSE_DEVIATION_LINEAR ||
      abs(err.theta) > MAX_POSE_DEVIATION_ANGULAR) [[unlikely]]
    ULOG_WARNING("[intrpl]: sanity check: pose deviation too large");
  ULOG_DEBUG("[intrpl]: delta: [x: %.2f, y: %.2f, theta: %.2f]", err.x, err.y,
             static_cast<real_t>(err.theta));

  integral += err * dt;
  integral.x =
      std::clamp(integral.x, -MAX_INTEGRAL_LINEAR, MAX_INTEGRAL_LINEAR);
  integral.y =
      std::clamp(integral.y, -MAX_INTEGRAL_LINEAR, MAX_INTEGRAL_LINEAR);
  integral.theta = std::clamp(static_cast<real_t>(integral.theta),
                              -MAX_INTEGRAL_ANGULAR, MAX_INTEGRAL_ANGULAR);
  const auto err_diff = err - prev_err;

  return err * K_P + integral * K_I + err_diff / dt * K_D;
}

static void interpolation_cb(rcl_timer_t*, int64_t last_call_time) {
  static auto vel_prev = vPose<real_t>{};
  static auto pose_sp = Pose<real_t>{};
  const auto dt = RCL_NS_TO_S(static_cast<real_t>(last_call_time));

  const auto vel_rf_sp = velocity_smoothen(vel_rf_target, vel_prev);
  vel_prev = vel_rf_sp;

  pose_sp += Pose<real_t>(vRF2vWF(vel_rf_sp, pose_sp.theta) * dt);

  const auto d_vel_wf = vPose<real_t>(pose_ctrl(pose_sp, pose_actual, dt) / dt);
  const auto vel_rf_corrected =
      vel_rf_sp + vWF2vRF(d_vel_wf, pose_actual.theta);
  const auto msg_vel_wheel_sp =
      DriveStateWrapper<DriveStateType::VEL_SP_ANGULAR>{
          backward_transform(VelRF{vel_rf_corrected.vx, vel_rf_corrected.vy,
                                   vel_rf_corrected.omega, epsilon * 0.2})};
  rcl_softguard(rcl_publish(&pub_wheel_vel, &msg_vel_wheel_sp.state, NULL));
}

rclc_executor_t* interpolation_init(rcl_node_t* node, rclc_support_t* support,
                                    const rcl_allocator_t* allocator) {
  rcl_guard(
      rclc_executor_init(&exe, &support->context, N_EXEC_HANDLES, allocator));

  rcl_guard(rclc_subscription_init_best_effort(
      &sub_cmd_vel, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  rcl_guard(rclc_executor_add_subscription(&exe, &sub_cmd_vel, &msg_cmd_vel,
                                               &cmd_vel_cb, ON_NEW_DATA));

  rcl_guard(rclc_subscription_init_best_effort(
      &sub_odometry, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), "odometry"));
  rcl_guard(rclc_executor_add_subscription(
      &exe, &sub_odometry, &msg_odom_pose, &pose_cb, ON_NEW_DATA));

  rcl_guard(rclc_timer_init_default2(&timer, support,
                                         RCL_MS_TO_NS(TIMER_TIMEOUT_MS),
                                         &interpolation_cb, true));
  rcl_guard(rclc_executor_add_timer(&exe, &timer));

  rcl_guard(rclc_publisher_init_best_effort(
      &pub_wheel_vel, node,
      DriveStateWrapper<DriveStateType::VEL_SP_ANGULAR>::get_msg_type_support(),
      "wheel_vel"));

  rclc_executor_set_semantics(&exe, RCLC_SEMANTICS_LOGICAL_EXECUTION_TIME);

  return &exe;
}
}
