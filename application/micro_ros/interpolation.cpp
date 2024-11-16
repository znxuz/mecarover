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

#include <application/mrcpptypes.hpp>
#include <application/robot_params.hpp>
#include <cmath>

#include "ctrl_utils.hpp"
#include "drive_state_wrapper.hpp"
#include "rcl_ret_check.hpp"

using namespace imsl;
using namespace robot_params;

static constexpr uint8_t N_EXEC_HANDLES = 3;
static constexpr uint16_t TIMER_TIMEOUT_MS =
    UROS_FREQ_MOD_INTERPOLATION_SEC * S_TO_MS;

extern "C" {
static rclc_executor_t interpolation_exe;

static rcl_subscription_t sub_cmd_vel;
static geometry_msgs__msg__Twist msg_cmd_vel{};

static rcl_subscription_t sub_odometry;
static geometry_msgs__msg__Pose2D msg_odom_pose{};

static auto interpolation_timer = rcl_get_zero_initialized_timer();
static rcl_publisher_t pub_wheel_vel;

static auto vel_rf_target = vPose<real_t>{};
static auto pose_actual = Pose<real_t>{};

static void cmd_vel_cb(const void* arg) {
  const auto* msg = reinterpret_cast<const geometry_msgs__msg__Twist*>(arg);
  vel_rf_target.vx = msg->linear.x * 1000;  // m/s -> mm/s
  vel_rf_target.vy = msg->linear.y * 1000;  // m/s -> mm/s
  vel_rf_target.omega = msg->angular.z;     // rad/s
}

static void pose_cb(const void* arg) {
  const auto* msg = reinterpret_cast<const geometry_msgs__msg__Pose2D*>(arg);
  pose_actual = {msg->x, msg->y, msg->theta};
}

static constexpr vPose<real_t> velocity_smoothen(
    const vPose<real_t>& vel_target, const vPose<real_t>& vel_cur) {
  constexpr auto FACTOR = 4;
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
                              const Pose<real_t>& pose_cur) {
  static constexpr real_t K_P = 0.17;

  const auto err = pose_sp - pose_cur;
  using std::abs;
  if (abs(err.x) > MAX_LINEAR_DEVIATION || abs(err.y) > MAX_LINEAR_DEVIATION ||
      abs(err.theta) > MAX_ANGULAR_DEVIATION) [[unlikely]]
    ULOG_WARNING("[intrpl]: sanity check: pose deviation too large");

  ULOG_DEBUG("[intrpl]: delta pose: [x: %.2f, y: %.2f, theta: %.2f]", err.x,
             err.y, static_cast<real_t>(err.theta));

  return err * K_P;
}

static void interpolation_cb(rcl_timer_t*, int64_t last_call_time) {
  static auto vel_prev = vPose<real_t>{};
  static auto pose_sp = Pose<real_t>{};
  const auto dt = RCL_NS_TO_S(static_cast<real_t>(last_call_time));

  const auto vel_rf_sp = velocity_smoothen(vel_rf_target, vel_prev);
  vel_prev = vel_rf_sp;

  pose_sp += Pose<real_t>(vRF2vWF(vel_rf_sp, pose_actual.theta) * dt);
  const auto d_vel_wf = vPose<real_t>(pose_ctrl(pose_sp, pose_actual) / dt);

  const auto vel_rf_corrected =
      vel_rf_sp + vWF2vRF(d_vel_wf, pose_actual.theta);
  const auto msg_vel_wheel_sp =
      DriveStateWrapper<DriveStateType::VEL_SP_ANGULAR>{
          vRF2vWheel(VelRF{vel_rf_corrected.vx, vel_rf_corrected.vy,
                           vel_rf_corrected.omega, 0})};
  rcl_ret_softcheck(rcl_publish(&pub_wheel_vel, &msg_vel_wheel_sp.state, NULL));
}

rclc_executor_t* interpolation_init(rcl_node_t* node, rclc_support_t* support,
                                    const rcl_allocator_t* allocator) {
  rcl_ret_check(rclc_executor_init(&interpolation_exe, &support->context,
                                   N_EXEC_HANDLES, allocator));

  rcl_ret_check(rclc_subscription_init_best_effort(
      &sub_cmd_vel, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  rcl_ret_check(rclc_executor_add_subscription(&interpolation_exe, &sub_cmd_vel,
                                               &msg_cmd_vel, &cmd_vel_cb,
                                               ON_NEW_DATA));

  rcl_ret_check(rclc_subscription_init_best_effort(
      &sub_odometry, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), "odometry"));
  rcl_ret_check(rclc_executor_add_subscription(&interpolation_exe,
                                               &sub_odometry, &msg_odom_pose,
                                               &pose_cb, ON_NEW_DATA));

  rcl_ret_check(rclc_timer_init_default2(&interpolation_timer, support,
                                         RCL_MS_TO_NS(TIMER_TIMEOUT_MS),
                                         &interpolation_cb, true));
  rcl_ret_check(
      rclc_executor_add_timer(&interpolation_exe, &interpolation_timer));

  rcl_ret_check(rclc_publisher_init_best_effort(
      &pub_wheel_vel, node,
      DriveStateWrapper<DriveStateType::VEL_SP_ANGULAR>::get_msg_type_support(),
      "wheel_vel"));

  return &interpolation_exe;
}
}
