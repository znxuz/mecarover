#include "interpolation.hpp"

#include <geometry_msgs/msg/pose2_d.h>
#include <geometry_msgs/msg/twist.h>
#include <mecarover/mrtypes.h>
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

#include <mecarover/mrcpptypes.hpp>
#include <mecarover/robot_params.hpp>

#include "WheelDataWrapper.hpp"
#include "ctrl_utils.hpp"
#include "rcl_ret_check.hpp"

using namespace imsl;

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
static auto pose_cur = Pose<real_t>{};

static void cmd_vel_cb(const void*) {
  vel_rf_target.vx = msg_cmd_vel.linear.x * 1000.0;  // m/s -> mm/s
  vel_rf_target.vy = msg_cmd_vel.linear.y * 1000.0;  // m/s -> mm/s
  vel_rf_target.omega = msg_cmd_vel.angular.z;       // rad/s
}

static void pose_cb(const void*) {
  pose_cur = {msg_odom_pose.x, msg_odom_pose.y, msg_odom_pose.theta};
}

// static bool sanity_check(const Pose<real_t>& dpose) {
//   using std::abs;
//   if (abs(dpose.x) > ctrl_params.LageSchleppMax.x ||
//       abs(dpose.y) > ctrl_params.LageSchleppMax.y ||
//       abs(dpose.theta) > ctrl_params.LageSchleppMax.theta) [[unlikely]] {
//     ULOG_ERROR("[pose sanity check]: pose deviation too large");
//     return false;
//   }
//   return true;
// }

static void interpolation_cb(rcl_timer_t*, int64_t) {
  static auto vel_prev = vPose<real_t>{};
  static auto pose_sp = Pose<real_t>{};

  auto vel_rf_sp = velocity_smoothen(vel_rf_target, vel_prev);
  vel_prev = vel_rf_sp;
  ULOG_DEBUG("%s: [%.02f, %.02f, %.02f]",
             "[interpolation]: vel_rf(smoothened) from cmd_vel", vel_rf_sp.vx,
             vel_rf_sp.vy, vel_rf_sp.omega);
  pose_sp +=
      vRF2vWF(vel_rf_sp, pose_cur.theta) * UROS_FREQ_MOD_INTERPOLATION_SEC;
  ULOG_DEBUG("%s: [x: %.2f, y: %.2f, theta: %.2f]",
             "[interpolation]: cumulated pose_sp from vel_rf", pose_sp.x,
             pose_sp.y, static_cast<real_t>(pose_sp.theta));

  auto dpose = pose_sp - pose_cur;

  // sanity_check(dpose); // TODO

  ULOG_DEBUG("%s: [x: %.2f, y: %.2f, theta: %.2f]",
             "[interpolation]: delta pose:", dpose.x, dpose.y,
             static_cast<real_t>(dpose.theta));

  /* pose control */

  static constexpr real_t pose_kv = 0.1;

  // TODO: refactor operator overloads and here
  vPose<real_t> d_vel = {
      dpose.x / UROS_FREQ_MOD_INTERPOLATION_SEC * pose_kv,
      dpose.y / UROS_FREQ_MOD_INTERPOLATION_SEC * pose_kv,
      dpose.theta / UROS_FREQ_MOD_INTERPOLATION_SEC * pose_kv};
  auto vel_rf_corrected = vel_rf_sp + d_vel;
  ULOG_DEBUG("%s: [dx: %.2f, dy: %.2f, omega: %.2f]",
             "[interpolation]: (corrected) vel_rf_sp", vel_rf_corrected.vx,
             vel_rf_corrected.vy, vel_rf_corrected.omega);

  auto vel_wheel_sp_mtx = vRF2vWheel(VelRF{
      vel_rf_corrected.vx, vel_rf_corrected.vy, vel_rf_corrected.omega, 0});
  auto wheel_sp_msg =
      WheelDataWrapper<real_t, WheelDataType::VEL_SP>{vel_wheel_sp_mtx};
  ULOG_DEBUG("%s: [%.02f, %.02f, %.02f, %.02f]",
             "[interpolation]: pub vel_wheel_sp from vel_rf_sp",
             wheel_sp_msg[0], wheel_sp_msg[1], wheel_sp_msg[2],
             wheel_sp_msg[3]);

  rcl_ret_softcheck(rcl_publish(&pub_wheel_vel, &wheel_sp_msg.msg, NULL));
}

rclc_executor_t* interpolation_init(rcl_node_t* node, rclc_support_t* support,
                                    const rcl_allocator_t* allocator) {
  rcl_ret_check(rclc_executor_init(&interpolation_exe, &support->context,
                                   N_EXEC_HANDLES, allocator));

  rcl_ret_check(rclc_subscription_init_default(
      &sub_cmd_vel, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  rcl_ret_check(rclc_executor_add_subscription(&interpolation_exe, &sub_cmd_vel,
                                               &msg_cmd_vel, &cmd_vel_cb,
                                               ON_NEW_DATA));

  rcl_ret_check(rclc_subscription_init_default(
      &sub_odometry, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), "odometry"));
  rcl_ret_check(rclc_executor_add_subscription(&interpolation_exe,
                                               &sub_odometry, &msg_odom_pose,
                                               &pose_cb, ON_NEW_DATA));

  // FIXME: interpolation cb still has the same frequency
  rcl_ret_check(rclc_timer_init_default2(&interpolation_timer, support,
                                         RCL_MS_TO_NS(TIMER_TIMEOUT_MS),
                                         &interpolation_cb, true));
  rcl_ret_check(
      rclc_executor_add_timer(&interpolation_exe, &interpolation_timer));

  rcl_ret_check(rclc_publisher_init_default(
      &pub_wheel_vel, node,
      WheelDataWrapper<real_t, WheelDataType::VEL_SP>::get_msg_type_support(),
      "wheel_vel"));

  return &interpolation_exe;
}
}
