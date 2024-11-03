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

#include <cmath>
#include <mecarover/mrcpptypes.hpp>
#include <mecarover/robot_params.hpp>

#include "WheelDataWrapper.hpp"
#include "ctrl_utils.hpp"
#include "geometry_msgs/msg/pose2_d.h"
#include "geometry_msgs/msg/twist.h"
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

static void cmd_vel_cb(const void* arg) {
  const auto* msg = reinterpret_cast<const geometry_msgs__msg__Twist*>(arg);
  vel_rf_target.vx = msg->linear.x * 1000.0;  // m/s -> mm/s
  vel_rf_target.vy = msg->linear.y * 1000.0;  // m/s -> mm/s
  vel_rf_target.omega = msg->angular.z;       // rad/s
}

static void pose_cb(const void* arg) {
  const auto* msg = reinterpret_cast<const geometry_msgs__msg__Pose2D*>(arg);
  pose_cur = {msg->x, msg->y, msg->theta};
}

static void sanity_check(const Pose<real_t>& dpose) {
  static constexpr real_t LINEAR_DEVIATION_MAX = 100;
  static constexpr real_t ANGULAR_DEVIATION_MAX = M_PI;

  using std::abs;
  if (abs(dpose.x) > LINEAR_DEVIATION_MAX ||
      abs(dpose.y) > LINEAR_DEVIATION_MAX ||
      abs(dpose.theta) > ANGULAR_DEVIATION_MAX) [[unlikely]] {
    ULOG_ERROR("[pose sanity check]: pose deviation too large");
  }
}

static void interpolation_cb(rcl_timer_t*, int64_t last_call_time) {
  static auto vel_prev = vPose<real_t>{};
  static auto pose_sp = Pose<real_t>{};
  const auto dt = RCL_NS_TO_S(static_cast<real_t>(last_call_time));

  auto vel_rf_sp = velocity_smoothen(vel_rf_target, vel_prev);
  vel_prev = vel_rf_sp;
  // ULOG_DEBUG("%s: [%.02f, %.02f, %.02f]",
  //            "[intrpl]: vel_rf(smoothened) from cmd_vel", vel_rf_sp.vx,
  //            vel_rf_sp.vy, vel_rf_sp.omega);

  pose_sp +=
      Pose<real_t>(vRF2vWF(vel_rf_sp, pose_cur.theta) * dt);
  // ULOG_DEBUG("%s: [x: %.2f, y: %.2f, theta: %.2f]",
  //            "[intrpl]: cumulated pose_sp from vel_rf", pose_sp.x,
  //            pose_sp.y, static_cast<real_t>(pose_sp.theta));

  // TODO PID control this shit, rn k_p (alone) is too sensitive to noise

  auto dpose = pose_sp - pose_cur;
  sanity_check(dpose);

  ULOG_DEBUG("[intrpl]: delta pose: [x: %.2f, y: %.2f, theta: %.2f]", dpose.x,
             dpose.y, static_cast<real_t>(dpose.theta));

  static constexpr real_t k_v = 0.35;

  // TODO: refactor operator overloads and here
  // vPose<real_t> d_vel_wf = {dpose.x / dt * k_v, dpose.y / dt * k_v,
  //                           dpose.theta / dt * k_v};
  vPose<real_t> d_vel_wf = vPose<real_t>(dpose / dt) * k_v;

  auto vel_rf_corrected = vel_rf_sp + vWF2vRF(d_vel_wf, pose_cur.theta);
  // ULOG_DEBUG("%s: [dx: %.2f, dy: %.2f, omega: %.2f]",
  //            "[intrpl]: (corrected) vel_rf_sp", vel_rf_corrected.vx,
  //            vel_rf_corrected.vy, vel_rf_corrected.omega);

  auto msg_vel_wheel_sp = WheelDataWrapper<real_t, WheelDataType::VEL_SP>{
      vRF2vWheel(VelRF{vel_rf_corrected.vx, vel_rf_corrected.vy,
                       vel_rf_corrected.omega, 0})};
  // ULOG_DEBUG("%s: [%.02f, %.02f, %.02f, %.02f]",
  //            "[intrpl]: pub vel_wheel_sp from vel_rf_sp",
  //            msg_vel_wheel_sp[0], msg_vel_wheel_sp[1], msg_vel_wheel_sp[2],
  //            msg_vel_wheel_sp[3]);

  rcl_ret_softcheck(rcl_publish(&pub_wheel_vel, &msg_vel_wheel_sp.msg, NULL));
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
