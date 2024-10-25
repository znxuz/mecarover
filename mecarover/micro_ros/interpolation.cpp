#include "interpolation.hpp"

#include <geometry_msgs/msg/pose2_d.h>
#include <geometry_msgs/msg/twist.h>
#include <mecarover/mrlogger/mrlogger.h>
#include <mecarover/mrtypes.h>
#include <rcl/allocator.h>
#include <rcl/node.h>
#include <rclc/executor.h>
#include <rclc/executor_handle.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <rclc/timer.h>
#include <rclc/types.h>

#include "WheelDataWrapper.hpp"
#include "ctrl_utils.hpp"
#include "mecarover/mrcpptypes.hpp"
#include "rcl_ret_check.hpp"

using namespace imsl;

static constexpr uint8_t N_EXEC_HANDLES = 2;

extern "C" {
static rclc_executor_t interpolation_exe;

static rcl_subscription_t sub_odometry;
static geometry_msgs__msg__Pose2D odometry_msg_buf{};

static rcl_subscription_t sub_cmd_vel;
static geometry_msgs__msg__Twist cmd_vel_msg_buf{};
static rcl_publisher_t pub_wheel_vel;

static auto pose_sp = Pose<real_t>{};
static auto pose_cur = Pose<real_t>{};
static auto vel_rf_target = vPose<real_t>{};
static auto vel_rf_sp = vPose<real_t>{};
static auto vel_wf_sp = vPose<real_t>{};

static real_t freq = 0.5;  // FIXME: hardcoded frequency for testing

static void cmd_vel_cb(const void* arg) {
  static vPose<real_t> vel_prev{};

  if (arg) {
    vel_rf_target.vx = cmd_vel_msg_buf.linear.x * 1000.0;  // m/s -> mm/s
    vel_rf_target.vy = cmd_vel_msg_buf.linear.y * 1000.0;  // m/s -> mm/s
    vel_rf_target.omega = cmd_vel_msg_buf.angular.z;       // rad/s
  }

  vel_rf_sp = velocity_smoothen(vel_rf_target, vel_prev);
  vel_prev = vel_rf_sp;

  log_message(log_debug,
              "[interpolation]: vel_rf current triggered from cmd_vel: [%.02f, "
              "%.02f, %.02f]",
              vel_rf_sp.vx, vel_rf_sp.vy, vel_rf_sp.omega);

  vel_wf_sp = vRF2vWF(vel_rf_sp, pose_cur.theta);
  // pose_sp.x += vel_wf_sp.vx * sampling_times.dt_pose_ctrl;
  // pose_sp.y += vel_wf_sp.vy * sampling_times.dt_pose_ctrl;
  // pose_sp.theta += vel_wf_sp.omega * sampling_times.dt_pose_ctrl;
  pose_sp.x += vel_wf_sp.vx * freq;
  pose_sp.y += vel_wf_sp.vy * freq;
  pose_sp.theta += vel_wf_sp.omega * freq;
}

static bool sanity_check(const Pose<real_t>& dpose) {
  using std::abs;
  if (abs(dpose.x) > ctrl_params.LageSchleppMax.x ||
      abs(dpose.y) > ctrl_params.LageSchleppMax.y ||
      abs(dpose.theta) > ctrl_params.LageSchleppMax.theta) [[unlikely]] {
    log_message(log_error, "pose deviation too large");
    log_message(log_error, "sp: [x: %f, y: %f, theta: %f]", pose_sp.x,
                pose_sp.y, static_cast<real_t>(pose_sp.theta));
    log_message(log_error, "cur: [x: %f, y: %f, theta: %f]", pose_cur.x,
                pose_cur.y, static_cast<real_t>(pose_cur.theta));
    return false;
  }
  return true;
}

static void interpolation_cb(const void* arg) {
  pose_cur = {odometry_msg_buf.x, odometry_msg_buf.y, odometry_msg_buf.theta};

  auto dpose = pose_sp - pose_cur;
  // if (!sanity_check(dpose)) {
  //   // TODO: termnate - maybe via a global flag for uros superloop
  //   return;
  // }

  vPose<real_t> d_vel = {dpose.x / freq * ctrl_params.LageKv.x,
                         dpose.y / freq * ctrl_params.LageKv.y,
                         dpose.theta / freq * ctrl_params.LageKv.theta};
  auto vel_rf_corrected = vWF2vRF(vel_wf_sp + d_vel, pose_cur.theta);

  auto vel_wheel_sp_mtx = vRF2vWheel(VelRF{
      vel_rf_corrected.vx, vel_rf_corrected.vy, vel_rf_corrected.omega, 0});
  auto wheel_sp_msg =
      WheelDataWrapper<real_t, WheelDataType::VEL_SP>{vel_wheel_sp_mtx};
  log_message(log_debug,
              "[interpolation]: pub vel_wheel_sp triggered from odometry: "
              "[%.02f, %.02f, %.02f, %.02f]",
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
                                               &cmd_vel_msg_buf, &cmd_vel_cb,
                                               ALWAYS));

  rcl_ret_check(rclc_subscription_init_default(
      &sub_odometry, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), "odometry"));
  rcl_ret_check(rclc_executor_add_subscription(&interpolation_exe,
                                               &sub_odometry, &odometry_msg_buf,
                                               &interpolation_cb, ALWAYS));

  rcl_ret_check(rclc_publisher_init_default(
      &pub_wheel_vel, node,
      WheelDataWrapper<real_t, WheelDataType::VEL_SP>::get_msg_type_support(),
      "wheel_vel"));

  /*
   * trigger the executor callback chain as long as any of the needed values of
   * the callbacks becomes available
   */
  rcl_ret_check(rclc_executor_set_trigger(&interpolation_exe,
                                          rclc_executor_trigger_any, NULL));

  return &interpolation_exe;
}
}
