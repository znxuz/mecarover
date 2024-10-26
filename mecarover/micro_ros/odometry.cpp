#include "odometry.hpp"

#include <geometry_msgs/msg/pose2_d.h>
#include <mecarover/mrlogger/mrlogger.h>
#include <rclc/executor.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <rclc/types.h>

#include <mecarover/hal/stm_hal.hpp>
#include <mecarover/mrcpptypes.hpp>

#include "WheelDataWrapper.hpp"
#include "ctrl_utils.hpp"
#include "mecarover/robot_params.hpp"
#include "rcl_ret_check.hpp"

using namespace imsl;

static constexpr uint8_t N_EXEC_HANDLES = 1;

extern "C" {
static auto odometry_exe = rclc_executor_get_zero_initialized_executor();

static rcl_subscription_t sub_encoder_data;
static auto enc_msg_buf =
    WheelDataWrapper<real_t, WheelDataType::ENC_DELTA_RAD>{};
static Pose<real_t> pose;

static rcl_publisher_t pub_odometry;

/*
 * odometry: encoder delta gets feeded directly into the inverted jacobian
 * matrix without dividing the dt for the reason being:
 * d_enc in rad * r / dt = vel -> tf() = robot_vel * dt = dpose
 * => dt can be spared because its unnecessary calculation
 */
static void odometry_cb(const void*) {
  VelRF dpose_rframe_matrix = vWheel2vRF(VelWheel(enc_msg_buf.msg.data.data) *
                                         robot_params.wheel_radius);
  Pose<real_t> dpose_rf{dpose_rframe_matrix(0), dpose_rframe_matrix(1),
                        dpose_rframe_matrix(2)};
  // update_epsilon(dpose_rframe_matrix(3)); // factor for this is zero anyway

  Pose<real_t> dpose_wf =
      pRF2pWF(dpose_rf, pose.theta + dpose_rf.theta / static_cast<real_t>(2));
  pose += dpose_wf;

  log_message(
      log_debug,
      "[odometry]: publish pose cur triggered from enc: [x: %.02f, y: %.02f, "
      "theta: %.02f]",
      pose.x, pose.y, static_cast<real_t>(pose.theta));
  auto msg = geometry_msgs__msg__Pose2D{pose.x, pose.y, pose.theta};
  rcl_ret_softcheck(rcl_publish(&pub_odometry, &msg, NULL));
}

rclc_executor_t* odometry_init(rcl_node_t* node, rclc_support_t* support,
                               const rcl_allocator_t* allocator) {
  rcl_ret_check(rclc_executor_init(&odometry_exe, &support->context,
                                   N_EXEC_HANDLES, allocator));

  rclc_subscription_init_default(
      &sub_encoder_data, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "encoder_data");
  rcl_ret_check(rclc_executor_add_subscription(&odometry_exe, &sub_encoder_data,
                                               &enc_msg_buf.msg, &odometry_cb,
                                               ON_NEW_DATA));

  rcl_ret_check(rclc_publisher_init_default(
      &pub_odometry, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), "odometry"));

  return &odometry_exe;
}
}
