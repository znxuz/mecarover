#include "odometry.hpp"

#include <geometry_msgs/msg/pose2_d.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <rclc/types.h>
#include <ulog.h>

#include <application/hal/hal.hpp>
#include <application/pose_types.hpp>
#include <application/robot_params.hpp>

#include "drive_state_wrapper.hpp"
#include "jacobi_transformation.hpp"
#include "rcl_guard.hpp"

using namespace imsl;
using namespace robot_params;

static constexpr uint8_t N_EXEC_HANDLES = 1;

extern "C" {
static auto odometry_exe = rclc_executor_get_zero_initialized_executor();

static rcl_subscription_t sub_encoder_data;
static auto enc_data_msg = DriveStateWrapper<DriveStateType::ENC_DELTA_RAD>{};
static Pose<real_t> pose_wf;

static rcl_publisher_t pub_odometry;

extern real_t epsilon;

static void odometry_cb(const void* arg) {
  const auto* enc_delta_rad = reinterpret_cast<const DriveState*>(arg);
  /*
   * odometry: encoder delta gets fed directly into the inverted jacobian
   * matrix without dividing the dt for the reason being:
   * enc delta in rad / dt = vel -> forward_transform(vel) = vel_rf * dt = dpose
   * => dt can be canceled out on both sides
   */
  const auto dpose_rf_mtx =
      forward_transform(VelWheel(enc_delta_rad->front_right_wheel_velocity,
                                 enc_delta_rad->front_left_wheel_velocity,
                                 enc_delta_rad->back_left_wheel_velocity,
                                 enc_delta_rad->back_right_wheel_velocity));
  Pose<real_t> dpose_rf{dpose_rf_mtx(0), dpose_rf_mtx(1), dpose_rf_mtx(2)};
  epsilon += dpose_rf_mtx(3);

  // use the theta average for more precise odometry calculation
  pose_wf += pRF2pWF(
      dpose_rf, (static_cast<real_t>(pose_wf.theta) * 2 + dpose_rf.theta) / 2);

  auto msg = geometry_msgs__msg__Pose2D{pose_wf.x, pose_wf.y, pose_wf.theta};
  rcl_softguard(rcl_publish(&pub_odometry, &msg, NULL));
}

rclc_executor_t* odometry_init(rcl_node_t* node, rclc_support_t* support,
                               const rcl_allocator_t* allocator) {
  rcl_guard(rclc_executor_init(&odometry_exe, &support->context,
                                   N_EXEC_HANDLES, allocator));

  rclc_subscription_init_best_effort(
      &sub_encoder_data, node,
      DriveStateWrapper<DriveStateType::ENC_DELTA_RAD>::get_msg_type_support(),
      "encoder_data");
  rcl_guard(rclc_executor_add_subscription(&odometry_exe, &sub_encoder_data,
                                               &enc_data_msg.state,
                                               &odometry_cb, ON_NEW_DATA));

  rcl_guard(rclc_publisher_init_best_effort(
      &pub_odometry, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), "odometry"));

  return &odometry_exe;
}
}
