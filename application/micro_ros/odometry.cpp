#include "odometry.hpp"

#include <geometry_msgs/msg/pose2_d.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <rclc/timer.h>
#include <rclc/types.h>
#include <ulog.h>

#include <application/hal/hal.hpp>
#include <application/jacobi_transformation.hpp>
#include <application/pose_types.hpp>
#include <application/robot_params.hpp>

#include "drive_state_wrapper.hpp"
#include "rcl_guard.hpp"

using namespace imsl;
using namespace robot_params;

static constexpr uint8_t N_EXEC_HANDLES = 2;

extern "C" {
static auto exe = rclc_executor_get_zero_initialized_executor();

static rcl_subscription_t sub_enc_data;
static auto enc_data_msg = DriveStateWrapper<DriveStateType::ENC_DELTA_RAD>{};
static Pose<real_t> pose_wf;

static auto timer = rcl_get_zero_initialized_timer();
static rcl_publisher_t pub_odometry;

real_t epsilon;

static void odometry_cb(const void* arg) {
  const auto* enc_delta_rad = reinterpret_cast<const DriveState*>(arg);
  /*
   * odometry: encoder delta gets fed directly into the inverted jacobian
   * matrix without dividing the dt for the reason being:
   * enc delta in rad / dt = vel -> forward_transform(vel) = vel_rf * dt = dpose
   * => dt gets canceled out on both sides
   */
  const auto dpose_rf_mtx =
      forward_transform(VelWheel(enc_delta_rad->front_right_wheel_velocity,
                                 enc_delta_rad->front_left_wheel_velocity,
                                 enc_delta_rad->back_left_wheel_velocity,
                                 enc_delta_rad->back_right_wheel_velocity));
  Pose<real_t> dpose_rf{dpose_rf_mtx(0), dpose_rf_mtx(1), dpose_rf_mtx(2)};

  taskENTER_CRITICAL();
  epsilon += dpose_rf_mtx(3);
  taskEXIT_CRITICAL();

  // aggregate into the pose sum and use the theta average for more precise
  // angle calculation
  pose_wf += pRF2pWF(
      dpose_rf, (static_cast<real_t>(pose_wf.theta) * 2 + dpose_rf.theta) / 2);
}

static void odom_pub_cb(rcl_timer_t*, int64_t) {
  auto msg = geometry_msgs__msg__Pose2D{pose_wf.x, pose_wf.y, pose_wf.theta};
  rcl_softguard(rcl_publish(&pub_odometry, &msg, NULL));
}

rclc_executor_t* odometry_init(rcl_node_t* node, rclc_support_t* support,
                               const rcl_allocator_t* allocator) {
  rcl_guard(
      rclc_executor_init(&exe, &support->context, N_EXEC_HANDLES, allocator));

  rclc_subscription_init_best_effort(
      &sub_enc_data, node,
      DriveStateWrapper<DriveStateType::ENC_DELTA_RAD>::get_msg_type_support(),
      "encoder_data");
  rcl_guard(rclc_executor_add_subscription(
      &exe, &sub_enc_data, &enc_data_msg.state, &odometry_cb, ON_NEW_DATA));

  rcl_guard(rclc_timer_init_default2(&timer, support,
                                     RCL_S_TO_NS(POSE_CTRL_PERIOD_S),
                                     &odom_pub_cb, true));
  rcl_guard(rclc_executor_add_timer(&exe, &timer));

  rcl_guard(rclc_publisher_init_best_effort(
      &pub_odometry, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), "odometry"));

  return &exe;
}
}
