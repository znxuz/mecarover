#pragma once

#include <rcl/allocator.h>
#include <rcl/node.h>
#include <rclc/executor.h>
#include <rclc/executor_handle.h>
#include <mecarover/mrtypes.h>
#include <algorithm>
#include <array>
#include "mecarover/robot_params.hpp"

extern "C" {
rclc_executor_t* wheel_ctrl_init(rcl_node_t* node, rclc_support_t* support,
                                 const rcl_allocator_t* allocator);
}

inline std::array<real_t, N_WHEEL> vel_to_duty_cycle(
    const std::array<real_t, N_WHEEL>& wheel_vel) {
  auto ret = std::array<real_t, N_WHEEL>{};

  std::transform(begin(wheel_vel), end(wheel_vel), begin(ret), [](real_t elem) {
    return std::clamp(elem / MAX_VELOCITY, -1.0, 1.0);
  });
  return ret;
}
