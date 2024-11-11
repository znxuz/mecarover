#include "lidar.hpp"

#include <rcl/timer.h>
#include <rclc/executor.h>
#include <rclc/publisher.h>
#include <rclc/timer.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <tim.h>
#include <usart.h>

#include <application/robot_params.hpp>

#include "rcl_ret_check.hpp"
#include "stm32f7xx_hal_uart.h"

using namespace robot_params;

static constexpr uint16_t TIMER_TIMEOUT_MS = UROS_FREQ_MOD_LIDAR_SEC * S_TO_MS;
static constexpr uint8_t N_EXEC_HANDLES = 1;

static constexpr uint8_t START_CMD[2] = {0xA5, 0x20};
static constexpr uint8_t STOP_CMD[2] = {0xA5, 0x25};
static constexpr uint8_t RESET_CMD[2] = {0xA5, 0x40};
static constexpr uint8_t GET_HEALTH_CMD[2] = {0xA5, 0x52};

extern "C" {
static auto wheel_ctrl_timer = rcl_get_zero_initialized_timer();

static rclc_executor_t lidar_exe;

static rcl_publisher_t pub_lidar_data;

static uint8_t rx_buf[15];

static void lidar_cb(rcl_timer_t* timer, int64_t last_call_time) {
  rcl_ret_softcheck(
      HAL_UART_Transmit_DMA(&huart2, GET_HEALTH_CMD, sizeof(GET_HEALTH_CMD)));
  vTaskDelay(100);

  // TODO idle line detection
}

rclc_executor_t* lidar_init(rcl_node_t* node, rclc_support_t* support,
                            const rcl_allocator_t* allocator) {
  rcl_ret_check(rclc_executor_init(&lidar_exe, &support->context,
                                   N_EXEC_HANDLES, allocator));

  rcl_ret_check(rclc_timer_init_default2(&wheel_ctrl_timer, support,
                                         RCL_MS_TO_NS(TIMER_TIMEOUT_MS),
                                         &lidar_cb, true));
  rcl_ret_check(rclc_executor_add_timer(&lidar_exe, &wheel_ctrl_timer));

  rcl_ret_check(rclc_publisher_init_best_effort(
      &pub_lidar_data, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "lidar_data"));

  rcl_ret_softcheck(HAL_UART_Receive_DMA(&huart2, rx_buf, sizeof(rx_buf)));
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);

  return &lidar_exe;
}
}
