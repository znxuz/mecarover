#include "lidar.hpp"

#include <rcl/subscription.h>
#include <rcl/timer.h>
#include <rclc/executor.h>
#include <rclc/executor_handle.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <rclc/timer.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/bool.h>
#include <tim.h>
#include <usart.h>

#include <application/robot_params.hpp>

#include "rcl_ret_check.hpp"
#include "stm32f7xx_hal_uart.h"
#include "ulog.h"

using namespace robot_params;

static constexpr uint16_t TIMER_TIMEOUT_MS = UROS_FREQ_MOD_LIDAR_SEC * S_TO_MS;
static constexpr uint8_t N_EXEC_HANDLES = 2;

static constexpr uint8_t START_CMD[2] = {0xA5, 0x20};
static constexpr uint8_t STOP_CMD[2] = {0xA5, 0x25};
static constexpr uint8_t RESET_CMD[2] = {0xA5, 0x40};
static constexpr uint8_t GET_HEALTH_CMD[2] = {0xA5, 0x52};

extern "C" {
static auto timer = rcl_get_zero_initialized_timer();

static rclc_executor_t exe;

static rcl_subscription_t enable_sub;
static std_msgs__msg__Bool enable_msg;

static rcl_publisher_t data_sub;

static uint8_t rx_buf[14];

static void init_lidar_motor(void) {
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
}

static void start(void) {
  __HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF | UART_CLEAR_OREF);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, htim11.Instance->ARR);
  HAL_UART_Transmit_DMA(&huart2, START_CMD, sizeof(START_CMD));
}

static void stop(void) {
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
  HAL_UART_Transmit_DMA(&huart2, STOP_CMD, sizeof(STOP_CMD));
}

static void enable_cb(const void* arg) {
  reinterpret_cast<const std_msgs__msg__Bool*>(arg)->data ? start() : stop();
}

static void timer_cb(rcl_timer_t* timer, int64_t last_call_time) {
  rcl_ret_softcheck(
      HAL_UART_Transmit_DMA(&huart2, GET_HEALTH_CMD, sizeof(GET_HEALTH_CMD)));
  vTaskDelay(100);

  ULOG_DEBUG(
      "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
      rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5],
      rx_buf[6], rx_buf[7], rx_buf[8], rx_buf[9], rx_buf[10], rx_buf[11],
      rx_buf[12], rx_buf[13]);
}

rclc_executor_t* lidar_init(rcl_node_t* node, rclc_support_t* support,
                            const rcl_allocator_t* allocator) {
  rcl_ret_softcheck(
      rclc_executor_init(&exe, &support->context, N_EXEC_HANDLES, allocator));

  rcl_ret_softcheck(rclc_timer_init_default2(
      &timer, support, RCL_MS_TO_NS(TIMER_TIMEOUT_MS), &timer_cb, true));
  rcl_ret_softcheck(rclc_executor_add_timer(&exe, &timer));

  rcl_ret_softcheck(rclc_publisher_init_best_effort(
      &data_sub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
      "lidar_data"));

  rcl_ret_softcheck(rclc_subscription_init_best_effort(
      &enable_sub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "lidar_enable"));
  rcl_ret_softcheck(rclc_executor_add_subscription(
      &exe, &enable_sub, &enable_msg, &enable_cb, ON_NEW_DATA));

  // setup DMA for receiving
  rcl_ret_softcheck(HAL_UART_Receive_DMA(&huart2, rx_buf, sizeof(rx_buf)));

  init_lidar_motor();

  return &exe;
}
}
