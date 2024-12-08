#include "lidar.hpp"

#include <application/real_t.h>
#include <rcl/subscription.h>
#include <rcl/time.h>
#include <rcl/timer.h>
#include <rclc/executor.h>
#include <rclc/executor_handle.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <rclc/timer.h>
#include <rmw_microros/time_sync.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/bool.h>
#include <tim.h>
#include <usart.h>
#include <numbers>

#include <application/robot_params.hpp>

#include "rcl_guard.hpp"

using namespace robot_params;

static constexpr uint16_t TIMER_TIMEOUT_MS = UROS_FREQ_MOD_LIDAR_SEC * S_TO_MS;
static constexpr uint8_t N_EXEC_HANDLES = 2;

static constexpr uint8_t START_CMD[] = {0xA5, 0x20};
static constexpr uint8_t STOP_CMD[] = {0xA5, 0x25};
static constexpr uint8_t RESET_CMD[] = {0xA5, 0x40};
// static constexpr uint8_t GET_HEALTH_CMD[] = {0xA5, 0x52};

static constexpr uint16_t LIDAR_RANGE = 360;

extern "C" {
static auto timer = rcl_get_zero_initialized_timer();
static rclc_executor_t exe;

static rcl_subscription_t enable_sub;
static std_msgs__msg__Bool enable_msg;

static rcl_publisher_t scan_pub;
static sensor_msgs__msg__LaserScan scan_msg{};

static uint8_t rxbuf[1024];
static uint16_t distances_mm[LIDAR_RANGE];
static float qualities[LIDAR_RANGE];

/*
 * setup DMA with idle line detecting interrupt for receiving into the `rxbuf`:
 * HAL_UARTEx_ReceiveToIdle_DMA will generate calls to user defined
 * HAL_UARTEx_RxEventCallback for each occurrence of the following events:
 * - DMA RX Half Transfer event (HT)
 * - DMA RX Transfer Complete event (TC)
 * - IDLE event on UART Rx line (indicating a pause of the UART reception flow)
 *
 * second parameter: value of the next, unwritten byte position in the range of
 * [1, sizeof(rxbuf)] - meaning it should be the limit of where to stop to read
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,
                                uint16_t target_idx) {
  static size_t rxbuf_idx = 0;

  uint8_t quality = 0;
  uint16_t dist_mm = 0;
  uint16_t angle = 0;

  bool process_packet = false;
  size_t packet_idx = 0;

  while (rxbuf_idx != target_idx) {
    /*
     * DMA in circular mode:
     * the target_idx will be $sizeof(rx_buf)$ instead of 0, if the whole buffer
     * is already written completely and is ready to write the next byte at the
     * start of the buffer, so when rxbuf_idx == sizeof(rxbuf), then it must
     * have already read the whole buffer and should start reading at the
     * beginning
     */
    rxbuf_idx %= sizeof(rxbuf);

    uint8_t val = rxbuf[rxbuf_idx];
    if (!process_packet && (val & 1) != (val & 2))
      process_packet = true;

    if (process_packet) [[likely]] {
      switch (packet_idx) {
        case 0:
          quality = val >> 2;
          break;
        case 1:
          if (!(val & 1)) [[unlikely]] {
            process_packet = false;
            packet_idx = 0;
            return;
          }
          angle = val >> 1;
          break;
        case 2:
          angle = static_cast<uint16_t>(val << 7 | angle) / 64;
          qualities[angle] = static_cast<float>(quality);
          break;
        case 3:
          dist_mm = val;
          break;
        case 4:
          distances_mm[angle] = static_cast<uint16_t>(dist_mm | val << 8) / 4;
          process_packet = false;
          break;
      }

      packet_idx = ++packet_idx % 5; // not 
    }

    ++rxbuf_idx;
  }
}

static void init_motor(void) {
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
}

static void init_ros_msg(void) {
  /* msg imp. detail see `ros2 interface show sensor_msgs/msg/LaserScan` */

  static char header_string[] = "scan data";
  int64_t ns = rmw_uros_epoch_nanos();
  scan_msg.header.stamp.nanosec = ns;
  scan_msg.header.stamp.sec = RCL_NS_TO_S(ns);
  scan_msg.header.frame_id.data = header_string;
  scan_msg.header.frame_id.size = scan_msg.header.frame_id.capacity =
      sizeof(header_string);

  static constexpr real_t LIDAR_RANGE_RAD = (2 * std::numbers::pi) /
      LIDAR_RANGE;
  scan_msg.angle_min = 0;
  scan_msg.angle_max = LIDAR_RANGE_RAD * (LIDAR_RANGE - 1);  // 359 deg in rad
  scan_msg.angle_increment = LIDAR_RANGE_RAD;
  scan_msg.range_min = 0.15;
  scan_msg.range_max = 12;
  scan_msg.ranges.size = LIDAR_RANGE;
  scan_msg.intensities.size = LIDAR_RANGE;
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

static void timer_cb(rcl_timer_t* timer, int64_t) {
  if (!enable_msg.data) return;

  static float distances_m[LIDAR_RANGE]{};

  // post process for the callback to be lightweight
  for (size_t i = 0; i < LIDAR_RANGE; ++i)
    distances_m[i] = static_cast<real_t>(distances_mm[i]) / 1000;

  // puts("===============================");
  // for (size_t i = 0; i < LIDAR_RANGE; ++i)
  //   printf("%.02f ", distances_m[i]);
  // puts("");

  scan_msg.ranges.data = distances_m;
  scan_msg.intensities.data = qualities;
  rcl_softguard(rcl_publish(&scan_pub, &scan_msg, NULL));
}

rclc_executor_t* lidar_init(rcl_node_t* node, rclc_support_t* support,
                            const rcl_allocator_t* allocator) {
  rcl_softguard(
      rclc_executor_init(&exe, &support->context, N_EXEC_HANDLES, allocator));

  rcl_softguard(rclc_timer_init_default2(
      &timer, support, RCL_MS_TO_NS(TIMER_TIMEOUT_MS), &timer_cb, true));
  rcl_softguard(rclc_executor_add_timer(&exe, &timer));

  rcl_softguard(rclc_subscription_init_best_effort(
      &enable_sub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "lidar_enable"));
  rcl_softguard(rclc_executor_add_subscription(
      &exe, &enable_sub, &enable_msg, &enable_cb, ON_NEW_DATA));

  rcl_softguard(rclc_publisher_init_default(
      &scan_pub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
      "lidar_scan"));

  /* setup DMA receiving into `rxbuf` */
  rcl_softguard(
      HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxbuf, sizeof(rxbuf)));

  rcl_softguard(
      HAL_UART_Transmit_DMA(&huart2, RESET_CMD, sizeof(RESET_CMD)));
  init_motor();
  init_ros_msg();

  return &exe;
}
}
