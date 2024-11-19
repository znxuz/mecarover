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

#include "application/real_t.h"
#include "rcl_ret_check.hpp"
#include "ulog.h"

using namespace robot_params;

static constexpr uint16_t TIMER_TIMEOUT_MS = UROS_FREQ_MOD_LIDAR_SEC * S_TO_MS;
static constexpr uint8_t N_EXEC_HANDLES = 2;

static constexpr uint8_t START_CMD[] = {0xA5, 0x20};
static constexpr uint8_t STOP_CMD[] = {0xA5, 0x25};
static constexpr uint8_t RESET_CMD[] = {0xA5, 0x40};
static constexpr uint8_t GET_HEALTH_CMD[] = {0xA5, 0x52};

static constexpr uint8_t RESP_FLAGS[] = {0xA5, 0x5A};

static constexpr uint16_t LIDAR_RANGE = 360;

extern "C" {

static auto timer = rcl_get_zero_initialized_timer();
static rclc_executor_t exe;

static rcl_subscription_t enable_sub;
static std_msgs__msg__Bool enable_msg;

static rcl_publisher_t data_sub;

static uint8_t rx_buf[2048];
static uint8_t process_buf[1024];
static size_t read_bytes;
volatile real_t distances[LIDAR_RANGE];
volatile real_t qualities[LIDAR_RANGE];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,
                                uint16_t target_idx) {
  static size_t rxbuf_idx = 0;
  static size_t angle_idx = 0;

  if (rxbuf_idx == target_idx) return;

  size_t resp_flag_idx = 0;
  bool resp_flag_found = false;
  size_t packet_idx = 0;
  uint16_t angle_cache = 0;
  uint16_t dist_mm_cache = 0;

  while (rxbuf_idx != target_idx) {
    uint8_t val = rx_buf[rxbuf_idx];

    // TODO: testing needed
    // TODO: remove retarget and write _read and _write to use uart transmit DMA
    if (val == RESP_FLAGS[0]) {
      resp_flag_idx = rxbuf_idx;
    } else if (val == RESP_FLAGS[1] && resp_flag_idx + 1 == rxbuf_idx) {
      resp_flag_found = true;
    } else if (resp_flag_found) [[likely]] {
      switch (packet_idx) {
        case 0: {
          if ((val & 1) != ~(val & 2)) [[unlikely]]
            ULOG_ERROR("packet start bit and inverted start bit are equal!");
          if (val & 1) angle_idx = 0;
          qualities[angle_idx] = static_cast<real_t>(val >> 2);
          break;
        }
        case 1: {
          if (!(val & 1)) [[unlikely]]
            ULOG_ERROR("packet check bit is 0!");
          angle_cache = val >> 1;
          break;
        }
        case 2: {
          angle_cache |= static_cast<uint16_t>(val << 7);
          auto angle = std::round(static_cast<real_t>(angle_cache) / 64);
          if (angle != angle_idx) [[unlikely]]
            ULOG_WARNING("scanned angle not equal to angle index, diff: %u",
                         angle - angle_idx);
          angle_idx = angle;
          break;
        }
        case 3: {
          dist_mm_cache = val;
          break;
        }
        case 4: {
          dist_mm_cache |= static_cast<uint16_t>(val << 8);
          distances[angle_idx] = static_cast<real_t>(dist_mm_cache) / 4;
          angle_idx = ++angle_idx % LIDAR_RANGE;
          resp_flag_found = false;
          break;
        }
      }
      packet_idx = ++packet_idx % 5;
    }

    rxbuf_idx = ++rxbuf_idx % sizeof(rx_buf);  // DMA in circular mode
  }

  // if (idx == target_idx) {
  //   return;
  // } else if (idx < target_idx) {
  //   read_bytes = target_idx - idx;
  // } else {
  //   read_bytes = sizeof(rx_buf) - idx + target_idx;
  // }
  // // copy the bytes into process_buf for easier linear data processing
  // for (size_t i = 0; i < read_bytes; ++i) {
  //   process_buf[i] = rx_buf[idx++];
  //   idx %= sizeof(rx_buf);
  // }
  // extract distance and write into the distance array
  // find header, then read 5 bytes
}

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

static void lidar_health_output(void) {
  rcl_ret_softcheck(
      HAL_UART_Transmit_DMA(&huart2, GET_HEALTH_CMD, sizeof(GET_HEALTH_CMD)));
  for (size_t i = 0; i < read_bytes; ++i) {
    printf("0x%02x ", process_buf[i]);
  }
  read_bytes = 0; // discard used bytes
  puts("");
}

static void timer_cb(rcl_timer_t* timer, int64_t last_call_time) {
  // lidar_health_output();

  // TODO: use distances array
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

  /*
   * setup DMA with idle line detecting interrupt for receiving:
   * HAL_UARTEx_ReceiveToIdle_DMA will generate calls to user defined
   * HAL_UARTEx_RxEventCallback for each occurrence of the following events:
   * - DMA RX Half Transfer event (HT)
   * - DMA RX Transfer Complete event (TC)
   * - IDLE event on UART Rx line (indicating a pause is UART reception flow)
   */
  rcl_ret_softcheck(
      HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, sizeof(rx_buf)));

  init_lidar_motor();

  return &exe;
}
}
