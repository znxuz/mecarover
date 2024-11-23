#include "lidar.hpp"

#include <application/real_t.h>
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

using namespace robot_params;

static constexpr uint16_t TIMER_TIMEOUT_MS = UROS_FREQ_MOD_LIDAR_SEC * S_TO_MS;
static constexpr uint8_t N_EXEC_HANDLES = 2;

static constexpr uint8_t START_CMD[] = {0xA5, 0x20};
static constexpr uint8_t STOP_CMD[] = {0xA5, 0x25};
static constexpr uint8_t GET_HEALTH_CMD[] = {0xA5, 0x52};
static constexpr uint8_t RESET_CMD[] = {0xA5, 0x40};

static constexpr uint8_t RESP_FLAGS[] = {0xA5, 0x5A};

static constexpr uint16_t LIDAR_RANGE = 360;

extern "C" {

static auto timer = rcl_get_zero_initialized_timer();
static rclc_executor_t exe;

static rcl_subscription_t enable_sub;
static std_msgs__msg__Bool enable_msg;

static rcl_publisher_t data_sub;

static uint8_t rxbuf[1024];
static uint8_t tmpbuf[512];
static volatile uint16_t distances_mm[LIDAR_RANGE];
static volatile uint16_t qualities[LIDAR_RANGE];

static size_t angle_cnt = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,
                                uint16_t target_idx) {
  static size_t rxbuf_idx = 0;
  static uint16_t angle_idx = 0;

  bool resp_flag_first_half = false;
  bool start_process_packet = false;
  uint8_t quality_cache = 0;
  uint16_t angle_cache = 0;
  uint16_t dist_mm_cache = 0;
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
    if (!start_process_packet && (val & 1) != (val & 2)) {
      start_process_packet = true;
    }

    if (start_process_packet) [[likely]] {
      switch (packet_idx) {
        case 0: {
          if (val & 1) [[unlikely]]
            angle_idx = 0;
          quality_cache = val >> 2;
          break;
        }
        case 1: {
          if (!(val & 1)) [[unlikely]] {
            start_process_packet = false;
            packet_idx = 0;
            return;
          }
          angle_cache = val >> 1;
          break;
        }
        case 2: {
          uint16_t parsed_angle =
              static_cast<uint16_t>((val << 7) | angle_cache) / 64;
          angle_idx = parsed_angle;
          qualities[angle_idx] = quality_cache;
          break;
        }
        case 3: {
          dist_mm_cache = val;
          break;
        }
        case 4: {
          distances_mm[angle_idx] =
              static_cast<uint16_t>(dist_mm_cache | val << 8) / 4;
          angle_idx = ++angle_idx % LIDAR_RANGE;
          start_process_packet = false;
          break;
        }
      }

      packet_idx = ++packet_idx % 5;
    }

    ++rxbuf_idx;
  }
}

static void init_lidar_motor(void) {
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
}

static void start(void) {
  __HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF | UART_CLEAR_OREF);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 5000);
  HAL_UART_Transmit_DMA(&huart2, START_CMD, sizeof(START_CMD));
}

static void stop(void) {
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
  HAL_UART_Transmit_DMA(&huart2, STOP_CMD, sizeof(STOP_CMD));
}

static void enable_cb(const void* arg) {
  reinterpret_cast<const std_msgs__msg__Bool*>(arg)->data ? start() : stop();
}

// static void lidar_check_health() {
//   HAL_UART_Transmit_DMA(&huart2, GET_HEALTH_CMD, sizeof(GET_HEALTH_CMD));
//
//   for (size_t i = 0; i < sizeof(rxbuf); ++i) {
//     printf("0x%02x ", rxbuf[i]);
//   }
//   puts("");
// }

static void timer_cb(rcl_timer_t* timer, int64_t last_call_time) {
  if (!enable_msg.data)
    return;

  puts("===================");
  for (size_t i = 0; i < LIDAR_RANGE; ++i) {
    printf("%u%s", distances_mm[i],
           (i && !(i % 36)) || i + 1 == LIDAR_RANGE ? "\n" : " ");
  }
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
      HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxbuf, sizeof(rxbuf)));

  HAL_UART_Transmit_DMA(&huart2, RESET_CMD, sizeof(RESET_CMD));

  init_lidar_motor();

  return &exe;
}
}
