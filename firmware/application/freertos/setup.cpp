#include "setup.hpp"

#include <FreeRTOS.h>
#include <queue.h>
#include <ulog.h>

#include <application/hal/hal.hpp>
#include <array>
#include <functional>
#include <utility>

#include "four_wheel_data.hpp"
#include "task_odometry.hpp"
#include "task_pose_control.hpp"
#include "task_recv_vel.hpp"
#include "vel2d_frame.hpp"

namespace freertos {

static QueueHandle_t vel_sp_queue;
static QueueHandle_t enc_delta_queue;
static QueueHandle_t odom_queue;
static QueueHandle_t wheel_vel_queue;

std::array<std::pair<QueueHandle_t*, std::function<QueueHandle_t(void)>>, 4> qs{
    std::make_pair(&vel_sp_queue,
                   []() { return xQueueCreate(10, sizeof(Vel2d)); }),
    std::make_pair(&enc_delta_queue,
                   []() { return xQueueCreate(10, sizeof(FourWheelData)); }),
    std::make_pair(&odom_queue,
                   []() { return xQueueCreate(10, sizeof(FourWheelData)); }),
    std::make_pair(&wheel_vel_queue,
                   []() { return xQueueCreate(10, sizeof(FourWheelData)); })};

static void queues_init() {
  std::for_each(begin(qs), end(qs), [](auto& pair) {
    auto& [queue, init] = pair;
    *queue = init();
    configASSERT(queue != NULL);
  });
}

void setup() {
  ULOG_INFO("freertos setup");
  hal_init();
  queues_init();

  task_vel_recv_init(vel_sp_queue);
  task_pose_control_init(vel_sp_queue, odom_queue, wheel_vel_queue);
  // task_odometry_init(enc_delta_queue, odom_queue);
  ULOG_INFO("freertos setup finished");
}

}  // namespace freertos
