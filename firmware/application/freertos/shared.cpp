#include "shared.hpp"

#include <algorithm>
#include <array>
#include <functional>
#include <utility>

#include "four_wheel_data.hpp"
#include "vel2d_frame.hpp"

namespace freertos {

QueueHandle_t vel_sp_queue;
QueueHandle_t enc_delta_odom_queue;
QueueHandle_t enc_delta_wheel_ctrl_queue;
QueueHandle_t odom_queue;
QueueHandle_t wheel_vel_queue;

using freertos::FourWheelData;

static std::array<std::pair<QueueHandle_t*, std::function<QueueHandle_t(void)>>,
                  5>
    queues{
        std::make_pair(&vel_sp_queue,
                       []() { return xQueueCreate(10, sizeof(Vel2d)); }),
        std::make_pair(&enc_delta_odom_queue,
                       []() { return xQueueCreate(1, sizeof(FourWheelData)); }),
        std::make_pair(
            &enc_delta_wheel_ctrl_queue,
            []() { return xQueueCreate(10, sizeof(FourWheelData)); }),
        std::make_pair(
            &odom_queue,
            []() { return xQueueCreate(10, sizeof(FourWheelData)); }),
        std::make_pair(
            &wheel_vel_queue,
            []() { return xQueueCreate(10, sizeof(FourWheelData)); }),
    };

void queues_init() {
  std::for_each(begin(queues), end(queues), [](auto& pair) {
    auto& [queue, init] = pair;
    *queue = init();
    configASSERT(queue != NULL);
  });
}

}  // namespace freertos
