#include "shared.hpp"

#include <algorithm>
#include <application/pose_types.hpp>
#include <array>
#include <functional>
#include <utility>

#include "four_wheel_data.hpp"

namespace freertos {

using namespace imsl;

using freertos::FourWheelData;

QueueHandle_t vel_sp_queue;
QueueHandle_t enc_delta_odom_queue;
QueueHandle_t enc_delta_wheel_ctrl_queue;
QueueHandle_t odom_queue;
QueueHandle_t vel_wheel_queue;

static std::array<std::pair<QueueHandle_t*, std::function<QueueHandle_t(void)>>,
                  5>
    static_queues{
        std::make_pair(&vel_sp_queue,
                       []() -> QueueHandle_t {
                         constexpr size_t QUEUE_SIZE = 10;
                         static vPose buf[QUEUE_SIZE];
                         static StaticQueue_t static_queue;
                         return xQueueCreateStatic(
                             QUEUE_SIZE, sizeof(*buf),
                             reinterpret_cast<uint8_t*>(buf), &static_queue);
                       }),
        std::make_pair(&enc_delta_odom_queue,
                       []() {
                         constexpr size_t QUEUE_SIZE = 1;
                         static FourWheelData buf[QUEUE_SIZE];
                         static StaticQueue_t static_queue;
                         return xQueueCreateStatic(
                             QUEUE_SIZE, sizeof(*buf),
                             reinterpret_cast<uint8_t*>(buf), &static_queue);
                       }),
        std::make_pair(&enc_delta_wheel_ctrl_queue,
                       []() {
                         constexpr size_t QUEUE_SIZE = 10;
                         static FourWheelData buf[QUEUE_SIZE];
                         static StaticQueue_t static_queue;
                         return xQueueCreateStatic(
                             QUEUE_SIZE, sizeof(*buf),
                             reinterpret_cast<uint8_t*>(buf), &static_queue);
                       }),
        std::make_pair(&odom_queue,
                       []() {
                         constexpr size_t QUEUE_SIZE = 10;
                         static FourWheelData buf[QUEUE_SIZE];
                         static StaticQueue_t static_queue;
                         return xQueueCreateStatic(
                             QUEUE_SIZE, sizeof(*buf),
                             reinterpret_cast<uint8_t*>(buf), &static_queue);
                       }),
        std::make_pair(&vel_wheel_queue,
                       []() {
                         constexpr size_t QUEUE_SIZE = 10;
                         static FourWheelData buf[QUEUE_SIZE];
                         static StaticQueue_t static_queue;
                         return xQueueCreateStatic(
                             QUEUE_SIZE, sizeof(*buf),
                             reinterpret_cast<uint8_t*>(buf), &static_queue);
                       }),
    };

static std::array<std::pair<QueueHandle_t*, std::function<QueueHandle_t(void)>>,
                  5>
    queues{
        std::make_pair(&vel_sp_queue,
                       []() { return xQueueCreate(10, sizeof(vPose)); }),
        std::make_pair(&enc_delta_odom_queue,
                       []() { return xQueueCreate(1, sizeof(FourWheelData)); }),
        std::make_pair(
            &enc_delta_wheel_ctrl_queue,
            []() { return xQueueCreate(10, sizeof(FourWheelData)); }),
        std::make_pair(&odom_queue,
                       []() { return xQueueCreate(10, sizeof(Pose)); }),
        std::make_pair(
            &vel_wheel_queue,
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
