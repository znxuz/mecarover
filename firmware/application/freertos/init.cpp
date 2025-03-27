#include "init.hpp"

#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <queue.h>
#include <ulog.h>
#include <usart.h>

#include <application/hal/hal.hpp>
#include <threadsafe_sink.hpp>

#include "shared.hpp"

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance != huart3.Instance) return;

  freertos::tsink_consume_complete<freertos::TSINK_CALL_FROM::ISR>();
}

namespace freertos {

void task_odom_init();
void task_pose_ctrl_init();
void task_vel_recv_init();
void task_hal_fetch_init();
void task_wheel_ctrl_init();
void task_runtime_stats_init();

void init() {
  auto tsink_consume = [](const uint8_t* buf, size_t size) static {
    auto flush_cache_aligned = [](uintptr_t addr, size_t size) static {
      constexpr auto align_addr = [](uintptr_t addr) { return addr & ~0x1F; };
      constexpr auto align_size = [](uintptr_t addr, size_t size) {
        return size + ((addr) & 0x1F);
      };

      SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(align_addr(addr)),
                              align_size(addr, size));
    };

    flush_cache_aligned(reinterpret_cast<uintptr_t>(buf), size);
    HAL_UART_Transmit_DMA(&huart3, buf, size);
  };
  tsink_init(tsink_consume, osPriorityAboveNormal);

#ifdef FREERTOS_STATIC_INIT
  ULOG_INFO("intializing freertos via static memory allocation");
#else
  ULOG_INFO("intializing freertos via dynamic memory allocation");
#endif

  hal_init();
  queues_init();
  task_hal_fetch_init();
  task_vel_recv_init();
  task_pose_ctrl_init();
  task_wheel_ctrl_init();
  task_odom_init();
  task_runtime_stats_init();
}

}  // namespace freertos
