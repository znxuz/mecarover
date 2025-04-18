#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <main.h>
#include <printf.h>
#include <semphr.h>
#include <stdarg.h>

#include <cstring>
#include <string_view>
#include <threadsafe_sink.hpp>
#include <utility>

#include "cycle_stamp.hpp"

static TaskHandle_t profiling_task_hdl;
static volatile size_t ctx_switch_cnt;

using namespace freertos;

extern "C" {
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  static constexpr uint8_t DEBOUNCE_TIME_MS = 100;
  static volatile uint32_t last_interrupt_time = 0;

  if (GPIO_Pin != USER_Btn_Pin) return;

  uint32_t current_time = HAL_GetTick();
  if (current_time - std::exchange(last_interrupt_time, current_time) >
      DEBOUNCE_TIME_MS) {
    stamping_enabled ^= 1;
    if (stamping_enabled) {
      stamp_idx = 0;
      cycle_stamp::initial_cycle = DWT->CYCCNT;

      static BaseType_t xHigherPriorityTaskWoken;
      vTaskNotifyGiveFromISR(profiling_task_hdl, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

void task_switched_isr(const char* name, uint8_t start) {
  if (!stamping_enabled) return;
  stamp(name, start);
  ctx_switch_cnt += 1;
}
}

namespace {
using namespace tsink;
using namespace std::literals::string_view_literals;

static uint32_t cycle_to_us(uint32_t cycle) {
  return static_cast<uint32_t>(static_cast<float>(cycle) / SystemCoreClock *
                               1000 * 1000);
}

void profiling_task_impl(void*) {
  static constexpr uint8_t configNUM_TASKS = 10;
  static char buf[50 * configNUM_TASKS];
  static size_t prev_idx = 0;

  auto output_task_stats = []() static {
    auto cycle = DWT->CYCCNT;
    __sync_synchronize();
    write_blocking("=============================================\n"sv);
    write_blocking(buf, snprintf(buf, sizeof(buf), "free heap:\t\t%u\n",
                                 xPortGetFreeHeapSize()));
    write_blocking(buf, snprintf(buf, sizeof(buf), "ctx switches:\t\t%u\n",
                                 std::exchange(ctx_switch_cnt, 0)));
    write_blocking("Task\t\tTime\t\t%%\n"sv);
    vTaskGetRunTimeStats(buf);
    write_blocking(buf, std::strlen(buf));
    write_blocking("---------------------------------------------\n"sv);
    vTaskList(buf);
    write_blocking("Task\t\tState\tPrio\tStack\tNum\n"sv);
    write_blocking(buf, std::strlen(buf));
    write_blocking("=============================================\n"sv);
    write_blocking(buf,
                   snprintf(buf, sizeof(buf), "output took %u us\n",
                            cycle_to_us(cycle - cycle_stamp::initial_cycle)));
  };
  auto output_stamps = []() static {
    auto end = stamp_idx;
    // auto diff = end - prev_idx;
    while (prev_idx != end) {
      const auto& [name, cycle, is_begin] = stamps[prev_idx++ % STAMP_BUF_SIZE];
      write_blocking(
          buf,
          snprintf(buf, sizeof(buf), "%s %u %u\n", name,
                   cycle_to_us(cycle - cycle_stamp::initial_cycle), is_begin));
    }
  };

  while (true) {
    if (!stamping_enabled) {
      if (std::exchange(prev_idx, 0)) output_task_stats();
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    output_stamps();
    vTaskDelay(pdMS_TO_TICKS(STAMP_OUTPUT_FREQ));
  }
}
}  // namespace

namespace freertos {
void task_profiling_init() {
  configASSERT(
      (xTaskCreate(profiling_task_impl, "profile", configMINIMAL_STACK_SIZE * 8,
                   NULL, osPriorityNormal, &profiling_task_hdl) == pdPASS));
}
}  // namespace freertos
