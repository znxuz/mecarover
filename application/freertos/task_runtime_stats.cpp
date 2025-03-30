#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <main.h>
#include <printf.h>
#include <semphr.h>
#include <stdarg.h>

#include <threadsafe_sink.hpp>
#include <utility>

#include "cycle_stamp.hpp"

static TaskHandle_t button_task_hdl;
static TaskHandle_t profiling_task_hdl;

static volatile size_t ctx_switch_cnt = 0;
using namespace freertos;

extern "C" {
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  static constexpr uint8_t DEBOUNCE_TIME_MS = 50;
  static volatile uint32_t last_interrupt_time = 0;

  if (GPIO_Pin != USER_Btn_Pin) return;

  uint32_t current_time = HAL_GetTick();
  if (current_time - std::exchange(last_interrupt_time, current_time) >
      DEBOUNCE_TIME_MS) {
    static BaseType_t xHigherPriorityTaskWoken;
    configASSERT(button_task_hdl != NULL);
    vTaskNotifyGiveFromISR(button_task_hdl, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void task_switched_in_isr(const char* name) {
  if (!stamping_enabled) return;

  stamp<true>(name, true);
  ctx_switch_cnt += 1;
}

void task_switched_out_isr(const char* name) {
  if (!stamping_enabled) return;

  stamp<true>(name, false);
  ctx_switch_cnt += 1;
}
}

namespace {
int prints(const char* format, ...) {
  char buffer[100] = {0};

  va_list args;
  va_start(args, format);
  size_t size = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  configASSERT(size <= sizeof(buffer));
  tsink_write(buffer, size);

  return size;
}

void button_task_impl(void*) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    taskENTER_CRITICAL();
    stamp_idx = 0;
    ctx_switch_cnt = 0;
    stamping_enabled = true;
    taskEXIT_CRITICAL();
    xTaskNotifyGive(profiling_task_hdl);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    stamping_enabled = false;
  }
}

void profiling_task_impl(void*) {
  auto print_stats = []() static {
    static constexpr uint8_t configNUM_TASKS = 10;
    static char stat_buf[50 * configNUM_TASKS];

    vTaskGetRunTimeStats(stat_buf);
    tsink_write_str("=============================================\n");
    prints("free heap:\t\t%u\n", xPortGetFreeHeapSize());
    prints("ctx switches:\t\t%u\n", ctx_switch_cnt);
    prints("Task\t\tTime\t\t%%\n");
    tsink_write_str(stat_buf);

    tsink_write_str("---------------------------------------------\n");

    vTaskList(stat_buf);
    prints("Task\t\tState\tPrio\tStack\tNum\n");
    tsink_write_str(stat_buf);
    tsink_write_str("=============================================\n");
  };

  size_t prev_idx = 0;
  size_t start_cycle = 0;
  while (true) {
    if (!stamping_enabled) {
      if (start_cycle) {
        print_stats();
        prints("output took %u us\n",
               static_cast<unsigned long>(
                   static_cast<double>(DWT->CYCCNT - start_cycle) /
                   SystemCoreClock * 1000 * 1000));
        start_cycle = 0;
      }

      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      prev_idx = 0;
      start_cycle = stamps.front().cycle;
    }

    vTaskDelay(RT_STAT_TRANSMIT_FREQ);
    while (prev_idx != stamp_idx) {
      const auto& [name, cycle, is_begin] = stamps[prev_idx];
      prints(
          "%s %lu %s\n", name,
          static_cast<unsigned long>(static_cast<double>(cycle - start_cycle) /
                                     SystemCoreClock * 1000 * 1000),
          (is_begin ? "in" : "out"));

      prev_idx = (prev_idx + 1) % stamps.size();
    }
  }
}
}  // namespace

namespace freertos {
void task_runtime_stats_init() {
  auto enable_dwt_cycle_count = []() static {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;  // software unlock
    DWT->CYCCNT = 1;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  };

  enable_dwt_cycle_count();
  configASSERT((xTaskCreate(button_task_impl, "btn", configMINIMAL_STACK_SIZE,
                            NULL, osPriorityNormal, &button_task_hdl)));
  configASSERT((xTaskCreate(profiling_task_impl, "rt_stats",
                            configMINIMAL_STACK_SIZE * 4, NULL,
                            osPriorityNormal, &profiling_task_hdl) == pdPASS));
}
}  // namespace freertos
