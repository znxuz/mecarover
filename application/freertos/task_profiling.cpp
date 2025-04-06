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
    static BaseType_t xHigherPriorityTaskWoken;
    configASSERT(button_task_hdl != NULL);
    vTaskNotifyGiveFromISR(button_task_hdl, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void task_switched_isr(const char* name, uint8_t start) {
  if (!stamping_enabled) return;
  stamp_isr(name, start);
  ctx_switch_cnt += 1;
}
}

namespace {
void button_task_impl(void*) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    taskENTER_CRITICAL();
    isr_stamp_idx = 0;
    ctx_switch_cnt = 0;
    stamping_enabled = true;
    cycle_stamp::initial_cycle = DWT->CYCCNT;
    taskEXIT_CRITICAL();
    xTaskNotifyGive(profiling_task_hdl);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    stamping_enabled = false;
  }
}

void profiling_task_impl(void*) {
  static constexpr uint8_t configNUM_TASKS = 10;
  static char buf[50 * configNUM_TASKS];
  auto print_stats = []() static {
    tsink_write_str("=============================================\n");
    tsink_write_blocking(buf, snprintf(buf, sizeof(buf), "free heap:\t\t%u\n",
                                       xPortGetFreeHeapSize()));
    tsink_write_blocking(
        buf,
        snprintf(buf, sizeof(buf), "ctx switches:\t\t%u\n", ctx_switch_cnt));
    tsink_write_str("Task\t\tTime\t\t%%\n");
    vTaskGetRunTimeStats(buf);
    tsink_write_str(buf);
    tsink_write_str("---------------------------------------------\n");
    vTaskList(buf);
    tsink_write_str("Task\t\tState\tPrio\tStack\tNum\n");
    tsink_write_str(buf);
    tsink_write_str("=============================================\n");
  };

  size_t prev_idx = 0;
  while (true) {
    if (!stamping_enabled) {
      if (std::exchange(prev_idx, 0)) {
        print_stats();
        tsink_write_blocking(
            buf,
            snprintf(buf, sizeof(buf), "output took %u us\n",
                     static_cast<unsigned long>(
                         static_cast<double>(
                             DWT->CYCCNT -
                             std::exchange(cycle_stamp::initial_cycle, 0)) /
                         SystemCoreClock * 1000 * 1000)));
      }

      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    vTaskDelay(pdMS_TO_TICKS(ISR_STAMP_WRITE_FREQ));

    // TODO refactor into named lambda
    while (prev_idx != isr_stamp_idx) {
      const auto& [name, cycle, ticket, is_begin] =
          isr_stamps[prev_idx++ % ISR_STAMP_BUF_SIZE];
      tsink_write_ordered(
          buf,
          snprintf(buf, sizeof(buf), "%s %lu %s\n", name,
                   static_cast<unsigned long>(
                       static_cast<double>(cycle - cycle_stamp::initial_cycle) /
                       SystemCoreClock * 1000 * 1000),
                   (is_begin ? "in" : "out")),
          ticket);
    }
  }
}
}  // namespace

namespace freertos {
void task_profiling_init() {
  configASSERT((xTaskCreate(button_task_impl, "btn", configMINIMAL_STACK_SIZE,
                            NULL, osPriorityNormal, &button_task_hdl)));
  configASSERT(
      (xTaskCreate(profiling_task_impl, "profile", configMINIMAL_STACK_SIZE * 8,
                   NULL, osPriorityNormal, &profiling_task_hdl) == pdPASS));
}
}  // namespace freertos
