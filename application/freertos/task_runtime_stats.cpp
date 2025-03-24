#include "task_runtime_stats.hpp"

#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <main.h>
#include <printf.h>
#include <semphr.h>
#include <stdarg.h>

#include <threadsafe_sink.hpp>
#include <utility>

#include "task_records.hpp"

static TaskHandle_t button_task_hdl;
static TaskHandle_t profiling_task_hdl;

static volatile size_t ctx_switch_cnt = 0;
using namespace freertos;

extern "C" {
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  static constexpr uint8_t DEBOUNCE_TIME_MS = 200;
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
  if (!task_switch_profiling_enabled) return;

  records[record_idx].is_begin = true;
  records[record_idx].name = name;
  records[record_idx].cycle = DWT->CYCCNT;
  record_idx += 1;
  ctx_switch_cnt += 1;
}

void task_switched_out_isr(const char* name) {
  if (!task_switch_profiling_enabled) return;

  records[record_idx].is_begin = false;
  records[record_idx].name = name;
  records[record_idx].cycle = DWT->CYCCNT;
  record_idx += 1;
  ctx_switch_cnt += 1;
}
}

namespace {
int prints(const char* format, ...) {
  char buffer[500] = {0};

  va_list args;
  va_start(args, format);
  size_t size = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  configASSERT(size <= sizeof(buffer));
  tsink_write(buffer, size);

  return size;
}

void enable_dwt_cycle_count() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = 0xC5ACCE55;  // software unlock
  DWT->CYCCNT = 1;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void button_task_impl(void*) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // unblock the profiling task
    taskENTER_CRITICAL();
    record_idx = 0;
    ctx_switch_cnt = 0;
    task_switch_profiling_enabled = true;
    taskEXIT_CRITICAL();
    xTaskNotifyGive(profiling_task_hdl);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    task_switch_profiling_enabled = false;
  }
}

void profiling_task_impl(void*) {
  auto print_stats = []() static {
    static constexpr uint8_t configNUM_TASKS = 10;
    static char stat_buf[50 * configNUM_TASKS];

    vTaskGetRunTimeStats(stat_buf);
    prints("=============================================\n");
    prints("free heap:\t\t%u\n", xPortGetFreeHeapSize());
    prints("ctx switches:\t\t%u\n", ctx_switch_cnt);
    prints("record idx:\t\t%u\n", record_idx);
    prints("Task\t\tTime\t\t%%\n");
    prints("%s", stat_buf);

    prints("---------------------------------------------\n");

    vTaskList(stat_buf);
    prints("Task\t\tState\tPrio\tStack\tNum\n");
    prints("%s", stat_buf);
    prints("=============================================\n");
  };
  // auto print_records = []() static {
  //   prints("Task\t\tTime(ns)\tposition\n");
  //   for (size_t i = 0; i < record_idx; ++i) {
  //     const auto [name, cycle, is_begin] = records[i];
  //     prints("%s\t\t%lu\t%s\n", name,
  //            static_cast<unsigned long>(static_cast<double>(cycle) /
  //                                       SystemCoreClock * 1000 * 1000),
  //            (is_begin ? "in" : "out"));
  //   }
  //   prints("\n");
  // };
  // auto normalize = []() static {
  //   auto initial_value = records.front().cycle;
  //   for (size_t i = 0; i < record_idx; ++i) records[i].cycle -=
  //   initial_value;
  // };

  size_t prev_idx = 0;
  size_t iteration_cycle = 0;
  while (true) {
    if (!task_switch_profiling_enabled) {
      if (iteration_cycle) {
        print_stats();
        prints("output took %u us\n",
               static_cast<unsigned long>(
                   static_cast<double>(DWT->CYCCNT - iteration_cycle) /
                   SystemCoreClock * 1000 * 1000));
        iteration_cycle = 0;
      }

      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      prev_idx = 0;
      iteration_cycle = DWT->CYCCNT;
    }

    vTaskDelay(50);
    while (prev_idx != record_idx) {
      const auto& [name, cycle, is_begin] = records[prev_idx];
      prints("%s\t%lu\t%s\n", name, cycle,
             // static_cast<unsigned long>(static_cast<double>(cycle) /
             //                            SystemCoreClock * 1000 * 1000),
             (is_begin ? "in" : "out"));

      prev_idx = (prev_idx + 1) % records.size();
    }
  }
}
}  // namespace

namespace freertos {
void task_runtime_stats_init() {
  enable_dwt_cycle_count();

  configASSERT((xTaskCreate(button_task_impl, "btn", configMINIMAL_STACK_SIZE,
                            NULL, osPriorityNormal, &button_task_hdl)));
  configASSERT((xTaskCreate(profiling_task_impl, "rt_stats",
                            configMINIMAL_STACK_SIZE * 4, NULL,
                            osPriorityNormal, &profiling_task_hdl) == pdPASS));
}
}  // namespace freertos
