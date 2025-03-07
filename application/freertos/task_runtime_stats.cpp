#include "task_runtime_stats.hpp"

#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <main.h>
#include <semphr.h>
#include <stdarg.h>
#include <printf.h>

static TaskHandle_t runtime_task_hdl;
static TaskHandle_t button_task_hdl;

SemaphoreHandle_t printf_semphr;

volatile size_t ctx_switch_cnt = 0;

extern "C" {
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  static constexpr uint8_t DEBOUNCE_TIME_MS = 200;
  static volatile uint32_t last_interrupt_time = 0;

  if (GPIO_Pin == USER_Btn_Pin) {
    uint32_t current_time = HAL_GetTick();

    if (current_time - last_interrupt_time > DEBOUNCE_TIME_MS) {
      static BaseType_t xHigherPriorityTaskWoken;
      configASSERT(button_task_hdl != NULL);
      vTaskNotifyGiveFromISR(button_task_hdl, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

      last_interrupt_time = current_time;
    }
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

static void enable_dwt_cycle_count() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = 0xC5ACCE55;  // software unlock
  DWT->CYCCNT = 1;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

int mtxprintf(const char* format, ...) {
  xSemaphoreTake(printf_semphr, portMAX_DELAY);
  va_list args;
  va_start(args, format);
  int result = vprintf(format, args);
  va_end(args);
  xSemaphoreGive(printf_semphr);

  return result;
}

static void runtime_task_impl(void*) {
  auto print_records = []() {
    mtxprintf("Task\t\tTime(ns)\tposition\n");
    for (size_t i = 0; i < record_idx; ++i) {
      const auto [name, cycle, is_begin] = records[i];
      mtxprintf("%s\t\t%lu\t%s\n", name,
                static_cast<unsigned long>(static_cast<double>(cycle) /
                                           SystemCoreClock * 1000 * 1000),
                (is_begin ? "in" : "out"));
    }
    mtxprintf("\n");
  };
  auto print_stats = []() {
    static constexpr uint8_t configNUM_TASKS = 10;
    static char stat_buf[40 * configMAX_TASK_NAME_LEN * configNUM_TASKS];

    vTaskGetRunTimeStats(stat_buf);
    mtxprintf("=============================================\n");
    mtxprintf("free heap:\t\t%u\n", xPortGetFreeHeapSize());
    mtxprintf("ctx switches:\t\t%u\n", ctx_switch_cnt);
    mtxprintf("record idx:\t\t%u\n", record_idx);
    mtxprintf("Task\t\tTime\t\t%%\n");
    mtxprintf("%s", stat_buf);

    mtxprintf("---------------------------------------------\n");

    vTaskList(stat_buf);
    mtxprintf("Task\t\tState\tPrio\tStack\tNum\n");
    mtxprintf("%s", stat_buf);
    mtxprintf("=============================================\n");
  };
  auto normalize = []() {
    auto initial_value = records.front().cycle;
    for (auto& record : records) record.cycle -= initial_value;
  };

  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    normalize();
    print_records();
    print_stats();
  }
}

static void button_task_impl(void*) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    taskENTER_CRITICAL();
    task_switch_profiling_enabled = true;
    record_idx = 0;
    ctx_switch_cnt = 0;
    taskEXIT_CRITICAL();

    vTaskDelay(pdMS_TO_TICKS(1000));  // profile for 1s

    task_switch_profiling_enabled = false;
    xTaskNotifyGive(runtime_task_hdl);
  }
}
}

namespace freertos {
void task_runtime_stats_init() {
  enable_dwt_cycle_count();

  configASSERT((printf_semphr = xSemaphoreCreateMutex()));
  configASSERT((xTaskCreate(button_task_impl, "btn", configMINIMAL_STACK_SIZE,
                            NULL, osPriorityNormal, &button_task_hdl)));
  configASSERT(
      (xTaskCreate(runtime_task_impl, "rt_stats", configMINIMAL_STACK_SIZE * 4,
                   NULL, osPriorityNormal, &runtime_task_hdl) == pdPASS));
}
}  // namespace freertos
