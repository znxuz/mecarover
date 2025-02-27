#include "task_runtime_stats.hpp"

#include <cmsis_os2.h>
#include <main.h>
#include <stdio.h>

static TaskHandle_t runtime_task_hdl;
static TaskHandle_t button_task_hdl;

std::array<TaskRecord, 28000> records{};
volatile size_t record_idx = 0;

volatile bool task_switch_profiling_enabled = 0;
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

[[maybe_unused]] static void print_stats(void) {
  static constexpr uint8_t configNUM_TASKS = 10;
  static char stat_buf[40 * configMAX_TASK_NAME_LEN * configNUM_TASKS];

  vTaskGetRunTimeStats(stat_buf);
  puts("=============================================");
  printf("free heap:\t\t%u\n", xPortGetFreeHeapSize());
  printf("ctx switches:\t\t%u\n", ctx_switch_cnt);
  printf("record idx:\t\t%u\n", record_idx);
  puts("Task\t\tTime\t\t%%");
  printf("%s", stat_buf);

  puts("---------------------------------------------");

  vTaskList(stat_buf);
  puts("Task\t\tState\tPrio\tStack\tNum");
  printf("%s", stat_buf);
  puts("=============================================\n");
}

[[maybe_unused]] static void print_records(void) {
  puts("Task\t\tt_stamp\tposition");
  for (size_t i = 0; i < record_idx; ++i) {
    const auto [name, cycle, is_begin] = records[i];
    printf("%s\t\t%u\t%s\n", name, cycle, (is_begin ? "in" : "out"));
  }
}

static void runtime_task_impl(void*) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    print_records();
    puts("");
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

    vTaskDelay(pdMS_TO_TICKS(1000));

    task_switch_profiling_enabled = false;
    xTaskNotifyGive(runtime_task_hdl);
  }
}
}

namespace freertos {
void task_runtime_stats_init() {
  enable_dwt_cycle_count();

  for (auto& record : records) {
    record.cycle = DWT->CYCCNT;
  }

  configASSERT(xTaskCreate(runtime_task_impl, "rt_stats",
                           configMINIMAL_STACK_SIZE * 4, NULL, osPriorityNormal,
                           &runtime_task_hdl) == pdPASS);
  configASSERT(xTaskCreate(button_task_impl, "btn", configMINIMAL_STACK_SIZE,
                           NULL, osPriorityNormal, &button_task_hdl));
}
}  // namespace freertos
