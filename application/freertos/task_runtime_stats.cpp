#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <main.h>
#include <stdio.h>
#include <stm32f767xx.h>
#include <task.h>

#include <string_view>
#include <utility>

static TaskHandle_t task_handle;

struct TaskRecord {
  const char* name;
  uint32_t timestamp;
  bool is_begin;
} __attribute__((packed));

static TaskRecord task_records[1000];
static size_t task_records_idx;
static volatile bool task_switch_profiling_enabled;
static size_t switch_cnt;

struct timestamp {
  timestamp(std::string_view name) : name{name} {
    if (task_switch_profiling_enabled) {
      taskENTER_CRITICAL();
      task_records[task_records_idx++] = {name.data(), DWT->CYCCNT, true};
      taskEXIT_CRITICAL();
    }
  }
  ~timestamp() {
    if (task_switch_profiling_enabled) {
      taskENTER_CRITICAL();
      task_records[task_records_idx++] = {name.data(), DWT->CYCCNT, false};
      taskEXIT_CRITICAL();
    }
  }

  std::string_view name;
};

extern "C" {
[[maybe_unused]] static void print_stats(void) {
  static constexpr uint8_t configNUM_TASKS = 10;
  static char stat_buf[40 * configMAX_TASK_NAME_LEN * configNUM_TASKS];

  vTaskGetRunTimeStats(stat_buf);
  puts("==========================================");
  printf("Task\t\tTime\t\t%%\n");
  printf("%s\n", stat_buf);

  vTaskList(stat_buf);
  printf("Task\t\tState\tPrio\tStack\tNum\n");
  printf("%s\n", stat_buf);
}

static void task_impl(void*) {
  // const TickType_t xFrequency = pdMS_TO_TICKS(1000);
  // TickType_t xLastWakeTime = xTaskGetTickCount();
  //
  // while (true) {
  //   // print_stats();
  //
  //   vTaskDelayUntil(&xLastWakeTime, xFrequency);
  // }

  bool print_once = false;
  while (true) {
    if (!task_switch_profiling_enabled) {
      if (std::exchange(print_once, true)) continue;

      // taskENTER_CRITICAL();
      // auto idx_cp = task_records_idx;
      // taskEXIT_CRITICAL();

      // for (size_t i = 0; i < idx_cp; ++i) {
      //   const auto& [name, timestamp, is_begin] = task_records[i];
      //   printf("%s:\t%s\t%09u\n%s", (is_begin ? "begin" : "end"), name,
      //          timestamp, (i + 1 == idx_cp) ? "\n" : "");
      // }

      printf("switch_cnt = %u\n", switch_cnt);
    } else {
      print_once = false;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void enable_dwt_cycle_count() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = 0xC5ACCE55;  // software unlock
  DWT->CYCCNT = 1;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

TaskHandle_t button_task_handle;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  static constexpr uint8_t DEBOUNCE_TIME_MS = 200;
  static volatile uint32_t last_interrupt_time = 0;

  if (GPIO_Pin == USER_Btn_Pin) {
    uint32_t current_time = HAL_GetTick();

    if (current_time - last_interrupt_time > DEBOUNCE_TIME_MS) {
      static BaseType_t xHigherPriorityTaskWoken;
      configASSERT(button_task_handle != NULL);
      vTaskNotifyGiveFromISR(button_task_handle, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

      last_interrupt_time = current_time;
    }
  }
}
void button_task(void*) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    taskENTER_CRITICAL();
    task_switch_profiling_enabled ^= 1;
    if (task_switch_profiling_enabled) task_records_idx = switch_cnt = 0;
    taskEXIT_CRITICAL();
  }
}

void task_switched_in_callback(const char* name) {
  if (!task_switch_profiling_enabled) return;

  // task_records[task_records_idx].is_begin = true;
  // task_records[task_records_idx].name = name;
  // task_records[task_records_idx++].timestamp = DWT->CYCCNT;

  ++switch_cnt;
}

void task_switched_out_callback(const char* name) {
  if (!task_switch_profiling_enabled) return;

  // task_records[task_records_idx].name = name;
  // task_records[task_records_idx++].timestamp = DWT->CYCCNT;
  ++switch_cnt;
}
}

namespace freertos {
void task_runtime_stats_init() {
  enable_dwt_cycle_count();

  constexpr size_t STACK_SIZE = configMINIMAL_STACK_SIZE * 4;
#ifdef FREERTOS_STATIC_INIT
  static StackType_t taskStack[STACK_SIZE];
  static StaticTask_t taskBuffer;
  configASSERT((task_handle = xTaskCreateStatic(
                    task_impl, "runtime_stats", STACK_SIZE, NULL,
                    osPriorityNormal, taskStack, &taskBuffer)) != NULL);
#else
  configASSERT(xTaskCreate(task_impl, "runtime_stats", STACK_SIZE, NULL,
                           osPriorityNormal, &task_handle) == pdPASS);
  configASSERT(xTaskCreate(button_task, "button", configMINIMAL_STACK_SIZE,
                           NULL, osPriorityNormal, &button_task_handle));
#endif
}
}  // namespace freertos
