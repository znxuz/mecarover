#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <stdio.h>
#include <task.h>

static TaskHandle_t task_handle;

static constexpr uint8_t configNUM_TASKS = 10;
static char stat_buf[40 * configMAX_TASK_NAME_LEN * configNUM_TASKS];

extern "C" {
static void print_stats(void) {
  vTaskGetRunTimeStats(stat_buf);
  puts("==========================================");
  printf("Task\t\tTime\t\t%%\n");
  printf("%s\n", stat_buf);

  vTaskList(stat_buf);
  printf("Task\t\tState\tPrio\tStack\tNum\n");
  printf("%s\n", stat_buf);
}

static void task_impl(void *) {
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    print_stats();

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
}

namespace freertos {
void task_runtime_stats_init() {
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
#endif
}
}  // namespace freertos
