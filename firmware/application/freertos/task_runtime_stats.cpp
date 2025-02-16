#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <task.h>

#define configNUM_TASKS 6
#define STATS_BUFFER_SIZE (40 * configMAX_TASK_NAME_LEN * configNUM_TASKS)

static TaskStatus_t task_status;

void task_switched_in_callback() {
  vTaskGetInfo(xTaskGetCurrentTaskHandle(), &task_status, false, eRunning);
}

static TaskHandle_t task_handle;

static char stat_buf[STATS_BUFFER_SIZE];

extern "C" {
static void task_impl(void *) {
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    vTaskGetRunTimeStats(stat_buf);
    printf("Task\t\tTime\t\t%%\n");
    printf("%s\n", stat_buf);

    vTaskList(stat_buf);
    printf("Task\tState\tPrio\tStack\tNum\n");
    printf("%s\n", stat_buf);

    // TODO get task switch infos and timestamps

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
}

namespace freertos {
void task_runtime_stats_init() {
  configASSERT(xTaskCreate(task_impl, "runtime_stats",
                           configMINIMAL_STACK_SIZE * 4, NULL,
                           osPriorityNormal1, &task_handle) == pdPASS);
}
}  // namespace freertos
