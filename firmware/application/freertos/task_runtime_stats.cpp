#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <task.h>

#include "main.h"

static TaskHandle_t task_handle;

static constexpr uint8_t configNUM_TASKS = 6;
static char stat_buf[40 * configMAX_TASK_NAME_LEN * configNUM_TASKS];

static TaskStatus_t task_status;
static size_t switched_cnt;

static volatile bool sample = false;

extern "C" {
void task_switched_in_callback() {
  vTaskGetInfo(xTaskGetCurrentTaskHandle(), &task_status, false, eRunning);
  if (sample) ++switched_cnt;
  // TODO maybe directly copy the needed name & timestamp into a container
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  static constexpr uint8_t DEBOUNCE_TIME_MS = 200;
  static volatile uint32_t last_interrupt_time = 0;

  if (GPIO_Pin == USER_Btn_Pin) {
    uint32_t current_time = HAL_GetTick();

    if (current_time - last_interrupt_time > DEBOUNCE_TIME_MS) {
      sample = !sample;

      last_interrupt_time = current_time;
    }
  }
}

static void print_stats(void) {
  vTaskGetRunTimeStats(stat_buf);
  printf("Task\t\tTime\t\t%%\n");
  printf("%s\n", stat_buf);

  vTaskList(stat_buf);
  printf("Task\tState\tPrio\tStack\tNum\n");
  printf("%s\n", stat_buf);
}

static void task_impl(void *) {
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    // print_stats();

    printf("switched_cnt: %u\n", switched_cnt);

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
