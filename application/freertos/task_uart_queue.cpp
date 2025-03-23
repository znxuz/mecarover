#include "task_uart_queue.hpp"

#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <semphr.h>
#include <task.h>

static SemaphoreHandle_t task_semphr;
static SemaphoreHandle_t write_semphr;
static TaskHandle_t task_hdl;

static uint8_t ring_buf[RING_BUF_SIZE];
static bool consumable[RING_BUF_SIZE];
static volatile size_t write_idx;

static consume_function consume;

extern "C" {
void uq_write(const char* ptr, size_t len) {
  xSemaphoreTake(write_semphr, portMAX_DELAY);
  for (size_t i = 0; i < len; ++i) {
    while (consumable[write_idx]) vTaskDelay(1);

    ring_buf[write_idx] = ptr[i];
    consumable[write_idx] = true;
    write_idx = (write_idx + 1) % RING_BUF_SIZE;
  }
  xSemaphoreGive(task_semphr);
  xSemaphoreGive(write_semphr);
}

void uq_consume_complete(void) {
  static BaseType_t xHigherPriorityTaskWoken;
  vTaskNotifyGiveFromISR(task_hdl, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
}

namespace {
inline void consume_and_wait(size_t pos, size_t size) {
  auto update_for_writer = [](size_t pos, size_t size) static {
    for (size_t i = 0; i < size; ++i) consumable[pos++] = false;
  };

  consume(ring_buf + pos, size);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  update_for_writer(pos, size);
}

void task_uart_consume(void*) {
  size_t start = 0;
  while (true) {
    xSemaphoreTake(task_semphr, portMAX_DELAY);

    auto end = write_idx;
    if (start == end) continue;  // just for safety

    auto size = (start < end ? end : RING_BUF_SIZE) - start;
    consume_and_wait(start, size);
    if (start > end && end) consume_and_wait(0, end);

    start = end;
  }
}
}  // namespace

namespace freertos {
void task_uart_queue_init(consume_function f) {
  consume = f;

  // TODO statically allocate
  configASSERT((write_semphr = xSemaphoreCreateMutex()));
  configASSERT((task_semphr = xSemaphoreCreateBinary()));
  configASSERT((xTaskCreate(task_uart_consume, "uart_consume",
                            configMINIMAL_STACK_SIZE * 4, NULL,
                            osPriorityAboveNormal, &task_hdl) == pdPASS));
}
}  // namespace freertos
