#include "task_uart_queue.hpp"

#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <semphr.h>
#include <task.h>
#include <unistd.h>
#include <usart.h>

#include <cerrno>
#include <cstdarg>
#include <cstdio>

static SemaphoreHandle_t task_semphr;
static SemaphoreHandle_t write_semphr;
static TaskHandle_t task_hdl;

static uint8_t ring_buf[STREAMBUF_SIZE];
static bool consumable[STREAMBUF_SIZE];

static volatile size_t write_idx;

extern "C" {
void uq_write(const char* ptr, size_t len) {
  xSemaphoreTake(write_semphr, portMAX_DELAY);
  for (size_t i = 0; i < len; ++i) {
    while (consumable[write_idx]) vTaskDelay(1);

    ring_buf[write_idx] = ptr[i];
    consumable[write_idx] = true;
    write_idx = (write_idx + 1) % STREAMBUF_SIZE;
  }
  xSemaphoreGive(task_semphr);
  xSemaphoreGive(write_semphr);
}

void transfer_cplt_callback(void) {
  static BaseType_t xHigherPriorityTaskWoken;
  vTaskNotifyGiveFromISR(task_hdl, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
}

namespace {
inline void write_and_wait(size_t pos, size_t size) {
  auto flush_cache_aligned = [](uintptr_t addr, size_t size) static {
    constexpr auto align_addr = [](uintptr_t addr) { return addr & ~0x1F; };
    constexpr auto align_size = [](uintptr_t addr, size_t size) {
      return size + ((addr) & 0x1F);
    };

    SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(align_addr(addr)),
                            align_size(addr, size));
  };
  auto update_for_writer = [](size_t pos, size_t size) static {
    for (size_t i = 0; i < size; ++i) consumable[pos++] = false;
  };

  flush_cache_aligned(reinterpret_cast<uintptr_t>(ring_buf + pos), size);
  HAL_UART_Transmit_DMA(&huart3, ring_buf + pos, size);

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  update_for_writer(pos, size);
}

void task_uart_streambuf(void*) {
  size_t start = 0;
  while (true) {
    xSemaphoreTake(task_semphr, portMAX_DELAY);

    auto end = write_idx;
    if (start == end) continue;  // just for safety

    auto size = (start < end ? end : STREAMBUF_SIZE) - start;
    write_and_wait(start, size);
    if (start > end && end) write_and_wait(0, end);

    start = end;
  }
}
}  // namespace

namespace freertos {
void task_uart_streambuf_init() {
  // TODO statically allocate
  configASSERT((write_semphr = xSemaphoreCreateMutex()));
  configASSERT((task_semphr = xSemaphoreCreateBinary()));
  configASSERT((xTaskCreate(task_uart_streambuf, "uart_streambuf",
                            configMINIMAL_STACK_SIZE * 4, NULL,
                            osPriorityAboveNormal, &task_hdl) == pdPASS));
}
}  // namespace freertos
