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
static TaskHandle_t task_hdl;

static constexpr size_t BUF_SIZE = 2048;
static uint8_t ring_buf[BUF_SIZE];
static volatile size_t write_idx;

static uint8_t read_iteration[BUF_SIZE];

extern "C" {
void rbuf_write(const char* ptr, size_t len) {
  static uint8_t write_iteration;
  for (size_t i = 0; i < len; ++i) {
    if (!write_iteration)  // handle overflow
      while (write_iteration != read_iteration[write_idx]) vTaskDelay(1);
    else
      while (write_iteration > read_iteration[write_idx]) vTaskDelay(1);
    ring_buf[write_idx] = ptr[i];

    taskENTER_CRITICAL();
    write_idx = (write_idx + 1) % BUF_SIZE;
    write_iteration += !write_idx;
    taskEXIT_CRITICAL();

    xSemaphoreGive(task_semphr);
  }
}

void _putchar(char c) { rbuf_write((const char*)&c, 1); }

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance != huart3.Instance) return;

  static BaseType_t xHigherPriorityTaskWoken;
  vTaskNotifyGiveFromISR(task_hdl, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
}

namespace {

void flush_cache_aligned(uintptr_t addr, size_t size) {
  constexpr auto align_addr = [](uintptr_t addr) { return addr & ~0x1F; };
  constexpr auto align_size = [](uintptr_t addr, size_t size) {
    return size + ((addr) & 0x1F);
  };

  SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(align_addr(addr)),
                          align_size(addr, size));
}

void write_and_wait(size_t pos, size_t size) {
  flush_cache_aligned(reinterpret_cast<uintptr_t>(ring_buf + pos), size);
  HAL_UART_Transmit_DMA(&huart3, ring_buf + pos, size);

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

void update_read_iteration(size_t pos, size_t size) {
  for (size_t i = 0; i < size; ++i) read_iteration[pos++] += 1;
}

void task_uart_streambuf(void*) {
  size_t start = 0;
  while (true) {
    xSemaphoreTake(task_semphr, portMAX_DELAY);

    auto end = write_idx;
    if (start == end) continue;  // just for safety

    auto size = (start < end ? end : BUF_SIZE) - start;
    write_and_wait(start, size);
    update_read_iteration(start, size);

    if (start > end && end) {
      write_and_wait(0, end);
      update_read_iteration(0, end);
    }

    start = end;
  }
}
}  // namespace

namespace freertos {
void task_uart_streambuf_init() {
  configASSERT((task_semphr = xSemaphoreCreateBinary()));
  configASSERT((xTaskCreate(task_uart_streambuf, "uart_streambuf",
                            configMINIMAL_STACK_SIZE * 4, NULL,
                            osPriorityNormal, &task_hdl) == pdPASS));
}
}  // namespace freertos
