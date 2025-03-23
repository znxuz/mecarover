#pragma once

#include <cstddef>
#include <cstdint>

#ifndef RING_BUF_SIZE
static constexpr size_t RING_BUF_SIZE = 2048;
#endif

typedef void (*consume_function)(const uint8_t* buf, size_t size);

extern "C" {
// write `len` from `ptr` buffer into the queue
void uq_write(const char* ptr, size_t len);

// callback upon consume completion to signal the queue task
void uq_consume_complete(void);
}

namespace freertos {
// init function taking a function pointer to consume the bytes from the queue
void task_uart_queue_init(consume_function f);
}  // namespace freertos
