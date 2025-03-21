#pragma once

#include <cstddef>

#ifndef STREAMBUF_SIZE
static constexpr size_t STREAMBUF_SIZE = 2048;
#endif

extern "C" {
void uq_write(const char* ptr, size_t len);
void transfer_cplt_callback(void);
}

namespace freertos {
void task_uart_streambuf_init();
}
