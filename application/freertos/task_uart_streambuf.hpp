#pragma once

extern "C" {
void uart_write(const char* ptr, int len);
}

namespace freertos {
void task_uart_streambuf_init();
}
