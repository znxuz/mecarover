#pragma once

#include <stdbool.h>

// TODO: refactor as uros module

#if __cplusplus
extern "C" {
#endif

typedef enum mr_logprio_t {
  log_error = 31,
  log_debug = 32,
  log_warning = 33,
  log_info = 34,
} mr_logprio_t;

void log_message(mr_logprio_t prio, const char* fmt, ...)
    __attribute__((__format__(__printf__, 2, 3)));
void logger_init();
void logger_use_colors(bool use_colors);

#if __cplusplus
}
#endif
