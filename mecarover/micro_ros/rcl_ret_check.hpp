#pragma once

#include <FreeRTOS.h>
#include <mecarover/mrlogger/mrlogger.h>
#include <rcl/types.h>
#include <task.h>

#include <source_location>

static inline void rcl_ret_check(
    rcl_ret_t ret_code,
    const std::source_location location = std::source_location::current()) {
  if (ret_code) {
    log_message(log_error, "Failed status on line %d: %d in %s. Aborting.",
                static_cast<int>(location.line()), static_cast<int>(ret_code),
                location.file_name());
    vTaskDelete(NULL);
  }
}

static inline void rcl_ret_softcheck(
    rcl_ret_t ret_code,
    const std::source_location location = std::source_location::current()) {
  if (ret_code)
    log_message(log_warning, "Failed status on line %d: %d in %s. Continuing.",
                static_cast<int>(location.line()), static_cast<int>(ret_code),
                location.file_name());
}
