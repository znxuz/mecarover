#pragma once

#include <FreeRTOS.h>
#include <rcl/types.h>
#include <task.h>

#include <source_location>

#include "ulog.h"

static inline void rcl_ret_check(
    rcl_ret_t ret_code,
    const std::source_location location = std::source_location::current()) {
  if (ret_code) {
    ULOG_ERROR("Failed status on line %d: %d in %s. Aborting.",
               static_cast<int>(location.line()), static_cast<int>(ret_code),
               location.file_name());
    vTaskDelete(NULL);
  }
}

static inline void rcl_ret_softcheck(
    rcl_ret_t ret_code,
    const std::source_location location = std::source_location::current()) {
  if (ret_code)
    ULOG_WARNING("Failed status on line %d: %d in %s. Continuing.",
                 static_cast<int>(location.line()), static_cast<int>(ret_code),
                 location.file_name());
}
