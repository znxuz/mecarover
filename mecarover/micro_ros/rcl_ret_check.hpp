#pragma once

#include "mecarover/mrlogger/mrlogger.h"
#include <FreeRTOS.h>
#include <experimental/source_location>
#include <rcl/types.h>
#include <task.h>

static inline void
rcl_ret_check(rcl_ret_t ret_code,
			  const std::experimental::source_location location
			  = std::experimental::source_location::current())
{
	if (ret_code) {
		log_message(log_error, "Failed status on line %d: %d in %s. Aborting.",
					static_cast<int>(location.line()),
					static_cast<int>(ret_code), location.file_name());
		vTaskDelete(NULL);
	}
}

static inline void
rcl_ret_softcheck(rcl_ret_t ret_code,
				  const std::experimental::source_location location
				  = std::experimental::source_location::current())
{
	if (ret_code)
		log_message(log_warning,
					"Failed status on line %d: %d in %s. Continuing.\n",
					static_cast<int>(location.line()),
					static_cast<int>(ret_code), location.file_name());
}
