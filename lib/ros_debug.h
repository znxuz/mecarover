/*
 * this header defines a C interface to ros debug messages
 * it is implemented in the C++ component ros_interface
 */
 
#pragma once

#include <mecarover/mrlogger/mrlogger.h>

#if __cplusplus
extern "C" {
#endif

void ros_log_message(mr_logprio_t prio, const char* msg);

#if __cplusplus
}
#endif
