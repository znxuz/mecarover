/*
 * this header defines a C interface to ros debug messages
 * it is implemented in the C++ component ros_interface
 */
 
#pragma once
#ifndef ROS_DEBUG_H
#define ROS_DEBUG_H

#include <mrlogger.h>

#if __cplusplus
extern "C" {
#endif

void ros_log_message(mr_logprio_t prio, const char* msg);

#if __cplusplus
}
#endif

#endif // INCLUDED__H
