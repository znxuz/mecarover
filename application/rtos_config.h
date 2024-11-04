#pragma once

/* definitions for FreeRTOS - 0 lowest priority 56 highest priority */

#define POSE_CONTROLLER_PRIORITY 29
#define WHEEL_CONTROLLER_PRIORITY 29
#define MICRO_ROS_TASK_PRIORITY 28
#define SCAN_TASK_PRIORITY 27
#define LOGGER_TASK_PRIORITY 25

#define MAIN_TASK_STACK_SIZE 7500
#define SCAN_STACK_SIZE 3000
