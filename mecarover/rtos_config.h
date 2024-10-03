#pragma once

/* definitions for FreeRTOS - 0 lowest priority 56 highest priority */

#define POSE_CONTROLLER_PRIORITY 30
#define WHEEL_CONTROLLER_PRIORITY 29
#define MICRO_ROS_TASK_PRIORITY 28
#define LaserScanner_TASK_PRIORITY 27
#define LOGGER_TASK_PRIORITY 25
#define ETH_TASK_PRIORITY 24
#define LWIP_TASK_PRIORITY 23
#define STACK_SIZE 7500
#define LWIP_STACK_SIZE 1500
#define ETH_STACK_SIZE 1500
