#pragma once

// definitions for RTOS - 0 lowest Priority 56 highest Priority
#define POSE_CONTROLLER_PRIORITY 30 //highest Priority
#define WHEEL_CONTROLLER_PRIORITY 29
#define MICRO_ROS_TASK_PRIORITY 28
#define LaserScanner_TASK_PRIORITY 27 //defined LaserScanner.c, Stack Size = 3000
#define MAIN_TASK_PRIORITY 26//27 dunno why this macro should be used
#define LOGGER_TASK_PRIORITY 25
#define ETH_TASK_PRIORITY 24 //defined ethernetif.c
#define LWIP_TASK_PRIORITY 23 //defined lwip.c, lowest Priority, Normal Prio - 56 is max, Priority defined in cmsis_os2.h
#define STACK_SIZE 7500 //20000 Stack Size - Specified in words, 1 word = 4 bytes
#define LWIP_STACK_SIZE 1500
#define ETH_STACK_SIZE 1500
//StackSize can be set in STMCubeMX and FreeRTOSConfig.h at configTOTAL_HEAP_SIZE
