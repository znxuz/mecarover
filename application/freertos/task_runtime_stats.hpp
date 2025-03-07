#pragma once

#include <FreeRTOS.h>
#include <stm32f767xx.h>
#include <task.h>

#include <array>

struct TaskRecord {
  const char* name;
  uint32_t cycle;
  bool is_begin;
} __attribute__((packed));

inline std::array<TaskRecord, 28000> records{};
volatile inline size_t record_idx = 0;

volatile inline bool task_switch_profiling_enabled = 0;

struct cycle_stamp {
  cycle_stamp(const char* name) : name{name} {
    if (task_switch_profiling_enabled) {
      taskENTER_CRITICAL();
      records[record_idx] = {name, DWT->CYCCNT, true};
      record_idx += 1;
      taskEXIT_CRITICAL();
    }
  }
  ~cycle_stamp() {
    if (task_switch_profiling_enabled) {
      taskENTER_CRITICAL();
      records[record_idx] = {name, DWT->CYCCNT, false};
      record_idx += 1;
      taskEXIT_CRITICAL();
    }
  }

  const char* name;
};
