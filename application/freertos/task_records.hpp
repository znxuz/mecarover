#pragma once

#include <FreeRTOS.h>
#include <stm32f767xx.h>
#include <task.h>

#include <array>

namespace freertos {
struct TaskRecord {
  const char* name;
  uint32_t cycle;
  bool is_begin;
} __attribute__((packed));

inline std::array<TaskRecord, 512> records{};
volatile inline size_t record_idx = 0;
volatile inline bool task_switch_profiling_enabled = 0;

inline void record(const char* name, bool is_begin) {
  records[record_idx] = {name, DWT->CYCCNT, is_begin};
  record_idx = (record_idx + 1) % records.size();
}

struct cycle_stamp {
  cycle_stamp(const char* name) : name{name} {
    if (task_switch_profiling_enabled) {
      taskENTER_CRITICAL();
      record(name, true);
      taskEXIT_CRITICAL();
    };
  }
  ~cycle_stamp() {
    if (task_switch_profiling_enabled) {
      taskENTER_CRITICAL();
      record(name, false);
      taskEXIT_CRITICAL();
    }
  }

  const char* name;
};
}  // namespace freertos
