#pragma once

#include <FreeRTOS.h>
#include <stm32f767xx.h>
#include <task.h>
#include <array>

struct TaskRecord;

extern std::array<TaskRecord, 28000> records;
extern volatile size_t record_idx;

extern volatile bool task_switch_profiling_enabled;
extern volatile size_t ctx_switch_cnt;

struct TaskRecord {
  const char* name;
  uint32_t cycle;
  bool is_begin;
} __attribute__((packed));

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
