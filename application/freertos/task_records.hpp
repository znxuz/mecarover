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
volatile inline bool task_profiling_enabled = 0;

template <bool from_isr>
inline void record(const char* name, bool is_begin) {
  struct disable_isr_guard {
    disable_isr_guard() { taskENTER_CRITICAL(); }
    ~disable_isr_guard() { taskEXIT_CRITICAL(); }
  };

  if constexpr (!from_isr) volatile auto _ = disable_isr_guard();

  records[record_idx] = {name, DWT->CYCCNT, is_begin};
  record_idx = (record_idx + 1) % records.size();
}

struct cycle_stamp {
  cycle_stamp(const char* name) : name{name} {
    if (task_profiling_enabled) record<false>(name, true);
  }
  ~cycle_stamp() {
    if (task_profiling_enabled) record<false>(name, false);
  }

  const char* name;
};
}  // namespace freertos
