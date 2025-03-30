#pragma once

#include <FreeRTOS.h>
#include <stm32f767xx.h>
#include <task.h>

#include <array>

namespace freertos {
struct cycle_stamp {
  const char* name;
  uint32_t cycle;
  bool is_begin;
} __attribute__((packed));

inline constexpr size_t RT_STAT_TRANSMIT_FREQ = 10;
inline constexpr size_t STAMP_BUF_SIZE = 512;

inline std::array<cycle_stamp, STAMP_BUF_SIZE> stamps{};
volatile inline size_t stamp_idx = 0;
volatile inline bool stamping_enabled = 0;

template <bool from_isr>
inline void stamp(const char* name, bool is_begin) {
  struct disable_isr_guard {
    disable_isr_guard() { taskENTER_CRITICAL(); }
    ~disable_isr_guard() { taskEXIT_CRITICAL(); }
  };

  if constexpr (!from_isr) volatile auto _ = disable_isr_guard();

  stamps[stamp_idx] = {name, DWT->CYCCNT, is_begin};
  stamp_idx = (stamp_idx + 1) % stamps.size();
}

struct cycle_stamp_raii {
  cycle_stamp_raii(const char* name) : name{name} {
    if (stamping_enabled) stamp<false>(name, true);
  }
  ~cycle_stamp_raii() {
    if (stamping_enabled) stamp<false>(name, false);
  }

  const char* name;
};
}  // namespace freertos
