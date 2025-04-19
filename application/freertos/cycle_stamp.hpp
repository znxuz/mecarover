#pragma once

#include <FreeRTOS.h>
#include <printf.h>
#include <stm32f767xx.h>

#include <atomic>
#include <threadsafe_sink.hpp>

namespace freertos {
struct cycle_stamp {
  const char* name;
  size_t cycle;
  bool is_begin;

  static inline uint32_t initial_cycle = 0;
};

inline constexpr size_t STAMP_OUTPUT_FREQ = 10;
inline constexpr size_t STAMP_BUF_SIZE = 512;
inline cycle_stamp stamps[STAMP_BUF_SIZE]{};
volatile inline std::atomic<size_t> stamp_idx = 0;
volatile inline bool stamping_enabled = false;

inline uint32_t cycle_to_us(uint32_t cycle) {
  return static_cast<uint32_t>(static_cast<float>(cycle) / SystemCoreClock *
                               1000 * 1000);
}

inline void stamp(const char* name, bool is_begin) {
  volatile auto cycle = DWT->CYCCNT;
  volatile auto idx = stamp_idx.fetch_add(1);
  stamps[idx % STAMP_BUF_SIZE] = {name, cycle, is_begin};
}

inline void stamp_direct(const char* name, bool is_begin) {
  char buf[50];  // local because multithreaded, otherwise race condition baby
  volatile auto cycle = DWT->CYCCNT;
  tsink::write_blocking(
      buf, snprintf(buf, sizeof(buf), "%s %u %u\n", name,
                    cycle_to_us(cycle - cycle_stamp::initial_cycle), is_begin));
  ;
}

struct cycle_stamp_raii {
  cycle_stamp_raii(const char* name) : name{name} {
    if (stamping_enabled) stamp(name, true);
  }
  ~cycle_stamp_raii() {
    if (stamping_enabled) stamp(name, false);
  }

  const char* name;
};
}  // namespace freertos
