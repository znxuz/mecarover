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
  size_t ticket;
  bool is_begin;

  static inline uint32_t initial_cycle = 0;
};

inline constexpr size_t ISR_STAMP_WRITE_FREQ = 10;
inline constexpr size_t ISR_STAMP_BUF_SIZE = 512;
inline cycle_stamp isr_stamps[ISR_STAMP_BUF_SIZE]{};
volatile inline size_t isr_stamp_idx = 0;
volatile inline bool stamping_enabled = false;

volatile inline std::atomic<size_t> ticket_machine;

inline void stamp_isr(const char* name, bool is_begin) {
  volatile auto cycle = DWT->CYCCNT;
  auto ticket = ticket_machine.fetch_add(1, std::memory_order_acquire);
  isr_stamps[isr_stamp_idx % ISR_STAMP_BUF_SIZE] = {name, cycle, ticket,
                                                    is_begin};
  isr_stamp_idx += 1;
}

inline void stamp(const char* name, bool is_begin) {
  volatile auto cycle = DWT->CYCCNT;
  auto ticket = ticket_machine.fetch_add(1, std::memory_order_acquire);

  char buf[50];  // must be local because multithreaded
  tsink_write_ordered(
      buf,
      snprintf(buf, sizeof(buf), "%s %lu %s\n", name,
               static_cast<unsigned long>(
                   static_cast<double>(cycle - cycle_stamp::initial_cycle) /
                   SystemCoreClock * 1000 * 1000),
               (is_begin ? "in" : "out")),
      ticket);
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
