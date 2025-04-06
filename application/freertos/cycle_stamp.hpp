#pragma once

#include <FreeRTOS.h>
#include <printf.h>
#include <stm32f767xx.h>
#include <task.h>

#include <atomic>
#include <threadsafe_sink.hpp>

namespace freertos {
struct cycle_stamp {
  const char* name;
  uint32_t cycle;
  uint32_t ticket;
  bool is_begin;

  inline static uint32_t initial_cycle = 0;
} __attribute__((packed));

inline constexpr size_t RT_STAT_TRANSMIT_FREQ = 5;
inline constexpr size_t STAMP_BUF_SIZE = 8092;

volatile inline bool stamping_enabled{};

inline cycle_stamp isr_stamps[STAMP_BUF_SIZE];
volatile inline size_t isr_stamp_idx{};

template <TSINK_CALL_FROM callsite>
inline void stamp(const char* name, bool is_begin) {
  // struct disable_irq_guard {
  //   disable_irq_guard() { taskENTER_CRITICAL(); }
  //   ~disable_irq_guard() { taskEXIT_CRITICAL(); }
  // };
  static std::atomic<size_t> ticket_machine;

  auto stamp =
      cycle_stamp{name, DWT->CYCCNT, ticket_machine.fetch_add(1), is_begin};
  if constexpr (callsite == TSINK_CALL_FROM::ISR) {
    isr_stamps[isr_stamp_idx] = stamp;
    isr_stamp_idx = (isr_stamp_idx + 1) % STAMP_BUF_SIZE;
  } else {
    static char buf[50];
    const auto& [name, cycle, ticket, is_begin] = stamp;
    tsink_write_ordered(
        buf,
        snprintf(buf, sizeof(buf), "%s %lu %s\n", name,
                 static_cast<unsigned long>(
                     static_cast<double>(cycle - cycle_stamp::initial_cycle) /
                     SystemCoreClock * 1000 * 1000),
                 (is_begin ? "in" : "out")),
        ticket);
  }
}

struct cycle_stamp_raii {
  cycle_stamp_raii(const char* name) : name{name} {
    if (stamping_enabled) stamp<TSINK_CALL_FROM::NON_ISR>(name, true);
  }
  ~cycle_stamp_raii() {
    if (stamping_enabled) stamp<TSINK_CALL_FROM::NON_ISR>(name, false);
  }

  const char* name;
};
}  // namespace freertos
