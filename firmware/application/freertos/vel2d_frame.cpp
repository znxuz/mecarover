#include "vel2d_frame.hpp"

#include <crc.h>

#include <utility>

Vel2dFrame::Vel2dFrame(Vel2d vel)
    : vel{std::move(vel)},
      // first cast vel to u8 array to suppress compiler warning
      crc{HAL_CRC_Calculate(&hcrc, (uint32_t*)(uint8_t*)&vel, sizeof(Vel2d))} {}
