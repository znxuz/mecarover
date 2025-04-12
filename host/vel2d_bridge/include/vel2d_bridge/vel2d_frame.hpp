#pragma once

#include <cstdint>

struct Vel2d {
  float x;
  float y;
  float z;
};

struct Vel2dFrame {
  Vel2d vel;
  uint32_t crc;

  Vel2dFrame(Vel2d vel);
  const uint8_t* data() const { return reinterpret_cast<const uint8_t*>(this); }
  bool compare(uint32_t rhs) { return crc == rhs; }
};

inline constexpr std::size_t VEL2D_FRAME_LEN = sizeof(Vel2dFrame);

static_assert(sizeof(Vel2d) == 3 * sizeof(float));
static_assert(VEL2D_FRAME_LEN == sizeof(Vel2d) + sizeof(uint32_t),
              "Vel2dFrame size incorrect!");
