#pragma once

#include <cstdint>

namespace freertos {

enum struct FourWheelDataType : uint32_t {
  ENC_DELTA_RAD,
  VEL_RAD_PV,
  VEL_RAD_SP,
};

struct FourWheelData {
  double front_right;
  double front_left;
  double back_left;
  double back_right;
  FourWheelDataType type;
};

}
