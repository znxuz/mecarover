#pragma once

#include <cstdint>
#include "application/robot_params.hpp"

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

  FourWheelData() = default;

  FourWheelData(const std::array<double, robot_params::N_WHEEL>& data,
                FourWheelDataType type)
      : front_right{data[0]},
        front_left{data[1]},
        back_left{data[2]},
        back_right{data[3]},
        type{type} {}
};

}  // namespace freertos
