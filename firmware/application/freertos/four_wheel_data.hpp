#pragma once

#include <application/robot_params.hpp>

namespace freertos {

struct FourWheelData {
  double front_right;
  double front_left;
  double back_left;
  double back_right;

  FourWheelData() = default;

  FourWheelData(const std::array<double, robot_params::N_WHEEL>& data)
      : front_right{data[0]},
        front_left{data[1]},
        back_left{data[2]},
        back_right{data[3]} {}

  FourWheelData(const robot_params::VelWheel& vel) {
    front_right = vel(0);
    front_left = vel(1);
    back_left = vel(2);
    back_right = vel(3);
  }

  operator robot_params::VelWheel() {
    return robot_params::VelWheel{front_right, front_left, back_left,
                                  back_right};
  }
};

}  // namespace freertos
