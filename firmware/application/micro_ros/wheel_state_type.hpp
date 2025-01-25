#pragma once

#include <ulog.h>

#include <optional>
#include <string_view>

inline constexpr std::string_view DELTA_PHI_SV =
    "wheel encoder delta in radian";
inline constexpr std::string_view VEL_RAD_SV =
    "wheel angular velocity setpoint";

enum class WheelStateType { DELTA_PHI, VEL_RAD };

inline constexpr std::optional<WheelStateType> parse_wheel_state(
    std::string_view sv) {
  if (sv == DELTA_PHI_SV) {
    return WheelStateType::DELTA_PHI;
  } else if (sv == VEL_RAD_SV) {
    return WheelStateType::VEL_RAD;
  }

  ULOG_ERROR("at %s in %d : parsing unknown wheel state type", __FILE__,
             __LINE__);
  return {};
}

inline constexpr std::optional<std::string_view> to_string(
    WheelStateType type) {
  switch (type) {
    case WheelStateType::DELTA_PHI:
      return DELTA_PHI_SV;
    case WheelStateType::VEL_RAD:
      return VEL_RAD_SV;
  }

  ULOG_ERROR("at %s in %d : parsing unknown wheel state type", __FILE__,
             __LINE__);
  return {};
}

inline constexpr bool operator==(const WheelStateType& e, std::string_view sv) {
  switch (e) {
    case WheelStateType::DELTA_PHI:
      return sv == DELTA_PHI_SV;
    case WheelStateType::VEL_RAD:
      return sv == VEL_RAD_SV;
  }
  return false;
}

inline constexpr bool operator==(std::string_view sv, const WheelStateType& e) {
  return e == sv;
}
