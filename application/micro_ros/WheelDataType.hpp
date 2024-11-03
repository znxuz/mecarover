#pragma once

#include <ulog.h>

#include <optional>
#include <string_view>

inline constexpr std::string_view STR_ENC_DELTA_RAD =
    "wheel encoder delta in radian";
inline constexpr std::string_view STR_VEL_SP = "wheel velocity setpoint";

enum class WheelDataType { ENC_DELTA_RAD, VEL_SP };

inline constexpr std::optional<WheelDataType> parse_wheel_data_type(
    std::string_view sv) {
  if (sv == STR_ENC_DELTA_RAD) {
    return WheelDataType::ENC_DELTA_RAD;
  } else if (sv == STR_VEL_SP) {
    return WheelDataType::VEL_SP;
  }

  ULOG_ERROR("at %s in %d : parsing unknown wheel data type", __FILE__,
             __LINE__);
  return {};
}

inline constexpr std::optional<std::string_view> to_string(WheelDataType type) {
  switch (type) {
    case WheelDataType::ENC_DELTA_RAD:
      return STR_ENC_DELTA_RAD;
    case WheelDataType::VEL_SP:
      return STR_VEL_SP;
  }

  ULOG_ERROR("at %s in %d : parsing unknown wheel data type", __FILE__,
             __LINE__);
  return {};
}

inline constexpr bool operator==(const WheelDataType& e, std::string_view sv) {
  switch (e) {
    case WheelDataType::ENC_DELTA_RAD:
      return sv == STR_ENC_DELTA_RAD;
    case WheelDataType::VEL_SP:
      return sv == STR_VEL_SP;
  }
  return false;
}

inline constexpr bool operator==(std::string_view sv, const WheelDataType& e) {
  return e == sv;
}
