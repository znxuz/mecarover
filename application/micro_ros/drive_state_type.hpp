#pragma once

#include <ulog.h>

#include <optional>
#include <string_view>

inline constexpr std::string_view STR_ENC_DELTA_RAD =
    "wheel encoder delta in radian";
inline constexpr std::string_view STR_VEL_SP = "wheel velocity setpoint";

enum class DriveStateType { ENC_DELTA_RAD, VEL_SP };

inline constexpr std::optional<DriveStateType> parse_wheel_data_type(
    std::string_view sv) {
  if (sv == STR_ENC_DELTA_RAD) {
    return DriveStateType::ENC_DELTA_RAD;
  } else if (sv == STR_VEL_SP) {
    return DriveStateType::VEL_SP;
  }

  ULOG_ERROR("at %s in %d : parsing unknown wheel data type", __FILE__,
             __LINE__);
  return {};
}

inline constexpr std::optional<std::string_view> to_string(DriveStateType type) {
  switch (type) {
    case DriveStateType::ENC_DELTA_RAD:
      return STR_ENC_DELTA_RAD;
    case DriveStateType::VEL_SP:
      return STR_VEL_SP;
  }

  ULOG_ERROR("at %s in %d : parsing unknown wheel data type", __FILE__,
             __LINE__);
  return {};
}

inline constexpr bool operator==(const DriveStateType& e, std::string_view sv) {
  switch (e) {
    case DriveStateType::ENC_DELTA_RAD:
      return sv == STR_ENC_DELTA_RAD;
    case DriveStateType::VEL_SP:
      return sv == STR_VEL_SP;
  }
  return false;
}

inline constexpr bool operator==(std::string_view sv, const DriveStateType& e) {
  return e == sv;
}
