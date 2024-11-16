#include "hal.hpp"

#include <tim.h>

#include <application/robot_params.hpp>
#include <array>
#include <cstdint>

#include "encoder.hpp"
#include "motor.hpp"

using namespace robot_params;

static std::array encoder_timer{&htim3, &htim1, &htim2, &htim4};
static std::array pwm_timer{&htim8, &htim5, &htim9, &htim8};
static constexpr std::array pwm_channels_a{TIM_CHANNEL_4, TIM_CHANNEL_1,
                                           TIM_CHANNEL_1, TIM_CHANNEL_1};
static constexpr std::array pwm_channels_b{TIM_CHANNEL_3, TIM_CHANNEL_4,
                                           TIM_CHANNEL_2, TIM_CHANNEL_2};

constexpr static std::array motor_direction{1, 1, -1, -1};
constexpr static std::array encoder_direction{1, 1, 1, 1};
// constexpr static std::array encoder_scaler{1.0, 1.0, 1.0, 1.0};

static std::array<Encoder, N_WHEEL> encoders;
static std::array<Motor, N_WHEEL> pwm_motors;
static std::array<int32_t, N_WHEEL> prev_encoder_val{};

void hal_init() {
  for (size_t i = 0; i < N_WHEEL; i++) {
    encoders[i].init(encoder_timer[i]);
    pwm_motors[i].init(pwm_timer[i], pwm_channels_a[i], pwm_channels_b[i],
                       motor_direction[i]);
  }

  hal_wheel_vel_set_pwm({0.0, 0.0, 0.0, 0.0});
}

std::array<real_t, N_WHEEL> hal_encoder_delta_rad() {
  auto encoder_delta = std::array<real_t, N_WHEEL>{};

  for (int i = 0; i < N_WHEEL; ++i) {
    auto encoder_val = encoders[i].get_val();
    encoder_delta[i] =
        INC2RAD * (static_cast<real_t>(encoder_val - prev_encoder_val[i])) *
        encoder_direction[i];  //  * encoder_scaler[i]

    prev_encoder_val[i] = encoder_val;
  }

  return encoder_delta;
}

std::array<uint32_t, N_WHEEL> hal_encoder_val() {
  auto ret = std::array<uint32_t, N_WHEEL>{};
  for (size_t i = 0; i < N_WHEEL; ++i) {
    ret[i] = encoders[i].get_val();
  }
  return ret;
}

void hal_wheel_vel_set_pwm(const std::array<real_t, N_WHEEL>& duty_cycle) {
  for (size_t i = 0; i < N_WHEEL; ++i) pwm_motors[i].set_pwm(duty_cycle[i]);
}

extern "C" {
void encoder_tim_cb(TIM_HandleTypeDef* htim) {
  for (size_t i = 0; i < N_WHEEL; ++i) {
    if (htim->Instance == encoder_timer[i]->Instance) {
      encoders[i].update_offset();
      return;
    }
  }
}
}
