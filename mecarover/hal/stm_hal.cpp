#include "stm_hal.hpp"

#include <cstdint>
#include <tim.h>

#include <mecarover/robot_params.hpp>

#include "stm_encoder.hpp"
#include "stm_motor_pwm.hpp"

TIM_HandleTypeDef* encoder_timer[] = {&htim3, &htim1, &htim2, &htim4};
TIM_HandleTypeDef* pwm_timer[] = {&htim8, &htim5, &htim9, &htim8};
constexpr uint32_t pwm_a_channels[]
	= {TIM_CHANNEL_4, TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_1};
constexpr uint32_t pwm_b_channels[]
	= {TIM_CHANNEL_3, TIM_CHANNEL_4, TIM_CHANNEL_2, TIM_CHANNEL_2};

constexpr int8_t motor_direction[N_WHEEL] = {+1, +1, -1, -1};
constexpr int8_t encoder_direction[N_WHEEL] = {+1, +1, +1, +1};
constexpr real_t encoder_scaler[N_WHEEL] = {1.0, 1.0, 1.0, 1.0};

static STMEncoder encoders[N_WHEEL];
static STMMotorPWM pwm_motors[N_WHEEL];
static int32_t prev_encoder_val[N_WHEEL];
static real_t inc2rad = 0.0;
static bool hal_is_init = false;

void hal_init(const robot_param_t* robot_params)
{
	inc2rad = robot_params->Ink2Rad;

	for (size_t i = 0; i < N_WHEEL; i++) {
		encoders[i].init(encoder_timer[i]);
		pwm_motors[i].init(pwm_timer[i], pwm_a_channels[i], pwm_b_channels[i],
						   motor_direction[i]);

		prev_encoder_val[i] = encoders[i].get_val();
	}

	hal_wheel_vel_set_pwm({0.0, 0.0, 0.0, 0.0});
	hal_is_init = true;
}

// TODO check over- and underflow 16 Bit
std::array<real_t, N_WHEEL> hal_encoder_delta()
{
	auto encoder_delta = std::array<real_t, 4>{};

	for (int i = 0; i < N_WHEEL; ++i) {
		auto encoder_val = encoders[i].get_val();
		encoder_delta[i] = inc2rad
			* (static_cast<real_t>(encoder_val) - prev_encoder_val[i])
			* encoder_direction[i] * encoder_scaler[i];

		prev_encoder_val[i] = encoder_val;
	}

	return encoder_delta;
}

void hal_wheel_vel_set_pwm(const std::array<real_t, N_WHEEL>& duty_cycle)
{
	for (size_t i = 0; i < N_WHEEL; ++i) {
		pwm_motors[i].setPWM(duty_cycle[i]);
	}
}

extern "C"
{
void stm_encoder_cb(TIM_HandleTypeDef* htim)
{
	if (!hal_is_init) [[unlikely]]
		return;

	for (size_t i = 0; i < N_WHEEL; ++i) {
		if (htim->Instance == encoder_timer[i]->Instance) {
			encoders[i].update_offset();
			break;
		}
	}
}
}
