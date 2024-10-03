#include "stm_hal.hpp"

#include <cstdint>
#include <tim.h>

#include "stm_encoder.hpp"
#include "stm_motor_pwm.hpp"

TIM_HandleTypeDef* encoder_timer[] = {&htim3, &htim1, &htim2, &htim4};
TIM_HandleTypeDef* pwm_timer[] = {&htim8, &htim5, &htim9, &htim8};
constexpr uint32_t pwm_a_channels[]
	= {TIM_CHANNEL_4, TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_1};
constexpr uint32_t pwm_b_channels[]
	= {TIM_CHANNEL_3, TIM_CHANNEL_4, TIM_CHANNEL_2, TIM_CHANNEL_2};

constexpr int8_t motor_direction[NUM_MOTORS] = {+1, +1, -1, -1};
constexpr int8_t encoder_direction[NUM_MOTORS] = {+1, +1, +1, +1};
constexpr real_t encoder_scaler[NUM_MOTORS] = {1.0, 1.0, 1.0, 1.0};

static STMEncoder encoders[NUM_MOTORS];
static STMMotorPWM pwm_motors[NUM_MOTORS];
static int32_t prev_encoder_val[NUM_MOTORS];
static real_t inc2rad = 0.0;
static bool hal_is_init = false; // TODO: necessary bool?

void hal_init(const Fahrzeug_t* fz)
{
	inc2rad = fz->Ink2Rad;

	for (size_t i = 0; i < NUM_MOTORS; i++) {
		if (!encoders[i].init(encoder_timer[i]))
			log_message(log_error, "Failed to initialize encoder[%d]", i);
		prev_encoder_val[i] = encoders[i].get_val();
		if (!pwm_motors[i].init(pwm_timer[i], pwm_a_channels[i],
								pwm_b_channels[i], motor_direction[i]))
			log_message(log_error, "Failed to initialize motor[%d]", i);
	}

	hal_wheel_vel_set_pwm(std::array<real_t, 4>{0.0, 0.0, 0.0, 0.0}.data());
	hal_is_init = true;
}

std::array<real_t, NUM_MOTORS> hal_encoder_delta()
{
	auto encoder_delta = std::array<real_t, 4>{};

	for (int i = 0; i < NUM_MOTORS; i++) {
		auto encoder_val = encoders[i].get_val();
		encoder_delta[i] = inc2rad
			* (static_cast<real_t>(encoder_val) - prev_encoder_val[i])
			* encoder_direction[i] * encoder_scaler[i];

		prev_encoder_val[i] = encoder_val;
	}

	return encoder_delta;
}

void hal_encoder_read(real_t* DeltaPhi)
{
	for (int i = 0; i < NUM_MOTORS; i++) {
		auto val = encoders[i].get_val();
		DeltaPhi[i] = inc2rad * (static_cast<real_t>(val) - prev_encoder_val[i])
			* encoder_direction[i] * encoder_scaler[i];
		prev_encoder_val[i] = val;
		// TODO check over- and underflow 16 Bit
	}
}

void hal_wheel_vel_set_pwm(const real_t* duty_cycle)
{
	for (size_t i = 0; i < NUM_MOTORS; ++i) {
		pwm_motors[i].setPWM(duty_cycle[i]);
	}
}

bool hal_get_estop()
{
	// probably no estop from the stm32 mcu; maybe the user button can be used
	return false;
}

extern "C"
{
void stm_encoder_cb(TIM_HandleTypeDef* htim)
{
	if (!hal_is_init) [[unlikely]]
		return;

	for (size_t i = 0; i < NUM_MOTORS; ++i) {
		if (htim->Instance == encoder_timer[i]->Instance) {
			encoders[i].update_offset();
			break;
		}
	}
}
}
