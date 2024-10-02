#include "stm_hal.hpp"

#include <cstdint>
#include <tim.h>

#include "stm_encoder.hpp"
#include "stm_motor_pwm.hpp"

constexpr real_t PWM_MAX = 100.0; // output value for 100% duty cycle
constexpr real_t VOLT_MAX = 10.0; // maximum positive output voltage
/*constexpr int16_t DACmax = 4095; // this value leads to VoltMax Output*/
/*constexpr int16_t DACmin = 0; // this value leads to -VoltMax Output*/

static real_t rad2pwm = 0.0;
static real_t rad2volt = 1.0; // 0.0
static real_t inc2rad = 0.0;

// TIM_HandleTypeDef *encoder_timer[4] = { &htim1, &htim2, &htim3, &htim4 };
TIM_HandleTypeDef *encoder_timer[4] = { &htim3, &htim1, &htim2, &htim4 };

// Pwm Timer und Channels
TIM_HandleTypeDef *pwm_timer[4] = { &htim8, &htim5, &htim9, &htim8 };
const uint32_t Channel_PwmA[4] = { TIM_CHANNEL_4, TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_1 };
const uint32_t Channel_PwmB[4] = { TIM_CHANNEL_3, TIM_CHANNEL_2, TIM_CHANNEL_2, TIM_CHANNEL_2 }; // FIXME: htim5 isn't configured to use channel 1 & 2 but rather channel 1 & 4

// conversion of physical rotation direction to logical rotation direction
int encoder_direction[NUM_MOTORS] = { +1, +1, +1, +1 }; // rotation direction of the wheel encoders: +1 or -1
constexpr real_t encoder_scaler[NUM_MOTORS] = { 1.0, 1.0, 1.0, 1.0 };
// const real_t encoder_scaler[4] = { 512.0 / 500.0, 1.0, 1.0, 1.0 }; // is multiplied with ink2rad, wheel 0 has an encoder with 500 lines
const int motor_direction[NUM_MOTORS] = { +1, +1, -1, -1 }; // rotation direction of the motors (DAC or PWM)  +1 or -1

static STMEncoder encoder[NUM_MOTORS];
static STMMotorPWM motor_pwm[NUM_MOTORS];
static int32_t prev_encoder_val[NUM_MOTORS];

bool hal_is_init = false;

void hal_init(const Fahrzeug_t *fz)
{
	// FIXME: rad2pwm and rad2volt not used, along with the defined max values
	rad2pwm = PWM_MAX * 60.0 * fz->Uebersetzung / (2.0 * M_PI * fz->OmegaMax); // OmegaMax is in revolutions / min
	rad2volt = VOLT_MAX * 60.0 * fz->Uebersetzung / (2.0 * M_PI * fz->OmegaMax);
	inc2rad = fz->Ink2Rad;

	for (size_t i = 0; i < NUM_MOTORS; i++) {
		if (!encoder[i].init(encoder_timer[i]))
			log_message(log_error, "Failed to initialize encoder[%d]", i);

		prev_encoder_val[i] = encoder[i].get_val();

		if (!motor_pwm[i].init(i, pwm_timer[i], Channel_PwmA[i], Channel_PwmB[i]))
			log_message(log_error, "Failed to initialize motor[%d]", i);
	}


	// Testen des Pwm Signals
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 10);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 10);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 10);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);

	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 10);
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);

	// write 0 velocity to all channels
	real_t zero[4] = { 0.0, 0.0, 0.0, 0.0 };
	hal_wheel_vel_set_pwm(zero);
	hal_is_init = true;
}

std::array<real_t, NUM_MOTORS> hal_encoder_delta()
{
	auto encoder_delta = std::array<real_t, 4>{};

	for (int i = 0; i < NUM_MOTORS; i++) {
		auto encoder_val = encoder[i].get_val();
		encoder_delta[i] = inc2rad
			* (static_cast<real_t>(encoder_val) - prev_encoder_val[i])
			* encoder_direction[i] * encoder_scaler[i];

		prev_encoder_val[i] = encoder_val;
	}

	return encoder_delta;
}

int hal_encoder_read(real_t *DeltaPhi)
{
	for (int i = 0; i < NUM_MOTORS; i++) {
		auto val = encoder[i].get_val();

		// Deltawerte der Raeder in Rad
		DeltaPhi[i] = inc2rad * (static_cast<real_t>(val) - prev_encoder_val[i])
			* encoder_direction[i] * encoder_scaler[i];

		prev_encoder_val[i] = val;
		// TODO check over- and underflow 16 Bit
		// log_message(log_info, "encoder[%d] delta: %f\n", i, DeltaPhi[i]);
	}
	return 0;
}

int hal_wheel_vel_set_pwm(real_t *duty)
{
	for (int i = 0; i < NUM_MOTORS; i++) {
		duty[i] *= motor_direction[i];
		duty[i] *= 1; // Rad2PWM // FIXME: why times 1? doesn't make sense

		motor_pwm[i].setPWM(duty[i], pwm_timer[i],
				Channel_PwmA[i], Channel_PwmB[i]);
		//		printf(" NR: %d dutycicle = %.2f\n", i, duty[i]);
	}

	return 0;
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
			encoder[i].update_offset();
			break;
		}
	}
}
}
