#include "stm_hal.hpp"

#include <cstdint>
#include <tim.h>
#include <stm32f7xx_hal_tim.h> //Für Kontrollausgaben bsp. __HAL_TIM_SET_Compare -> Pwm testen

#include "stm_counter.hpp"
#include "stm_motor_pwm.hpp"

constexpr real_t PWMmax = 100.0; // output value for 100% duty cycle
constexpr real_t VoltMax = 10.0; // maximum positive output voltage
/*constexpr int16_t DACmax = 4095; // this value leads to VoltMax Output*/
/*constexpr int16_t DACmin = 0; // this value leads to -VoltMax Output*/

static real_t Rad2PWM = 0.0;
static real_t Rad2Volt = 1.0; // 0.0
static real_t Ink2Rad = 0.0;

// Encoder Timer
// TIM_HandleTypeDef *Tim_Enc[4] = { &htim1, &htim2, &htim3, &htim4 };
TIM_HandleTypeDef *Tim_Enc[4] = { &htim3, &htim1, &htim2, &htim4 };

// Pwm Timer und Channels
TIM_HandleTypeDef *Tim_Pwm[4] = { &htim8, &htim5, &htim9, &htim8 };
const uint32_t Channel_PwmA[4] = { TIM_CHANNEL_4, TIM_CHANNEL_1, TIM_CHANNEL_1,
	TIM_CHANNEL_1 };
const uint32_t Channel_PwmB[4] = { TIM_CHANNEL_3, TIM_CHANNEL_2, TIM_CHANNEL_2,
	TIM_CHANNEL_2 };

// conversion of physical rotation direction to logical rotation direction
int encoderDirection[4] = { +1, +1, +1, +1 }; // rotation direction of the wheel encoders: +1 or -1
constexpr real_t encoderScaling[4] = { 1.0, 1.0, 1.0, 1.0 };
// const real_t encoderScaling[4] = { 512.0 / 500.0, 1.0, 1.0, 1.0 }; // is multiplied with ink2rad, wheel 0 has an encoder with 500 lines
const int motorDirection[4] = { +1, +1, -1, -1 }; // rotation direction of the motors (DAC or PWM)  +1 or -1

static STMCounter Encoder[NumMotors];
static STMMotorPWM MotorPWM[NumMotors];
static int64_t prev_encoder_val[NumMotors];

bool hal_is_init = false;

void hal_init(const Fahrzeug_t *fz)
{
	Rad2PWM = PWMmax * 60.0 * fz->Uebersetzung / (2.0 * M_PI * fz->OmegaMax); // OmegaMax is in revolutions / min
	Rad2Volt = VoltMax * 60.0 * fz->Uebersetzung / (2.0 * M_PI * fz->OmegaMax);
	Ink2Rad = fz->Ink2Rad;

	for (int i = 0; i < NumMotors; i++) {
		if (!Encoder[i].init(Tim_Enc[i], i))
			log_message(log_error, "Failed to initialize encoders");

		prev_encoder_val[i] = Encoder[i].getCount(i);

		if (!MotorPWM[i].init(i, Tim_Pwm[i], Channel_PwmA[i], Channel_PwmB[i]))
			log_message(log_error, "Failed to initialize encoders");
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
	hal_wheel_vel_set(zero);
	hal_is_init = true;
}

std::array<uint64_t, NumMotors> hal_encoder_delta()
{
	auto encoder_delta = std::array<uint64_t, 4>{};

	for (int i = 0; i < NumMotors; i++) {
		auto encoder_val = Encoder[i].getCount(i);
		encoder_delta[i] = Ink2Rad * static_cast<real_t>(encoder_val - prev_encoder_val[i])
			* encoderDirection[i] * encoderScaling[i];
		prev_encoder_val[i] = encoder_val;
	}

	return encoder_delta;
}

int hal_encoder_read(real_t *DeltaPhi)
{
	for (int i = 0; i < NumMotors; i++) {
		int64_t val = Encoder[i].getCount(i); // get Encoder Counter

		// Deltawerte der Raeder in rad fuer Odometrie
		DeltaPhi[i] = Ink2Rad * static_cast<real_t>(val - prev_encoder_val[i])
			* encoderDirection[i] * encoderScaling[i];
		prev_encoder_val[i] = val;
		// TODO check over- and underflow 16 Bit
		// log_message(log_info, "encoder[%d] delta: %f\n", i, DeltaPhi[i]);
	}
	return 0;
}

int mr_hal_wheel_vel_set_pwm(real_t *duty)
{
	for (int i = 0; i < NumMotors; i++) {
		duty[i] *= 1; // Rad2PWM

		MotorPWM[i].setPWM(duty[i], Tim_Pwm[i],
				Channel_PwmA[i], Channel_PwmB[i]);
		//		printf(" NR: %d dutycicle = %.2f\n", i, duty[i]);
	}
	return 0;
}

// TODO fusion the function above to the one below
int hal_wheel_vel_set(real_t *w)
{
	for (int i = 0; i < NumMotors; i++)
		w[i] *= motorDirection[i];

	return mr_hal_wheel_vel_set_pwm(w);
}

bool hal_get_estop()
{
	// probably no estop from the stm32 mcu, dunno
	return false;
}

/*
 * Motor driver without amplifiers at the moment -> planned for the future
 */

// int hal_amplifiers_enable() {
// #ifdef PWM
//   for (int i = 0; i < NumMotors; i++) {
//     MotorPWM[i].setPWM(0.0);
//   }
// #endif
//   gpio_set_level(gpio_num_t(EnablePin), 1);
//   return 0;
// }
//
// int hal_amplifiers_disable(void) {
// #ifdef PWM
//   for (int i = 0; i < NumMotors; i++) {
//     MotorPWM[i].setPWM(0.0);
//   }
// #endif
//   gpio_set_level(gpio_num_t(EnablePin), 0);
//   return 0;
// }
//
