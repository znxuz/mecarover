#pragma once

#include <stdint.h>

#include "mrtypes.h"

static inline constexpr uint8_t N_WHEEL = 4;
static inline constexpr uint16_t MS_TO_S = 1000;

struct ctrl_param_t {
	double k_i;
	double k_d;
	double k_ctrl_output;
	double integral_limit;
	Pose_t LageKv;
	Pose_t LageSchleppMax;
	double Koppel; /* Faktor für Ausregelung des Verkopplungsfehlers */
	// double ctrl_output_scaler[N_WHEEL]; // useless
	// double DzrSchleppMax;
	// double DzrKoppel[4];
};

struct sampling_time_t {
	// double Timer;
	double dt_pose_ctrl;
	double dt_wheel_ctrl;
	uint32_t ratio_pose_wheel;
};

static constexpr inline sampling_time_t sampling_times = {
	// .Timer = 0.001, // 1 ms timer tick
	.dt_pose_ctrl = 0.015, // 15 ms sampling time of pose control loop
	.dt_wheel_ctrl = 0.005, // 5 ms sampling time of wheel control loop
	.ratio_pose_wheel = 3 // 15 ms / 5 ms ratio pose / wheel
};

// ASK: why l_w_half is used rather than a+b for the velocity transformations?
static constexpr inline robot_param_t robot_params = {
	.increments = 48.0, // 4 x 1024 //4096
	.gear_ratio = 64.0, // gear ratio //68
	.inc2rad = 2.0 * M_PI / (64.0 * 48.0),
	.length = 325,
	.width = 360,
	.l_w_half = 0.5 * (360 + 325),
	.wheel_radius = 50,
	.JoystickBeschl = 1000.0, // in mm/s²
	// .rad2prm = 120.0 / 2.0 / M_PI,
	// .omega_max = 120.0, // n / min //6400
	// .roll_radius = 20.0, /* in mm                           */
	// .VBahnMax = 1000.0, /* in mm/s                         */
	// .VthetaMax = 2000.0, /* in rad/s                        */
	// .MaxRPM = {5740.6, 5740.6, 7268.4, 7268.4}
};

static constexpr inline ctrl_param_t ctrl_params = {
	.k_i = 0.1,
	.k_d = 0.0,
	.k_ctrl_output = 0.3,
	.integral_limit = 30.0,
	.LageKv = {5, 5, 5},
	.LageSchleppMax = {300.0, 300.0, 30.0},
	.Koppel = 0.0
	// .ctrl_output_scaler = {1.0, 1.0, 1.0, 1.0},
	// .DzrSchleppMax = 0.0,
	// .DzrKoppel = {-0.1, 0.1, -0.1, 0.1},
};
