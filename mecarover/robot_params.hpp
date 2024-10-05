#pragma once

#include <stdint.h>

#include "mrtypes.h"

static inline constexpr uint8_t N_WHEEL = 4;
static inline constexpr uint16_t MS_TO_S = 1000;

struct ctrl_param_t {
	double DzrKv;
	double DzrTaDTn;
	double DzrTvDTa;
	double DzrIntMax;
	double DzrSchleppMax;
	double DzrSkalierung[4];
	double DzrKoppel[4];
	Pose_t LageKv;
	Pose_t LageSchleppMax;
	double Koppel; /* Faktor für Ausregelung des Verkopplungsfehlers */
};

struct sampling_time_t {
	double Timer;
	double FzDreh;
	double FzLage;
	uint32_t FzLageZuDreh;
};

static constexpr inline sampling_time_t sampling_times = {
	.Timer = 0.001, // 1 ms timer tick
	.FzDreh = 0.005, // 5 ms sampling time of wheel control loop
	.FzLage = 0.015, // 15 ms sampling time of pose control loop
	.FzLageZuDreh = 3 // 15 ms / 5 ms ratio pose / wheel
};

static constexpr inline robot_param_t robot_params = {
	.Inkremente = 48.0, // 4 x 1024 //4096
	.Uebersetzung = 64.0, // gear ratio //68
	.OmegaMax = 120.0, // n / min //6400
	.Rad2RPM = 120.0 / 2.0 / M_PI,
	.Ink2Rad = 2.0 * M_PI / (64.0 * 48.0),
	.Laenge = 325,
	.Breite = 360,
	.LplusBhalbe = 0.5 * (360 + 325),
	.Radradius = 50,
	.Rollenradius = 20.0, /* in mm                           */
	.VBahnMax = 1000.0, /* in mm/s                         */
	.VthetaMax = 2000.0, /* in rad/s                        */
	.JoystickBeschl = 1000.0, // in mm/s²
	.MaxRPM = {5740.6, 5740.6, 7268.4, 7268.4}
};

static constexpr inline ctrl_param_t ctrl_params = {
	.DzrKv = 0.3,
	.DzrTaDTn = 0.1,
	.DzrTvDTa = 0.0,
	.DzrIntMax = 30.0,
	.DzrSchleppMax = 0.0,
	.DzrSkalierung = {1.0, 1.0, 1.0, 1.0},
	.DzrKoppel = {-0.1, 0.1, -0.1, 0.1},
	.LageKv = {5, 5, 5},
	.LageSchleppMax = {300.0, 300.0, 30.0},
	.Koppel = 0.0
};
