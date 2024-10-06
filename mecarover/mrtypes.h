#pragma once

#include <math.h>

#if __cplusplus
extern "C"
{
#endif

// Beschraenkung eines Winkels auf 0 ... 2PI
#ifndef MAX2PI
#define MAX2PI(x)                                                              \
	((x) > (2 * M_PI) ? ((x) - (2 * M_PI)) : ((x) < 0 ? ((x) + (2 * M_PI)) : x))
#endif

// Beschraenkung eines Winkels auf -PI ... +PI
#ifndef MAXPI
#define MAXPI(x)                                                               \
	((x) > (M_PI) ? ((x) - (2 * M_PI)) : ((x) < -M_PI ? ((x) + (2 * M_PI)) : x))
#endif

typedef double real_t;

typedef struct {
	double x;
	double y;
	double theta;
} Pose_t;

typedef enum {
	MRC_NOERR,
	MRC_DRZERR,
	MRC_LAGEERR,
	MRC_COMMERR,
	MRC_DOCKERR,
	MRC_EMGOFF,
	MRC_NOPERM
} mrc_stat;

typedef struct {
	double increments; /* Geberinkremente (4 * Striche)   */
	double gear_ratio; /* Getriebeuebersetzung            */
	double inc2rad; /* Ink --> rad                     */
	double length; /* in mm Abstand der Achsen        */
	double width; /* in mm Abstand der Achsen        */
	double l_w_half; /* in mm 0.5 * (Breite + Laenge)   */
	double wheel_radius; /* in mm                           */
	double JoystickBeschl; /* maximale Jostickbeschleunigung in mm/s/s */
	// double rad2prm; /* rad/s --> RPM                   */
	// double omega_max; /* Motornenndrehzahl in U/min      */
	// double roll_radius; /* in mm                           */
	// double VBahnMax; /* in mm/s                         */
	// double VthetaMax; /* in rad/s                        */
	// double MaxRPM[4]; /* Maximum Umdrehungen pro Minute Motor */
} robot_param_t;

#if __cplusplus
}
#endif
