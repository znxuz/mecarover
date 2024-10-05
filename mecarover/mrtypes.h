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
	double Inkremente; /* Geberinkremente (4 * Striche)   */
	double Uebersetzung; /* Getriebeuebersetzung            */
	double OmegaMax; /* Motornenndrehzahl in U/min      */
	double Rad2RPM; /* rad/s --> RPM                   */
	double Ink2Rad; /* Ink --> rad                     */
	double Laenge; /* in mm Abstand der Achsen        */
	double Breite; /* in mm Abstand der Achsen        */
	double LplusBhalbe; /* in mm 0.5 * (Breite + Laenge)   */
	double Radradius; /* in mm                           */
	double Rollenradius; /* in mm                           */
	double VBahnMax; /* in mm/s                         */
	double VthetaMax; /* in rad/s                        */
	double JoystickBeschl; /* maximale Jostickbeschleunigung in mm/s/s */
	double MaxRPM[4]; /* Maximum Umdrehungen pro Minute Motor */
} robot_param_t;

#if __cplusplus
}
#endif
