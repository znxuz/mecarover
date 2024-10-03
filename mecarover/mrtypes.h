#pragma once

#include <math.h>

#include <mecarover/mrlogger/mrlogger.h>

#if __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------*/
#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/*---------------------------------------------------------------------*/
#ifndef MIN
#define MIN(x,y)     ( ((x) <= (y)) ?  (x) : (y) )
#endif

#ifndef MAX
#define MAX(x,y)     ( ((x) >= (y)) ?  (x) : (y) )
#endif

#ifndef ABS
#define ABS(x)       ( (x) >= 0 ? (x) : (-(x)) )
#endif

#ifndef RAD
#define RAD(grad)    ( M_PI*(grad)/180.0 )
#endif

#ifndef GRAD
#define GRAD(rad)    ( 180.0*(rad)/M_PI )
#endif

// Beschraenkung eines Winkels auf 0 ... 2PI
#ifndef MAX2PI
#define MAX2PI(x) ((x) > (2*M_PI)?((x) - (2*M_PI)):((x) < 0?((x) + (2*M_PI)):x))
#endif

// Beschraenkung eines Winkels auf -PI ... +PI
#ifndef MAXPI
#define MAXPI(x) ((x) > (M_PI)?((x) - (2*M_PI)) : ((x) < -M_PI ? ((x) + (2*M_PI)) : x))
#endif

typedef double real_t;

typedef struct {
	double x;
	double y;
	double theta;
} Pose_t;

typedef struct {
	double x;
	double y;
	double theta;
	double vx;
	double vy;
	double omega;
} PoseV_t;

typedef struct {
	double delta_x;
	double delta_y;
	double delta_theta;
	double delta_phi_e;
} rks_move_t;

  typedef enum {
    MRC_VEHICLETYPE_UNKNOWN = 0,
    MRC_VEHICLETYPE_MECANUM = 1,
    MRC_VEHICLETYPE_DIFFERENTIAL = 2,
  } mrc_vehicletype_t;

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
	mrc_vehicletype_t type;	/* Fahrzeug-Typ                    */
	double Inkremente;		/* Geberinkremente (4 * Striche)   */
	double Uebersetzung;	/* Getriebeuebersetzung            */
	double OmegaMax;		/* Motornenndrehzahl in U/min      */
	//    double Volt;		/* Steuereing. Steller +/-         */
	//    double Ink2Volt;		/* Ink/Ta  --> Volt (Dzr)          */
	//    double Rad2Volt;		/* rad/s --> Volt                  */
	double Rad2RPM;		/* rad/s --> RPM                   */
	double Ink2Rad;		/* Ink --> rad                     */
	double Laenge;		/* in mm Abstand der Achsen        */
	double Breite;		/* in mm Abstand der Achsen        */
	double LplusBhalbe;		/* in mm 0.5 * (Breite + Laenge)   */
	double Radradius;		/* in mm                           */
	double Rollenradius;	/* in mm                           */
	double VBahnMax;		/* in mm/s                         */
	double VthetaMax;		/* in rad/s                        */
	double JoystickBeschl;	/* maximale Jostickbeschleunigung in mm/s/s */
	int SchutzfeldUmschaltung;	/* Soll das Schutzfeld der Lasterscanner vom Interpolator umgeschaltet werden? */
	int HubzylinderVorhanden;	/* Ist auf dem Fahrzeug ein Hubzylinder vorhanden, der gesteuert werden soll? */
	int ArmVorhanden;		/* Ist auf dem Fahrzeug ein Arm vorhanden, der gesteuert werden soll? */
	int BreakDown;		/* Soll das Fahrzeug bei einer Verletzung des Warnfelds stehen bleiben oder langsamer werden? */
	mrc_stat Stoerung;		/* Art der Stoerung */
	struct {
		double a;       /* Translatorische Bremsbeschleunigung */
		double alpha;   /* Rotatorische Bremsbeschleunigung */
	} bremsbeschl;    /* Bremsbeschleunigungen bei Nothalt */
	double MaxRPM[4];		/* Maximum Umdrehungen pro Minute Motor */
	int UsePipe;		/* Use fds 3 and 4 for communication with mrserver */
} Fahrzeug_t;

#if __cplusplus
}
#endif
