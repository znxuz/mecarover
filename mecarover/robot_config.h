#pragma once

//------------ configuration of the vehicle and controller parameters

#include <mecarover/controls/VehicleController.h> // definition of imsl::vehiclecontrol::ReglerParam_t, imsl::vehiclecontrol::Abtastzeit_t
#include <mrtypes.h> // definition of Fahrzeug_t

// ---------------------------------- OmniRob configuration
/*
   Typ = "Mecanum";
   Inkremente = 2048.0;
   Uebersetzung = 35.0;
   OmegaMax = 4000.0;
   Volt = 10.0;
   Laenge = 600.0;
   Breite = 490.0;
   Radradius = 112.5;
   VBahnMax = 800.0;
   VoltMax = 10.0;
   JoystickBeschl = 1000.0;
   SchutzfeldUmschaltung = 1;
   HubzylinderVorhanden = 1;
   ArmVorhanden = 0;
   BreakDown = 1;
BremsBeschl :
{
a = 400.0;
alpha = 0.5;
};
*/

// OmniRob robot parameters
// Fahrzeug_t fz = {
//                   .type = MRC_VEHICLETYPE_MECANUM,
//                   .Inkremente = 2048.0, // 4 x 512 quadrature encoder
//                   .Uebersetzung = 35.0, // gear ration
//                   .OmegaMax = 4000.0,   // n / min
//                   .Rad2RPM = 60.0 * fz.Uebersetzung / 2.0 / M_PI,
//                   .Ink2Rad = 2.0 * M_PI / (fz.Uebersetzung * fz.Inkremente),  // 2 * PI / ( gear * Increments)
//                   .Laenge = 600,
//                   .Breite = 490,
//                   .LplusBhalbe = 0.5 * (fz.Laenge + fz.Breite), // (length + width) / 2
//                   .Radradius = 112.5,
//                   .Rollenradius = 20.0, /* in mm                           */
//                   .VBahnMax = 1000.0,  /* in mm/s                         */
//                   .VthetaMax = 2000.0, /* in rad/s                        */
//                   .JoystickBeschl = 1000.0 // in mm/s²
//                   };

//// controller parameters
// imsl::vehiclecontrol::ReglerParam_t Regler = {
//                         0.3,  // Kv Drehzahl //1.0
//                         0.1, //0.006, // Ta/Tn  Ta Abstastzeit //0.001
//                         0.0,   // Tv/Ta //0.0
//                         3.0,   // max I //0.1
//                         0.0,   // e max, wheel
//                         {1.0, 1.0, 1.0, 1.0}, // Skalierung
//                         {-0.1, 0.1, -0.1, 0.1}, // Koppelfehler ?
////                        {10.0, 10.0, 10.0},     // Kv Lage
//                        {1.0, 1.0, 1.0},     // Kv Lage
////                        {3000000.0, 3000000.0, 300000.0},   // Schleppf. max, e_pose
//                        {300.0, 300.0, 300.0},   // Schleppf. max, e_pose
//                        0.0                     // Verkopplungsregelung
//                        };

// time constants
imsl::vehiclecontrol::Abtastzeit_t Ta = {
	0.001, // 1 ms timer tick
		   //                    0.003, // 3 ms sampling time of wheel control loop
	0.005, // 5 ms sampling time of wheel control loop
	0.015, // 15 ms sampling time of pose control loop
	3
}; // 15 ms / 5 ms ratio pose / wheel

/* FILU Roboter
// ------------------------- FILU robot configuration
*/
Fahrzeug_t fz = {
	.type = MRC_VEHICLETYPE_MECANUM,
	.Inkremente = 48.0, // 4 x 1024 //4096
	.Uebersetzung = 64.0, // gear ration //68
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
	.MaxRPM = { 5740.6, 5740.6, 7268.4, 7268.4 }
};

imsl::vehiclecontrol::ReglerParam_t Regler = {
	//                .MaxRPM = {5740.6, 5740.6, 7268.4, 7268.4},
	.DzrKv = 0.3,
	.DzrTaDTn = 0.1,
	.DzrTvDTa = 0.0,
	.DzrIntMax = 30.0,
	.DzrSchleppMax = 0.0,
	{ 1.0, 1.0, 1.0, 1.0 },
	{ -0.1, 0.1, -0.1, 0.1 },
	//                .DzrSkalierung[0] = 1.0,
	//                .DzrSkalierung[1] = 1.0,
	//                .DzrSkalierung[2] = 1.0,
	//                .DzrSkalierung[3] = 1.0,
	//                .Rad2Volt = 1,
	.LageKv = { 5, 5, 5 }, // 5 5 5
	.LageSchleppMax = { 300.0, 300.0, 30.0 },
	0.0
	//                .korrekturmatrix = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 }
};

/*
   typedef struct {
   double DzrKv;
   double DzrTaDTn;
   double DzrTvDTa;
   double DzrIntMax;
   double DzrSchleppMax;
   FourReal_t DzrSkalierung;
   FourReal_t DzrKoppel;
   Pose_t LageKv;
   Pose_t LageSchleppMax;
   double Koppel;
   } ReglerParam_t;
   */

/*
   typedef struct {
   double Timer;
   double FzDreh;
   double FzLage;
   uint32_t FzLageZuDreh;
   } Abtastzeit_t;
   */

#if 0
typedef struct {
	mrc_vehicletype_t type;	/* Fahrzeug-Typ                    */
	double Inkremente;		/* Geberinkremente (4 * Striche)   */
	double Uebersetzung;	/* Getriebeuebersetzung            */
	double OmegaMax;		/* Motornenndrehzahl in U/min      */
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
	mrc_stat_gyro GyroError;	/* Fehler beim Auslesen des Gyroskops */
	struct {
		double a;       /* Translatorische Bremsbeschleunigung */
		double alpha;   /* Rotatorische Bremsbeschleunigung */
	} bremsbeschl;    /* Bremsbeschleunigungen bei Nothalt */
	double MaxRPM[4];		/* Maximum Umdrehungen pro Minute Motor */
	int UsePipe;		/* Use fds 3 and 4 for communication with mrserver */
	struct {
		bool UseColors;
		mr_logprio_t LogLevel;
		mr_logprio_t ScreenLevel;
	} log;
} Fahrzeug_t;

#endif
