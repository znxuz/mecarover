#pragma once

#include <Eigen/Eigen/Core>
#include <Eigen/Eigen/LU>
#include <mecarover/mrlogger/mrlogger.h>

#include "VehicleControl.h"

namespace imsl::vehiclecontrol
{
template<typename T>
class FourWheelMecanumController : public VehicleController<T, 4, 4>
{
public:
	using typename VehicleController<T, 4, 4>::WheelVel;
	using typename VehicleController<T, 4, 4>::VehicleVel;
	using VehicleController<T, 4, 4>::J;
	using VehicleController<T, 4, 4>::iJ;
	using VehicleController<T, 4, 4>::vWheel2vRF;
	using VehicleController<T, 4, 4>::vRF2vWheel;

private:
	Eigen::Matrix<T, 3, 3> OdoCorrMat;
	T epsilon1;             // Verkopplungsfehler
	T LplusBhalbe;          /* in mm 0.5 * (Breite + Laenge)   */
	T Radradius;            /* in mm                           */
	T JoystickBeschl;       /* maximale Jostickbeschleunigung in mm/s/s */
	ReglerParam_t Regler;
	Abtastzeit_t Ta;        /* Abtastzeiten der Regler */
	WheelVel RAbwAlt;
	WheelVel Integr;
	dPose<T> DeltaPose;
	unsigned int DeltaPoseCounter;
	T DeltaHeading;
	unsigned int DeltaHeadingCounter;

public:
	// for data types WheelVel, OdoCorrMat  
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // eigenlib 16 Byte alignement

		FourWheelMecanumController() {
		}

	void Init(Fahrzeug_t * Fz, ReglerParam_t Regler, Abtastzeit_t Ta) {
		epsilon1 = 0;
		OdoCorrMat = Eigen::Matrix<T, 3, 3>::Identity(); // TODO aus Konfigdatei einlesen
		LplusBhalbe = Fz->LplusBhalbe;
		Radradius = Fz->Radradius;

		JoystickBeschl = Fz->JoystickBeschl;
		this->Regler = Regler;
		this->Ta = Ta;

		Integr = WheelVel::Zero();
		RAbwAlt = WheelVel::Zero();
		DeltaPoseCounter = 0;
		DeltaHeadingCounter = 0;
		DeltaHeading = 0.0;

		J << 1.0,  1.0,  LplusBhalbe,  1.0,
		  1.0, -1.0, -LplusBhalbe,  1.0,
		  1.0,  1.0, -LplusBhalbe, -1.0,
		  1.0, -1.0,  LplusBhalbe, -1.0;
		J /= Radradius;

		iJ <<  1.0,  1.0,  1.0,  1.0,
		   1.0, -1.0,  1.0, -1.0,
		   1.0 / LplusBhalbe, -1.0 / LplusBhalbe, -1.0 / LplusBhalbe, 1.0 / LplusBhalbe,
		   4.0 / Radradius, 4.0 / Radradius, -4.0 / Radradius, -4.0 / Radradius;
		iJ *= Radradius / 4.0;

		//            std::cerr << "J: " << J << std::endl;
		//            std::cerr << "iJ: " << iJ << std::endl;
	}

	FourWheelMecanumController(Fahrzeug_t * Fz) {
		Init(Fz);
	}

	Pose<T> odometry(Pose<T> oldPose, const WheelVel& m) override { 
		dPose<T> WFDelta, RFDelta; 
		Pose<T> newPose;

		// transform wheel movements to vehicle movements in robot frame
		VehicleVel v;
		v = vWheel2vRF(m); 

		RFDelta.x = v(0);           // movement in x direction
		RFDelta.y = v(1);           // movement in y direction
		RFDelta.theta = v(2);       // rotation around z axis (theta)

		epsilon1 += v(3); // coupling error (delta)

		// transform vehicle movements from robot frame to world frame
		WFDelta = dRF2dWF<T>(RFDelta, oldPose.theta + RFDelta.theta / T(2.0));

		newPose = oldPose + WFDelta;    // calculate new pose of vehicle in wolrd frame

		// update pose filter
		if (DeltaPoseCounter > 0) {
			newPose = newPose + DeltaPose;
			DeltaPoseCounter--;
		}

		// update heading filter
		if (DeltaHeadingCounter > 0) {
			newPose.theta += DeltaHeading;
			DeltaHeadingCounter--;
		}

		return newPose;
	} // end Odometry

	VehicleVel poseControl(PoseV<T> soll, PoseV<T> ist) override {
		dPose<T> Schleppabst;
		vPose<T> WKSStellV;

		Schleppabst.x = soll.x - ist.x;
		Schleppabst.y = soll.y - ist.y;
		Schleppabst.theta = soll.theta - ist.theta;

		/* Wert des Schleppwinkel von Phi in den Bereich -PI .. +PI bringen */
		Schleppabst.theta = Heading<T>(Schleppabst.theta);

		/* Uberpruefen des Schleppfehlers */
		if ((ABS(Schleppabst.x) > Regler.LageSchleppMax.x) || (ABS(Schleppabst.y) > Regler.LageSchleppMax.y) ||
				(ABS(Schleppabst.theta) > Regler.LageSchleppMax.theta)) {

			log_message(log_error, "%s, %s, deviation position controller too large: act.x: %f, ref.x: %f, act.y: %f, ref.y: %f, act.theta: %f, ref.theta: %f",
					__FILE__, __FUNCTION__, ist.x, soll.x, ist.y, soll.y, T(ist.theta), T(soll.theta));
			log_message(log_error, "Max. Schleppabstand: %f %f %f Schleppabstand %f %f %f", Regler.LageSchleppMax.x,
					Regler.LageSchleppMax.y, Regler.LageSchleppMax.theta, Schleppabst.x, Schleppabst.y, Schleppabst.theta);

			throw MRC_LAGEERR;  // TODO imsl error 
		}

		/* Stellgroesse in Weltkoordinaten berechnen */
		WKSStellV.vx = soll.vx + Regler.LageKv.x * Schleppabst.x;
		WKSStellV.vy = soll.vy + Regler.LageKv.y * Schleppabst.y;
		WKSStellV.omega = soll.omega + Regler.LageKv.theta * Schleppabst.theta;

		/* Transformation in Roboterkoordinaten */
		vPose<T> RKSGeschw;
		RKSGeschw = vWF2vRF<T>(WKSStellV, ist.theta);
		VehicleVel v;
		v(0) = RKSGeschw.vx;            
		v(1) = RKSGeschw.vy;
		v(2) = RKSGeschw.omega;
		v(3) = Regler.Koppel * epsilon1;
		return v;
	} // end PoseControl

	WheelVel wheelControl(const WheelVel& referenceVel, const WheelVel& actualVel) override {
		T rabw, diff;
		WheelVel StellV;

		for (int i = 0; i < 4; i++) {
			// Regelabweichung, Sollwert und Istwert in rad/s   
			rabw = referenceVel(i) - actualVel(i);

			// I-Anteil
			Integr(i) += Regler.DzrTaDTn * rabw;
			if (Integr(i) > Regler.DzrIntMax) {
				Integr(i) = Regler.DzrIntMax;
			} else if (Integr(i) < -(Regler.DzrIntMax)) {
				Integr(i) = -(Regler.DzrIntMax);
			}

			// D-Anteil
			diff = Regler.DzrTvDTa * (rabw - RAbwAlt(i));

			// Stellgroesse in rad/s
			StellV(i) = Regler.DzrKv * (rabw + Integr(i) + diff);

			// Vorsteuerung addieren
			StellV(i) += referenceVel(i);

			// Anpassung an individuellen Verstaerker
			StellV(i) *= Regler.DzrSkalierung[i];

			RAbwAlt(i) = rabw;

		}
		return StellV;
	}

	vPose<T> velocityFilter(vPose<T> RKSGeschwSoll, vPose<T> RKSGeschwAlt) override {
		T v_diff;
		vPose<T> RKSGeschw;
		RKSGeschw.vx = RKSGeschwSoll.vx;
		RKSGeschw.vy = RKSGeschwSoll.vy;
		RKSGeschw.omega = RKSGeschwSoll.omega;

		v_diff = JoystickBeschl * Ta.FzLage;

		if (RKSGeschw.vx - RKSGeschwAlt.vx > v_diff) {
			RKSGeschw.vx = RKSGeschwAlt.vx + v_diff;
		}
		else if (RKSGeschwAlt.vx - RKSGeschw.vx > v_diff) {
			RKSGeschw.vx = RKSGeschwAlt.vx - v_diff;
		}

		if (RKSGeschw.vy - RKSGeschwAlt.vy > v_diff) {
			RKSGeschw.vy = RKSGeschwAlt.vy + v_diff;
		}
		else if (RKSGeschwAlt.vy - RKSGeschw.vy > v_diff) {
			RKSGeschw.vy = RKSGeschwAlt.vy - v_diff;
		}

		v_diff /= LplusBhalbe;

		if (RKSGeschw.omega - RKSGeschwAlt.omega > v_diff) {
			RKSGeschw.omega = RKSGeschwAlt.omega + v_diff;
		}
		else if (RKSGeschwAlt.omega - RKSGeschw.omega > v_diff) {
			RKSGeschw.omega = RKSGeschwAlt.omega - v_diff;
		}
		return RKSGeschw;
	}

	void poseUpdate(dPose<T> delta, unsigned int divisor) override {
		DeltaPoseCounter = divisor;
		DeltaPose = delta / (T)divisor;
	}

	void headingUpdate(T delta, unsigned int divisor) override {
		DeltaHeadingCounter = divisor;
		DeltaHeading = delta / (T)divisor;
	}

};
}
