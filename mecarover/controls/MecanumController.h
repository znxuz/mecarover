#pragma once

#include <Eigen/Eigen/Core>
#include <Eigen/Eigen/LU>
#include <mecarover/mrlogger/mrlogger.h>

#include "VehicleController.h"

namespace imsl::vehiclecontrol
{

template <typename T>
class MecanumController : public VehicleController<T, 4, 4> {
public:
	using typename VehicleController<T, 4, 4>::VelWheel;
	using typename VehicleController<T, 4, 4>::VelRF;

private:
	Eigen::Matrix<T, 3, 3> OdoCorrMat = Eigen::Matrix<T, 3, 3>::Identity();
	T epsilon1 = 0;							// Verkopplungsfehler
	T LplusBhalbe;							/* in mm 0.5 * (Breite + Laenge)   */
	T Radradius;							/* in mm                           */
	T JoystickBeschl;						/* maximale Jostickbeschleunigung in mm/s/s */
	Abtastzeit_t Ta;						/* Abtastzeiten der Regler */
	ReglerParam_t Regler;
	VelWheel RAbwAlt = VelWheel::Zero();
	VelWheel Integr = VelWheel::Zero();
	dPose<T> DeltaPose;
	unsigned int DeltaPoseCounter = 0;
	T DeltaHeading = static_cast<T>(0);
	unsigned int DeltaHeadingCounter = 0;

public:
	// for data types VelWheel, OdoCorrMat
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // eigenlib 16 Byte alignement

	MecanumController() = default;

	void Init(Fahrzeug_t *Fz, ReglerParam_t Regler, Abtastzeit_t Ta)
	{
		LplusBhalbe = Fz->LplusBhalbe;
		Radradius = Fz->Radradius;

		JoystickBeschl = Fz->JoystickBeschl;
		this->Regler = Regler;
		this->Ta = Ta;

		this->j << 1.0, 1.0, LplusBhalbe, 1.0,
			1.0, -1.0, -LplusBhalbe, 1.0,
			1.0, 1.0, -LplusBhalbe, -1.0,
			1.0, -1.0, LplusBhalbe, -1.0;
		this->j /= Radradius;

		this->inv_j << 1.0, 1.0, 1.0, 1.0,
			1.0, -1.0, 1.0, -1.0,
			1.0 / LplusBhalbe, -1.0 / LplusBhalbe, -1.0 / LplusBhalbe, 1.0 / LplusBhalbe,
			4.0 / Radradius, 4.0 / Radradius, -4.0 / Radradius, -4.0 / Radradius;
		this->inv_j *= Radradius / 4.0;

		//            std::cerr << "J: " << J << std::endl;
		//            std::cerr << "iJ: " << iJ << std::endl;
	}

	/*
	 * convert the velocity delta of the wheels to the robot pose delta
	 * calculate new pose by adding the pose delta to the old pose
	 * vel: VelWheelMatrix
	 */
	Pose<T> odometry(Pose<T> oldPose, const VelWheel &vel) override
	{
		VelRF rframe_vel = this->vWheel2vRF(vel);

		dPose<T> rframe_delta;
		rframe_delta.x = rframe_vel(0); // movement in x direction
		rframe_delta.y = rframe_vel(1); // movement in y direction
		rframe_delta.theta = rframe_vel(2); // rotation around z axis (theta)
		epsilon1 += rframe_vel(3); // coupling error (delta)

		dPose<T> wframe_delta = dRF2dWF<T>(rframe_delta, oldPose.theta + rframe_delta.theta / static_cast<T>(2));

		Pose<T> newPose = oldPose + wframe_delta; // calculate new pose of vehicle in wolrd frame

		// unused because both the counter and the delta
		// are never updated
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
	}

	VelRF poseControl(PoseV<T> soll, PoseV<T> ist) override
	{
		dPose<T> Schleppabst;
		vPose<T> WKSStellV;

		Schleppabst.x = soll.x - ist.x;
		Schleppabst.y = soll.y - ist.y;
		Schleppabst.theta = soll.theta - ist.theta;

		/* Wert des Schleppwinkel von Phi in den Bereich -PI .. +PI bringen */
		Schleppabst.theta = Heading<T>(Schleppabst.theta);

		/* Uberpruefen des Schleppfehlers */
		if ((ABS(Schleppabst.x) > Regler.LageSchleppMax.x) || (ABS(Schleppabst.y) > Regler.LageSchleppMax.y) || (ABS(Schleppabst.theta) > Regler.LageSchleppMax.theta)) {
			log_message(log_error, "%s, %s, deviation position controller too large: act.x: %f, ref.x: %f, act.y: %f, ref.y: %f, act.theta: %f, ref.theta: %f",
				__FILE__, __FUNCTION__, ist.x, soll.x, ist.y, soll.y, T(ist.theta), T(soll.theta));
			log_message(log_error, "Max. Schleppabstand: %f %f %f Schleppabstand %f %f %f", Regler.LageSchleppMax.x,
				Regler.LageSchleppMax.y, Regler.LageSchleppMax.theta, Schleppabst.x, Schleppabst.y, Schleppabst.theta);

			throw MRC_LAGEERR;
		}

		/* Stellgroesse in Weltkoordinaten berechnen */
		WKSStellV.vx = soll.vx + Regler.LageKv.x * Schleppabst.x;
		WKSStellV.vy = soll.vy + Regler.LageKv.y * Schleppabst.y;
		WKSStellV.omega = soll.omega + Regler.LageKv.theta * Schleppabst.theta;

		/* Transformation in Roboterkoordinaten */
		vPose<T> RKSGeschw;
		RKSGeschw = vWF2vRF<T>(WKSStellV, ist.theta);
		VelRF v;
		v(0) = RKSGeschw.vx;
		v(1) = RKSGeschw.vy;
		v(2) = RKSGeschw.omega;
		v(3) = Regler.Koppel * epsilon1; // ASK Koppel is the proportional scaler from PID?
		return v;
	}

	VelWheel wheelControl(const VelWheel &refVel, const VelWheel &realVel) override
	{
		T regelAbw, diff;
		VelWheel StellV;

		for (int i = 0; i < 4; i++) {
			// Regelabweichung, Sollwert und Istwert in rad/s
			regelAbw = refVel(i) - realVel(i);

			// I-Anteil
			Integr(i) += Regler.DzrTaDTn * regelAbw;
			if (Integr(i) > Regler.DzrIntMax) {
				Integr(i) = Regler.DzrIntMax;
			} else if (Integr(i) < -(Regler.DzrIntMax)) {
				Integr(i) = -(Regler.DzrIntMax);
			}
			// printf("I-Anteil für den %d. Motor: %f\n", i, Integr(i));

			// D-Anteil
			diff = Regler.DzrTvDTa * (regelAbw - RAbwAlt(i));

			// Stellgroesse in rad/s
			StellV(i) = Regler.DzrKv * (regelAbw + Integr(i) + diff);

			// Vorsteuerung addieren
			StellV(i) += refVel(i);

			// Anpassung an individuellen Verstaerker
			StellV(i) *= Regler.DzrSkalierung[i];

			RAbwAlt(i) = regelAbw;
		}
		return StellV;
	}

	vPose<T> velocityFilter(vPose<T> RKSGeschwSoll, vPose<T> RKSGeschwAlt) override
	{
		T v_diff;
		vPose<T> RKSGeschw;
		RKSGeschw.vx = RKSGeschwSoll.vx;
		RKSGeschw.vy = RKSGeschwSoll.vy;
		RKSGeschw.omega = RKSGeschwSoll.omega;

		v_diff = JoystickBeschl * Ta.FzLage;

		if (RKSGeschw.vx - RKSGeschwAlt.vx > v_diff) {
			RKSGeschw.vx = RKSGeschwAlt.vx + v_diff;
		} else if (RKSGeschwAlt.vx - RKSGeschw.vx > v_diff) {
			RKSGeschw.vx = RKSGeschwAlt.vx - v_diff;
		}

		if (RKSGeschw.vy - RKSGeschwAlt.vy > v_diff) {
			RKSGeschw.vy = RKSGeschwAlt.vy + v_diff;
		} else if (RKSGeschwAlt.vy - RKSGeschw.vy > v_diff) {
			RKSGeschw.vy = RKSGeschwAlt.vy - v_diff;
		}

		v_diff /= LplusBhalbe;

		if (RKSGeschw.omega - RKSGeschwAlt.omega > v_diff) {
			RKSGeschw.omega = RKSGeschwAlt.omega + v_diff;
		} else if (RKSGeschwAlt.omega - RKSGeschw.omega > v_diff) {
			RKSGeschw.omega = RKSGeschwAlt.omega - v_diff;
		}
		return RKSGeschw;
	}

	// TODO functions below are not used, ask what they are for
	void poseUpdate(dPose<T> delta, unsigned int divisor) override
	{
		DeltaPoseCounter = divisor;
		DeltaPose = delta / static_cast<T>(divisor);
	}

	void headingUpdate(T delta, unsigned int divisor) override
	{
		DeltaHeadingCounter = divisor;
		DeltaHeading = delta / static_cast<T>(divisor);
	}
};

}
