#pragma once

#include <Eigen/Eigen/Core>
#include <Eigen/Eigen/LU>

#include "VehicleController.h"

namespace imsl::vehiclecontrol {
template <typename T>
class DifferentialDriveController : public VehicleController<T, 2, 2> {
public:
	using typename VehicleController<T, 2, 2>::VelWheel;
	using typename VehicleController<T, 2, 2>::VelRF;

private:
	T WheelTrack; // wheel track (width) in mm
	T WheelRadius; // in mm
	T JoystickAcc; // maximum acceleration in manual mode in mm/s/s
	ReglerParam_t Regler;
	Abtastzeit_t Ta; // Abtastzeiten der Regler
	VelWheel RAbwAlt;
	VelWheel Integr;
	dPose<T> DeltaPose;
	unsigned int DeltaPoseCounter;
	T DeltaHeading;
	T Vmax;
	T Omega_max;
	unsigned int DeltaHeadingCounter;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // eigenlib 16 Byte alignement

	DifferentialDriveController() = default;

	void Init(Fahrzeug_t *Fz, ReglerParam_t Regler, Abtastzeit_t Ta)
	{
		WheelTrack = Fz->Breite;
		WheelRadius = Fz->Radradius;
		JoystickAcc = Fz->JoystickBeschl;
		this->Regler = Regler;
		this->Ta = Ta;
		Vmax = Fz->VBahnMax;
		Omega_max = Fz->VthetaMax;
		Integr = VelWheel::Zero();
		RAbwAlt = VelWheel::Zero();
		DeltaPoseCounter = 0;
		DeltaHeadingCounter = 0;
		DeltaHeading = 0.0;

		/*          definition of wheel positions:
					both wheels are located on the y axis, the vehicle moves in direction of the x axis
					the left wheel is defined as 0 (y position in robot frame is positive)
					the right wheel is defined as 1 (y position in robot frame in negative)
					*/
		// robot frame to wheels
		this->j << 1.0, -WheelTrack / 2.0,
			1.0, +WheelTrack / 2.0;
		this->j /= WheelRadius;

		// wheels to robot frame
		this->inv_j << 1.0, 1.0,
			-1.0 / (WheelTrack / 2.0), +1.0 / (WheelTrack / 2.0);
		this->inv_j *= WheelRadius / 2.0;

		//            std::cerr << "J: " << J << std::endl;
		//            std::cerr << "iJ: " << iJ << std::endl;
	}

	DifferentialDriveController(Fahrzeug_t *Fz, ReglerParam_t Regler, Abtastzeit_t Ta)
	{
		Init(Fz, Regler, Ta);
	}

	Pose<T> odometry(Pose<T> oldPose, const VelWheel &m) override
	{
		dPose<T> WFDelta, RFDelta;
		Pose<T> newPose;

		// transform wheel movements to vehicle movements in robot frame
		VelRF v;
		v = vWheel2vRF(m);

		RFDelta.x = v(0);
		RFDelta.y = 0.0;
		RFDelta.theta = v(1);

		// transform vehicle movements from robot frame to world frame
		WFDelta = dRF2dWF<T>(RFDelta, oldPose.theta + RFDelta.theta / T(2.0));

		// calculate new pose
		// newPose.x = oldPose.x + WFDelta.x;
		// newPose.y = oldPose.y + WFDelta.y;
		// newPose.theta = oldPose.theta + WFDelta.theta;

		// move theta to -PI .. +PI
		// newPose.theta = MAXPI(newPose.theta);

		newPose = oldPose + WFDelta;

		if (DeltaPoseCounter > 0) {
			newPose = newPose + DeltaPose;
			DeltaPoseCounter--;
		}

		if (DeltaHeadingCounter > 0) {
			newPose.theta += DeltaHeading;
			// newPose.theta = MAXPI(newPose.theta);
			DeltaHeadingCounter--;
		}

		return newPose;
	} // end of method Odometry

	VelRF poseControl(PoseV<T> ref, PoseV<T> act) override
	{
		dPose<T> PoseErrorWF, PoseErrorRF;
		vPose<T> WKSStellV, vWFref, vRFref;

		PoseErrorWF.x = ref.x - act.x;
		PoseErrorWF.y = ref.y - act.y;
		PoseErrorWF.theta = ref.theta - act.theta;

		// move theta to -PI .. +PI
		PoseErrorWF.theta = Heading<T>(PoseErrorWF.theta);

		// check error limits
		if ((std::abs(PoseErrorWF.x) > Regler.LageSchleppMax.x) || (std::abs(PoseErrorWF.y) > Regler.LageSchleppMax.y) || (std::abs(PoseErrorWF.theta) > Regler.LageSchleppMax.theta)) {
			log_message(log_debug, "%s, %s, deviation position controller too large: act.x: %f, ref.x: %f, act.y: %f, ref.y: %f, act.theta: %f, ref.theta: %f",
				__FILE__, __FUNCTION__, act.x, ref.x, act.y, ref.y, T(act.theta), T(ref.theta));
			log_message(log_error, "Max. Schleppabstand: %f, %f, %f, Schleppabstand %f, %f, %f,", Regler.LageSchleppMax.x,
				Regler.LageSchleppMax.y, Regler.LageSchleppMax.theta, PoseErrorWF.x, PoseErrorWF.y, PoseErrorWF.theta);
			throw MRC_LAGEERR;
		}

		// tranformation of pose error into robot frame
		PoseErrorRF = dWF2dRF<T>(PoseErrorWF, act.theta);

		// reference velocity in world frame
		vWFref.vx = ref.vx;
		vWFref.vy = ref.vy;
		vWFref.omega = ref.omega;

		// transformation to reference velocity in robot frame
		vRFref = vWF2vRF<T>(vWFref, act.theta);

		// new reference heading
		Heading<T> ThetaRef; // T ThetaRef;
		ThetaRef = ref.theta + T(Regler.LageKv.y * PoseErrorRF.y); // * vRFref.vx / Vmax; // reduce lead angle at low speed

		// vehicle moves backwards (usually manual mode)
		if (vRFref.vx < -1.0) {
			ThetaRef = ref.theta + T(Regler.LageKv.y * PoseErrorRF.y * vRFref.vx / Vmax); // reduce lead angle at low speed backwards
		}
#if 0            
		// robot moves vorwards and turns in direction of the y error
		if (vRFref.vx > 0.1) {
			ThetaRef = ref.theta + Regler.LageKv.y * std::abs(vRFref.vx) / Vmax * PoseErrorRF.y; //MAXPI(ref.theta + Regler.LageKv.y * PoseErrorRF.y);
		} else if (vRFref.vx < -0.1) { // robot moves backwards and turns in opposite direction 
			ThetaRef = ref.theta - Regler.LageKv.y * std::abs(vRFref.vx) / Vmax * PoseErrorRF.y; //MAXPI(ref.theta - Regler.LageKv.y * PoseErrorRF.y);
		} else {
			ThetaRef = ref.theta; //MAXPI(ref.theta);
		}
#endif
		PoseErrorRF.theta = Heading<T>(ThetaRef - act.theta);

		// control velocity in robot frame
		VelRF v;
		v(0) = vRFref.vx * std::cos(PoseErrorRF.theta) + Regler.LageKv.x * PoseErrorRF.x;
		// v(1) = vRFref.omega + Regler.LageKv.theta * PoseErrorRF.y;
		v(1) = vRFref.omega + Regler.LageKv.theta * std::sin(PoseErrorRF.theta) * (std::abs(vRFref.omega) / Omega_max + std::abs(vRFref.vx) / Vmax);

		return v;
	} // end of method PoseControl

	VelWheel wheelControl(const VelWheel &ReferenceVel, const VelWheel &ActualVel) override
	{
		T rabw, diff;
		VelWheel StellV;

		for (int i = 0; i < 2; i++) {
			// Regelabweichung, Sollwert und Istwert in rad/s
			rabw = ReferenceVel(i) - ActualVel(i);

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
			StellV(i) += ReferenceVel(i);

			// Anpassung an individuellen Verstaerker
			StellV(i) *= Regler.DzrSkalierung[i];

			RAbwAlt(i) = rabw;
		}
		return StellV;
	}

	vPose<T> velocityFilter(vPose<T> vRFref, vPose<T> vRFold) override
	{
		T v_diff;
		vPose<T> vRFnew;
		vRFnew.vx = vRFref.vx;
		vRFnew.vy = 0.0;
		vRFnew.omega = vRFref.omega;

		v_diff = JoystickAcc * Ta.FzLage;

		if (vRFref.vx - vRFold.vx > v_diff) {
			vRFnew.vx = vRFold.vx + v_diff;
		} else if (vRFold.vx - vRFref.vx > v_diff) {
			vRFnew.vx = vRFold.vx - v_diff;
		}

		v_diff /= (WheelTrack / 2.0);

		if (vRFref.omega - vRFold.omega > v_diff) {
			vRFnew.omega = vRFold.omega + v_diff;
		} else if (vRFold.omega - vRFref.omega > v_diff) {
			vRFnew.omega = vRFold.omega - v_diff;
		}
		return vRFnew;
	}

	void poseUpdate(dPose<T> Delta, unsigned int Divisor) override
	{
		DeltaPoseCounter = Divisor;
		DeltaPose = Delta / (T)Divisor;
	}

	void headingUpdate(T Delta, unsigned int Divisor) override
	{
		DeltaHeadingCounter = Divisor;
		DeltaHeading = Delta / (T)Divisor;
	}
};
}
