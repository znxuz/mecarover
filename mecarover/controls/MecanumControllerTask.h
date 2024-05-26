#pragma once

#include "ControllerTask.h"
#include "MecanumController.h"

namespace imsl::vehiclecontrol {

template <typename T>
class MecanumControllerTask : public ControllerTask<T> {
private:
	using CT = ControllerTask<T>;

	MecanumController<T> Controller;
	RT_Mutex ContrMutex; // Mutex for controller object

public:
	//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // eigenlib 16 Byte alignement
	using WheelVel = Eigen::Matrix<T, 4, 1>;
	using VehicleVel = Eigen::Matrix<T, 4, 1>;

protected:
	int PoseControlInterface(PoseV_t reference, PoseV_t actual, vPose<T> &RKSGeschw, T *vWheel_ref) override
	{
		VehicleVel RFVel; // vehicle velocity in robot frame
		WheelVel WheelRefVel; // reference velocities of the wheels

		try {
			// call pose control method, obtain velocity in robot frame
			RFVel = Controller.poseControl(reference, actual);
		} catch (mrc_stat e) { // Schleppfehler aufgetreten
							   //        hal_amplifiers_disable();
			CT::SetControllerMode(CtrlMode::OFF);

			CT::SetControllerErrorStatus(MRC_LAGEERR);
			log_message(log_error, "deviation position controller too large");
		}

		// convert to global native type vPose<T>
		RKSGeschw.vx = RFVel(0);
		RKSGeschw.vy = RFVel(1);
		RKSGeschw.omega = RFVel(2);

		// calculate reference velocities of the wheels
		WheelRefVel = Controller.vRF2vWheel(RFVel);

		// convert to global native type T*
		for (int i = 0; i < CT::NumbWheels; i++) {
			vWheel_ref[i] = WheelRefVel(i);
		}

		return 0;
	}

	void WheelControlInterface(T *ReferenceVel, T *ActualVel, T *CorrectingVel) override
	{
		WheelVel RefVel = WheelVel::Zero(),
				 ActVel = WheelVel::Zero(),
				 CorrVel = WheelVel::Zero();

		// convert to specific type WheelVel
		for (int i = 0; i < CT::NumbWheels; i++) {
			RefVel(i) = ReferenceVel[i];
			ActVel(i) = ActualVel[i];
		}

		// call wheel control algorithm
		try {
			CorrVel = Controller.wheelControl(RefVel, ActVel);

			// convert correcting variable to native type T*
			for (int i = 0; i < CT::NumbWheels; i++) {
				CorrectingVel[i] = CorrVel(i);
			}
		} // end of try
		catch (mrc_stat e) {
			//        hal_amplifiers_disable();
			CT::SetControllerMode(CtrlMode::OFF);

			CT::SetControllerErrorStatus(MRC_DRZERR);
			log_message(log_error, "Schleppfehler Drehzahlregler");
		}
	}

	void OdometryInterface(T *RadDeltaPhi, dPose<T> &VehicleMovement) override
	{
		WheelVel wm;
		VehicleVel vm;

		for (int i = 0; i < CT::NumbWheels; i++) {
			wm(i) = RadDeltaPhi[i];
		}

		// movements in robot frame
		vm = Controller.vWheel2vRF(wm);

		// convert vm to global type dPose<T>
		VehicleMovement.x = vm(0);
		VehicleMovement.y = vm(1);
		VehicleMovement.theta = vm(2);

		Pose<T> oldPose, newPose;
		CT::getPose(oldPose);

		ContrMutex.lock();
		newPose = Controller.odometry(oldPose, wm);
		ContrMutex.unlock();

		CT::setPose(newPose);
	}

	void ManualModeInterface(vPose<T> &vRFref, vPose<T> &vRFold, Pose<T> &ManuPose) override
	{
		dPose<T> PoseErrorWF, PoseErrorRF;
		Pose<T> aktPose;
		vPose<T> vWFref;

		vRFref = Controller.velocityFilter(vRFref, vRFold);

		CT::getPose(aktPose); // obtain actual pose

		/* V von RKS -> WKS transformieren */
		vWFref = vRF2vWF<T>(vRFref, aktPose.theta);

		/* Positionssollwert anpassen, Geschwindigkeit WKS einstellen */
		ManuPose.x += vWFref.vx * CT::Ta.FzLage;
		ManuPose.y += vWFref.vy * CT::Ta.FzLage;
		ManuPose.theta += vWFref.omega * CT::Ta.FzLage;
		/*
		   static int count = 0;
		   if (count++ > 100) {
		   count = 0;
		   log_message(log_debug,"vWFref x: %f, y: %f, theta: %f", vWFref.vx, vWFref.vy, vWFref.omega);
		   }
		   */
	}

	int Init(Fahrzeug_t *fz, ReglerParam_t Regler, Abtastzeit_t Ta) override
	{
		log_message(log_info, "MecanumControllerTask init");
		CT::NumbWheels = 4;
		CT::DegFreed = 4;
		Controller.Init(fz, Regler, Ta);

		// if (!ContrMutex.create()) {
		//   log_message(log_error, "%s: Can init mutex: %s", __FUNCTION__,
		//   strerror(errno));
		//   return -1;
		// }

		return CT::Init(fz, Regler, Ta); // call base class Init
	}

	/*
	   int CleanUp() override {
	   ContrMutex.Delete();
	   return CT::CleanUp(); // call base class CleanUp
	   } // end of method CleanUp
	   */
	void PoseUpdate(dPose<T> PoseDelta, unsigned int Divisor) override
	{
		ContrMutex.lock();
		Controller.poseUpdate(PoseDelta, Divisor);
		ContrMutex.unlock();
	}

	void HeadingUpdate(T HeadingDelta, unsigned int Divisor) override
	{
		ContrMutex.lock();
		Controller.headingUpdate(HeadingDelta, Divisor);
		ContrMutex.unlock();
	}
};

}
