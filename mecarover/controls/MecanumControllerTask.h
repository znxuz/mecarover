#pragma once

#include "ControllerTask.h"
#include "MecanumController.h"

namespace imsl::vehiclecontrol
{

template<typename T>
class MecanumControllerTask : public ControllerTask<T> {
private:
	using CT = ControllerTask<T>;

	MecanumController<T> Controller;
	RT_Mutex ContrMutex; // Mutex for controller object

public:
	//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // eigenlib 16 Byte alignement
	using WheelVel = Eigen::Matrix<T, 4, 1>;
	using VehicleVel = Eigen::Matrix<T, 4, 1>;

	int Init(Fahrzeug_t* fz, ReglerParam_t Regler, Abtastzeit_t Ta) override
	{
		log_message(log_info, "MecanumControllerTask init");
		CT::NumbWheels = 4;
		CT::DegFreed = 4;
		Controller.Init(fz, Regler, Ta);

		// ASK why commented out?? the mutex is used all over the place
		// if (!ContrMutex.create()) {
		//   log_message(log_error, "%s: Can init mutex: %s", __FUNCTION__,
		//   strerror(errno));
		//   return -1;
		// }

		return CT::Init(fz, Regler, Ta); // call base class Init
	}

protected:
	int PoseControlInterface(const Pose<T>& reference, const Pose<T>& actual, vPose<T>& RKSGeschw, T* ve_wheel_sp) override
	{
		VehicleVel RFVel; // vehicle velocity in robot frame
		WheelVel WheelRefVel; // reference velocities of the wheels

		try {
			// obtain velocity in robot frame
			RFVel = Controller.poseControl(reference, actual);
		} catch (mrc_stat e) { // Schleppfehler aufgetreten
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
			ve_wheel_sp[i] = WheelRefVel(i);
		}

		return 0;
	}

	void WheelControlInterface(const T* ReferenceVel, const T* ActualVel, T* CorrectingVel) override
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

	void OdometryInterface(const T* RadDeltaPhi, dPose<T>& robot_vel_out) override
	{
		WheelVel wheel_rotation_delta_matrix;
		for (int i = 0; i < CT::NumbWheels; i++)
			wheel_rotation_delta_matrix(i) = RadDeltaPhi[i];

		// multiply the wheel rotation delta to the inverse jacobi matrix to
		// get the robot velocity
		VehicleVel matrix_robot_vel = Controller.vWheel2vRF(wheel_rotation_delta_matrix);

		// convert vm to global type dPose<T>
		robot_vel_out.x = matrix_robot_vel(0);
		robot_vel_out.y = matrix_robot_vel(1);
		robot_vel_out.theta = matrix_robot_vel(2);

		auto oldPose = CT::getPose();

		ContrMutex.lock();
		Pose<T> newPose = Controller.odometry(oldPose, wheel_rotation_delta_matrix);
		ContrMutex.unlock();

		CT::setPose(newPose);
	}

	void ManualModeInterface(vPose<T>& vel_rframe_sp, vPose<T>& vel_rframe_old, Pose<T>& pose_manual) override
	{
		vel_rframe_sp = Controller.velocityFilter(vel_rframe_sp, vel_rframe_old);

		vPose<T> vel_wframe_sp = vRF2vWF<T>(vel_rframe_sp, CT::getPose().theta);

		// ASK: multiply the velocity by the sampling frequency to get the pose
		/* Positionssollwert anpassen, Geschwindigkeit WKS einstellen */
		pose_manual.x += vel_wframe_sp.vx * CT::Ta.FzLage;
		pose_manual.y += vel_wframe_sp.vy * CT::Ta.FzLage;
		pose_manual.theta += vel_wframe_sp.omega * CT::Ta.FzLage;

		// ASK: why count to 100 before logging?
		/*
		   static int count = 0;
		   if (count++ > 100) {
		   count = 0;
		   log_message(log_debug,"vWFref x: %f, y: %f, theta: %f", vWFref.vx, vWFref.vy, vWFref.omega);
		   }
		   */
	}

	// ASK functions below are not used, ask what they are for
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
