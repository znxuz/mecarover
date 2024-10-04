#pragma once

#include <algorithm>
#include <cstring>
#include <mecarover/controls/MecanumController.h>
#include <mecarover/hal/stm_hal.hpp>
#include <mecarover/mrcpptypes.h>
#include <mecarover/robot_params.hpp>
#include <mecarover/rtos_config.h>
#include <mecarover/rtos_utils.h>
#include <utility>

extern "C"
{
void call_pose_control_task(void* arg);
void call_wheel_control_task(void* arg);
}

namespace imsl::vehiclecontrol
{

enum class CtrlMode {
	OFF,
	TWIST,
};

template<typename T>
class ControllerTask {
private:
	using VelWheel = typename MecanumController<T>::VelWheel;
	using VelRF = typename MecanumController<T>::VelRF;
	MecanumController<T> controller;
	RT_Mutex ContrMutex; // Mutex for controller object

	/* Uebergabe der Drehzahlsollwerte -> Drehzahlregler in rad/s */
	T RadSollWert[4] = {};
	RT_Mutex SollwMutex;

	RT_Task lageregler_thread;
	RT_Task drehzahlregler_thread;

	RT_Semaphore pose_ctrl_sem;

	Pose<T> PosAkt;
	RT_Mutex PosAktMut;

	RT_Mutex ControllerModeMut;
	//    mrc_mode ControllerMode = MRC_OFF;
	//    MutexObject<CtrlMode> ControllerMode = CtrlMode::OFF;
	CtrlMode ControllerMode = CtrlMode::OFF;

	RT_Mutex ControllerErrorStatusMut;
	mrc_stat Fz_Stoerung = MRC_NOERR;

	RT_Mutex ManuRefVelMut;
	vPose<T> ManuRefVel;

	ReglerParam_t Regler;
	Abtastzeit_t Ta;

	void ManualModeInterface(vPose<T>& vel_rframe_sp, vPose<T>& vel_rframe_prev,
							 Pose<T>& pose_manual)
	{
		vel_rframe_sp
			= controller.velocityFilter(vel_rframe_sp, vel_rframe_prev);

		vPose<T> vel_wframe_sp = vRF2vWF<T>(vel_rframe_sp, getPose().theta);

		pose_manual.x += vel_wframe_sp.vx * Ta.FzLage;
		pose_manual.y += vel_wframe_sp.vy * Ta.FzLage;
		pose_manual.theta += vel_wframe_sp.omega * Ta.FzLage;
	}

	void mr_radsollwert_set(const T* sollw)
	{
		SollwMutex.lock();
		for (int i = 0; i < controller.N_WHEEL; i++)
			RadSollWert[i] = sollw[i];
		SollwMutex.unlock();
	}

	int mr_radsollwert_get(T* sollw)
	{
		SollwMutex.lock();
		for (int i = 0; i < controller.N_WHEEL; i++) {
			sollw[i] = RadSollWert[i];
		}
		SollwMutex.unlock();
		return 0;
	}

	void Odometry(const T* RadDeltaPhi)
	{
		VelRF vel_rframe_matrix = controller.vWheel2vRF(VelWheel(RadDeltaPhi));

		auto oldPose = getPose();
		ContrMutex.lock();
		Pose<T> newPose = controller.odometry(oldPose, vel_rframe_matrix);
		ContrMutex.unlock();
		setPose(newPose);

		dPose<T> vel_rframe{vel_rframe_matrix(0), vel_rframe_matrix(1),
							vel_rframe_matrix(2)};
		PosAktMut.lock();
		dPose<T> delta_wframe = dRF2dWF<T>(
			vel_rframe, PosAkt.theta + vel_rframe.theta / static_cast<T>(2.0));
		PosAkt.vx = delta_wframe.x / Ta.FzDreh;
		PosAkt.vy = delta_wframe.y / Ta.FzDreh;
		PosAkt.omega = delta_wframe.theta / Ta.FzDreh;
		PosAktMut.unlock();
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // eigenlib 16 Byte alignement

	vPose<T> GetManuRef()
	{
		vPose<T> vel;
		ManuRefVelMut.lock();
		vel = ManuRefVel;
		ManuRefVelMut.unlock();
		return vel;
	}

	void SetManuRef(vPose<T> vel)
	{
		ManuRefVelMut.lock();
		ManuRefVel = vel;
		ManuRefVelMut.unlock();
	}

	CtrlMode GetControllerMode()
	{
		CtrlMode mode;
		ControllerModeMut.lock();
		mode = ControllerMode;
		ControllerModeMut.unlock();
		return mode;
	}

	int SetControllerMode(CtrlMode mode)
	{
		ControllerModeMut.lock();
		ControllerMode = mode;
		ControllerModeMut.unlock();
		return 0;
	}

	mrc_stat GetControllerErrorStatus()
	{
		mrc_stat status;
		ControllerErrorStatusMut.lock();
		status = Fz_Stoerung;
		ControllerErrorStatusMut.unlock();
		return status;
	}

	int SetControllerErrorStatus(mrc_stat status)
	{
		ControllerErrorStatusMut.lock();
		Fz_Stoerung = status;
		ControllerErrorStatusMut.unlock();
		return 0;
	}

	int Init(const Fahrzeug_t* fz, ReglerParam_t Regler, Abtastzeit_t Ta)
	{
		log_message(log_info, "MecanumControllerTask init");

		this->controller.init(fz, Regler, Ta);
		this->ControllerMode = CtrlMode::OFF;
		this->Regler = Regler;
		this->Ta = Ta;

		if (!PosAktMut.create()) {
			log_message(log_error, "%s: Can init mutex", __FUNCTION__);
			return -1;
		}

		if (!pose_ctrl_sem.create(10, 0)) {
			log_message(log_error, "%s: Can init semaphore", __FUNCTION__);
			return -1;
		}

		if (!SollwMutex.create()) {
			log_message(log_error, "%s: Can init mutex", __FUNCTION__);
			return -1;
		}

		if (!ControllerModeMut.create()) {
			log_message(log_error, "%s: Can init mutex", __FUNCTION__);
			return -1;
		}

		if (!ControllerErrorStatusMut.create()) {
			log_message(log_error, "%s: Can init mutex", __FUNCTION__);
			return -1;
		}

		if (!ManuRefVelMut.create()) {
			log_message(log_error, "%s: Can init mutex", __FUNCTION__);
			return -1;
		}

		if (!drehzahlregler_thread.create(call_wheel_control_task,
											 "WheelControllerTask", STACK_SIZE,
											 this, WHEEL_CONTROLLER_PRIORITY)) {
			log_message(log_error, "Can not create drehzahlregler thread");
			return -1;
		}

		vTaskDelay(10); // wait some ticks to give wheel controller some time to start

		if (!lageregler_thread.create(call_pose_control_task,
				"PoseControllerTask", STACK_SIZE, this,
				POSE_CONTROLLER_PRIORITY)) {
			log_message(log_error, "Can not create lageregler thread");
			return -1;
		}

		return 0;
	}

	void PoseControlTask()
	{
		vPose<T> vel_rframe_sp;
		vPose<T> vel_rframe_prev;
		vPose<T> vel_wframe_sp;
		Pose<T> pose_sp;
		Pose<T> actual_pose;
		Pose<T> pose_manual;
		T vel_wheel_sp[4] = {};

		CtrlMode oldmode = CtrlMode::OFF;
		CtrlMode controllerMode = CtrlMode::OFF;

		size_t free_heap = xPortGetMinimumEverFreeHeapSize();
		uint32_t free_stack = lageregler_thread.getStackHighWaterMark();
		log_message(log_info,
					"pose controller initialized, running into control loop, "
					"free heap: %d, free stack: %ld",
					free_heap, free_stack);

		while (true) {
			pose_ctrl_sem.wait();
			pose_ctrl_sem.wait(); // TEST: why wait two times?

			oldmode = std::exchange(controllerMode, GetControllerMode());
			actual_pose = getPose();

			if (controllerMode == CtrlMode::OFF) {
				pose_sp = actual_pose;
				pose_sp.vx = pose_sp.vy = pose_sp.omega = 0.0;
				pose_manual = actual_pose;
				vel_rframe_prev = vPose<T>{};
				continue;
			}

			/* First time after mode switch */
			if (oldmode != CtrlMode::TWIST)
				pose_manual = actual_pose;

			vel_rframe_sp = GetManuRef();

			// set pose_manual from velocity setpoint
			vel_rframe_sp
				= controller.velocityFilter(vel_rframe_sp, vel_rframe_prev);
			vPose<T> vel_wframe_sp = vRF2vWF<T>(vel_rframe_sp, getPose().theta);
			pose_manual.x += vel_wframe_sp.vx * Ta.FzLage;
			pose_manual.y += vel_wframe_sp.vy * Ta.FzLage;
			pose_manual.theta += vel_wframe_sp.omega * Ta.FzLage;

			static int count = 0;
			if (count++ > 100) {
				count = 0;
				log_message(log_info,
							"manual pose via velocity x: %f, y: %f, theta: %f",
							pose_manual.x, pose_manual.y, T(pose_manual.theta));
			}
			vel_rframe_prev = vel_rframe_sp;

			/* V von RKS -> WKS transformieren */
			vel_wframe_sp = vRF2vWF<T>(vel_rframe_sp, actual_pose.theta);

			pose_sp.x = pose_manual.x;
			pose_sp.y = pose_manual.y;
			pose_sp.theta = pose_manual.theta;
			pose_sp.vx = vel_wframe_sp.vx;
			pose_sp.vy = vel_wframe_sp.vy;
			pose_sp.omega = vel_wframe_sp.omega;

			try {
				// calculate reference velocities of the wheels
				VelWheel vel_wheel_sp_mtx = controller.vRF2vWheel(
					controller.poseControl(pose_sp, actual_pose));
				std::copy(vel_wheel_sp_mtx.data(),
						  vel_wheel_sp_mtx.data() + vel_wheel_sp_mtx.size(),
						  vel_wheel_sp);
			} catch (mrc_stat e) {
				SetControllerMode(CtrlMode::OFF);
				SetControllerErrorStatus(MRC_LAGEERR);
				log_message(log_error,
							"deviation position controller too large");
			}

			mr_radsollwert_set(vel_wheel_sp);
		}
	}

	void WheelControlTask()
	{
		T RadDeltaPhi[controller.N_WHEEL];
		T sollw[controller.N_WHEEL] = {}; /* lokaler Drehzahlsollwert der Achsen in rad/s */
		T Stellgroesse[controller.N_WHEEL] = {}; /* aufintegrierter Wert aus Regelabweichung  */
		T radgeschw[controller.N_WHEEL] = {};

		CtrlMode controllerMode;

		int counter = Ta.FzLageZuDreh;

		size_t free_heap = xPortGetMinimumEverFreeHeapSize();
		uint32_t free_stack = lageregler_thread.getStackHighWaterMark();
		log_message(log_info, "wheel controller initialized running into control loop, free heap: %d, free stack: %ld",
			free_heap, free_stack);

		RT_PeriodicTimer WheelControllerTimer(Ta.FzDreh * 1000); // periode in ms
		while (true) {
			controllerMode = GetControllerMode();

			switch (controllerMode) {
			case CtrlMode::OFF:
				std::memset(Stellgroesse, 0, sizeof(Stellgroesse));
				std::memset(sollw, 0, sizeof(sollw));
				break;
			case CtrlMode::TWIST:
				mr_radsollwert_get(sollw);
				break;
			}

			hal_encoder_read(RadDeltaPhi);

			for (int i = 0; i < controller.N_WHEEL; ++i)
				radgeschw[i] = RadDeltaPhi[i] / Ta.FzDreh;

			if (controllerMode != CtrlMode::OFF) {
				VelWheel correcting_vel_matrix = controller.wheelControl(
					VelWheel(sollw), VelWheel(radgeschw));

				std::copy(correcting_vel_matrix.data(),
						  correcting_vel_matrix.data()
							  + correcting_vel_matrix.size(),
						  Stellgroesse);
			}

			if (GetControllerErrorStatus())
				std::memset(Stellgroesse, 0, sizeof(Stellgroesse));

			hal_wheel_vel_set_pwm(Stellgroesse);

			Odometry(RadDeltaPhi);

			/* ++counter % n is undefined? */
			++counter;
			counter = counter % Ta.FzLageZuDreh;
			if (!counter)
				pose_ctrl_sem.signal();

			WheelControllerTimer.wait();
		}
	}

	Pose<T> getPose()
	{
		auto pose = Pose<T>{};

		PosAktMut.lock();
		pose = PosAkt;
		PosAktMut.unlock();

		return pose;
	}

	void setPose(const Pose<T>& pose)
	{
		PosAktMut.lock();
		PosAkt.x = pose.x;
		PosAkt.y = pose.y;
		PosAkt.theta = pose.theta;
		PosAktMut.unlock();
	}

};

}
