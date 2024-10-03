#pragma once

#include <mecarover/controls/MecanumController.h>
#include <mecarover/hal/stm_hal.hpp>
#include <mecarover/mrcpptypes.h>
#include <mecarover/robot_params.hpp>
#include <mecarover/rtos_config.h>
#include <mecarover/rtos_utils.h>

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
	T RadSollWert[4] = {0.0, 0.0, 0.0, 0.0};
	RT_Mutex SollwMutex; // Mutex fuer Drehzahlsollwert Lageregler -> Drehzahlregler

	// Threads, Semaphores and Mutexes
	RT_Task lageregler_thread;
	RT_Task drehzahlregler_thread;

	RT_Semaphore LgrSchedSem; // Semphore zum Wecken des Lagereglers

	Pose<T> PosAkt; /* Aktuelle Position des Roboters, alle Sensoren */
	RT_Mutex PosAktMut; // Mutex fuer die Posen

	bool UseWheelControllerTask = false;

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

	void pose_control(const Pose<T>& pose_sp,
							  const Pose<T>& actual_pose,
							  T* vel_wheel_sp)
	{
		try {
			VelRF vel_rframe_sp_mtx
				= controller.poseControl(pose_sp, actual_pose);

			// calculate reference velocities of the wheels
			VelWheel vel_wheel_sp_mtx
				= controller.vRF2vWheel(vel_rframe_sp_mtx);

			for (int i = 0; i < controller.N_WHEEL; i++)
				vel_wheel_sp[i] = vel_wheel_sp_mtx(i);
		} catch (mrc_stat e) { // Schleppfehler aufgetreten
			SetControllerMode(CtrlMode::OFF);
			SetControllerErrorStatus(MRC_LAGEERR);
			log_message(log_error, "deviation position controller too large");
		}
	}

	void wheel_control(const T* ReferenceVel, const T* ActualVel,
							   T* CorrectingVel)
	{
		VelWheel RefVel = VelWheel::Zero(), ActVel = VelWheel::Zero(),
				 CorrVel = VelWheel::Zero();

		// convert to specific type VelWheel
		for (int i = 0; i < controller.N_WHEEL; i++) {
			RefVel(i) = ReferenceVel[i];
			ActVel(i) = ActualVel[i];
		}

		// call wheel control algorithm
		try {
			CorrVel = controller.wheelControl(RefVel, ActVel);

			// convert correcting variable to native type T*
			for (int i = 0; i < controller.N_WHEEL; i++) {
				CorrectingVel[i] = CorrVel(i);
			}
		} // end of try
		catch (mrc_stat e) {
			//        hal_amplifiers_disable();
			SetControllerMode(CtrlMode::OFF);

			SetControllerErrorStatus(MRC_DRZERR);
			log_message(log_error, "Schleppfehler Drehzahlregler");
		}
	}

	void ManualModeInterface(vPose<T>& vel_rframe_sp, vPose<T>& vel_rframe_prev,
							 Pose<T>& pose_manual)
	{
		vel_rframe_sp
			= controller.velocityFilter(vel_rframe_sp, vel_rframe_prev);

		vPose<T> vel_wframe_sp = vRF2vWF<T>(vel_rframe_sp, getPose().theta);

		pose_manual.x += vel_wframe_sp.vx * Ta.FzLage;
		pose_manual.y += vel_wframe_sp.vy * Ta.FzLage;
		pose_manual.theta += vel_wframe_sp.omega * Ta.FzLage;

		/*
		   static int count = 0;
		   if (count++ > 100) {
		   count = 0;
		   log_message(log_debug,"vWFref x: %f, y: %f, theta: %f", vWFref.vx,
		   vWFref.vy, vWFref.omega);
		   }
		   */
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

	void Odometry(const T* RadDeltaPhi, T timediff) // timediff = Fz.Dreh
	{
		// TODO: search if eigen matrix really can be assigned directly with a
		// raw array
		VelWheel wheel_rotation_delta_matrix;
		for (int i = 0; i < controller.N_WHEEL; i++)
			wheel_rotation_delta_matrix(i) = RadDeltaPhi[i];

		VelRF matrix_robot_vel
			= controller.vWheel2vRF(wheel_rotation_delta_matrix);

		dPose<T> vel_rframe;
		vel_rframe.x = matrix_robot_vel(0);
		vel_rframe.y = matrix_robot_vel(1);
		vel_rframe.theta = matrix_robot_vel(2);

		auto oldPose = getPose();
		ContrMutex.lock();
		Pose<T> newPose
			= controller.odometry(oldPose, wheel_rotation_delta_matrix);
		ContrMutex.unlock();
		setPose(newPose);

		PosAktMut.lock();
		dPose<T> delta_wframe = dRF2dWF<T>(
			vel_rframe, PosAkt.theta + vel_rframe.theta / static_cast<T>(2.0));
		PosAkt.vx = delta_wframe.x / timediff;
		PosAkt.vy = delta_wframe.y / timediff;
		PosAkt.omega = delta_wframe.theta / timediff;
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

		UseWheelControllerTask = Ta.FzDreh > 0.0;

		if (!PosAktMut.create()) {
			log_message(log_error, "%s: Can init mutex", __FUNCTION__);
			return -1;
		}

		if (!LgrSchedSem.create(10, 0)) {
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

		if (UseWheelControllerTask
			&& !drehzahlregler_thread.create(call_wheel_control_task,
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
		vPose<T> vel_wframe_sp, vel_rframe_prev;
		vPose<T> vel_rframe_sp; // reference velocity of the pose in robot frame
		Pose<T> pose_sp;
		Pose<T> actual_pose;
		Pose<T> pose_manual;
		T vWheelref[4] = {0.0, 0.0, 0.0, 0.0};

		CtrlMode oldmode = CtrlMode::OFF;
		CtrlMode controllerMode = CtrlMode::OFF;

		size_t free_heap = xPortGetMinimumEverFreeHeapSize(); // ESP.getMinFreeHeap(); //lowest level of free heap since boot
		uint32_t free_stack = lageregler_thread.getStackHighWaterMark();
		//		uint32_t max_elapsed_time = 10; // time needed for control loop
		//		int64_t start_time = __HAL_TIM_GET_COUNTER(&htim13); // get time in microseconds since start
		log_message(log_info,
			"pose controller initialized, running into control loop, free heap: %d, free stack: %ld",
			free_heap, free_stack);

		int count = 0;
		while (true) {
			LgrSchedSem.wait(); // wait for signal from wheel controller
								//        int64_t stop_time = __HAL_TIM_GET_COUNTER(&htim13); // get time in microseconds since start
								//        uint32_t elapsed_time = stop_time - start_time; // time needed for control loop
								//        if (elapsed_time > max_elapsed_time) {
								//          max_elapsed_time = elapsed_time;
								////          log_message(log_debug, "pose controller loop max calculation time: %i µs", max_elapsed_time);
								//          log_message(log_debug, "pose controller jitter: %li µs", max_elapsed_time - 15000);
								//        }
			// QUESTION: why wait two times?
			LgrSchedSem.wait(); // wait for signal from wheel controller
								//        start_time = __HAL_TIM_GET_COUNTER(&htim13); // get time in microseconds since start

			// get actual pose
			actual_pose = getPose();
			oldmode = controllerMode;
			controllerMode = GetControllerMode(); // get actual controller mode from regelung.c

			switch (controllerMode) {
			case CtrlMode::OFF:
				pose_sp.x = actual_pose.x;
				pose_sp.y = actual_pose.y;
				pose_sp.theta = actual_pose.theta;
				pose_sp.vx = 0.0;
				pose_sp.vy = 0.0;
				pose_sp.omega = 0.0;
				pose_manual = actual_pose;
				/*
				   ManuPose.x = aktPose.x;
				   ManuPose.y = aktPose.y;
				   ManuPose.theta = aktPose.theta;
				   */
				vel_rframe_prev.vx = 0.0;
				vel_rframe_prev.vy = 0.0;
				vel_rframe_prev.omega = 0.0;
				break;
			case CtrlMode::TWIST:
				/* First time after mode switch */
				if (oldmode != CtrlMode::TWIST)
					pose_manual = actual_pose;

				/* Manuelle Vorgabe der Geschwindiglkeiten im RKS */
				/* Werte von manueller RKS-Vorgabe lesen */
				vel_rframe_sp = GetManuRef();

				// set pose_manual from vel
				ManualModeInterface(vel_rframe_sp, vel_rframe_prev, pose_manual);
				if (count++ > 100) {
					count = 0;
					log_message(log_info, "manual pose via velocity x: %f, y: %f, theta: %f",
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
				break;
			}

			if (controllerMode != CtrlMode::OFF) {
				pose_control(pose_sp, actual_pose, vWheelref);

				if (UseWheelControllerTask) {
					mr_radsollwert_set(vWheelref);
				} else { // no wheel controller task, set velocities via CAN
					hal_wheel_vel_set_pwm(vWheelref);
				}
			}
		}
	}

	void WheelControlTask()
	{
		T RadDeltaPhi[4]; /* DeltaPhi-Wert der Raeder in rad zwischen zwei Abtastschritten */
		T sollw[4]; /* lokaler Drehzahlsollwert der Achsen in rad/s */

		T IstwAlt[4]; /* vorhergehender Wert in rad/s */
		T SollwAlt[4]; /* vorhergehender Sollwert in rad/s */
		T DZRNullwert[4]; /* Nullwert fuer Drehzahlregler */
		T Stellgroesse[4]; /* aufintegrierter Wert aus Regelabweichung  */
		T* Sollwert; // Zeiger auf den aktuellen Sollwert-Vektor Umschalten 0, werte
		T radgeschw[4]; /* Current wheel speeds */

		/* Make compiler happy */
		(void)SollwAlt;
		(void)IstwAlt;

		CtrlMode controllerMode;

		for (int i = 0; i < 4; i++) {
			Stellgroesse[i] = 0.0;
			DZRNullwert[i] = 0.0;
			SollwAlt[i] = 0.0;
			IstwAlt[i] = 0.0;
		}

		/* Sollwert-Zeiger auf die Nullwerte setzen */
		Sollwert = DZRNullwert;

		hal_encoder_read(RadDeltaPhi); // Initialisierung der Werte

		// Initialisierung des Teilers fuer den Lagetakt
		// ASK can it also be 0?
		int counter = Ta.FzLageZuDreh;

		size_t free_heap = xPortGetMinimumEverFreeHeapSize();
		uint32_t free_stack = lageregler_thread.getStackHighWaterMark();
		log_message(log_info, "wheel controller initialized running into control loop, free heap: %d, free stack: %ld",
			free_heap, free_stack);
		//      uint32_t max_elapsed_time = 10; // time needed for control loop
		//      int64_t start_time = __HAL_TIM_GET_COUNTER(&htim13); // get time in microseconds since start

		RT_PeriodicTimer WheelControllerTimer(Ta.FzDreh * 1000); // periode in ms
		while (true) {
			controllerMode = GetControllerMode();

			switch (controllerMode) {
			case CtrlMode::OFF:
				for (int i = 0; i < 4; i++) {
					Stellgroesse[i] = 0.0;
					SollwAlt[i] = 0.0;
					sollw[i] = 0.0;
					IstwAlt[i] = 0.0;
				}
				/* kein break !!! */
				//        case MRC_NEUTRAL:
				Sollwert = DZRNullwert;
				break;
				//        case MRC_HOLD:
			case CtrlMode::TWIST:
				/* Die Drehzahlsollwerte vom Lageregler lesen. */
				mr_radsollwert_get(sollw);
				Sollwert = sollw; /* Sollwert-Zeiger darauf setzen */
				//				log_message(log_info, "Sollwert: %d\n", sollw);

				break;
			}

			/* Istwerte (Zaehlerstand seit dem letzten Lesen) von den Encodern lesen */
			hal_encoder_read(RadDeltaPhi);

			for (int i = 0; i < controller.N_WHEEL; i++) {
				// Radgeschwindigkeiten in rad/s fuer Client-Programme und Drehzahlregler
				radgeschw[i] = RadDeltaPhi[i] / Ta.FzDreh;
			}

			if (controllerMode != CtrlMode::OFF) {
				/*
				 * Drehzahlreglerfunktion aufrufen.
				 */
				wheel_control(Sollwert, radgeschw, Stellgroesse);
			}

			hal_wheel_vel_set_pwm(Stellgroesse);

			// Bei allen Stoerungen Steller aus
			if (GetControllerErrorStatus()) {
				hal_wheel_vel_set_pwm(DZRNullwert);
			}

			Odometry(RadDeltaPhi, Ta.FzDreh);

			/* Lageregler wecken */
			/*
			 * Den Lageregler im Teilerverhaeltnis Lage/Dreh mittels Event wecken
			 * Inkrement und Mudo-Operation müssen getrennt bleiben, sonst ist das
			 * Verhalten laut C-Standard undefiniert!
			 */
			++counter;
			counter = counter % Ta.FzLageZuDreh;
			if (!counter)
				LgrSchedSem.signal();

			// wait for the next tick
			WheelControllerTimer.wait();

			// measure the time needed in loop
			/*
			 * Drehzahlregler wird 5-mal aufgerufen bevor der Lageregler aufgerufen wird und neue Werte berechnet.
			 */

			//        int64_t stop_time = __HAL_TIM_GET_COUNTER(&htim13); // get time in microseconds since start
			//        uint32_t elapsed_time = stop_time - start_time; // time needed for control loop
			//        if (elapsed_time > max_elapsed_time) {
			//          max_elapsed_time = elapsed_time;
			////          log_message(log_debug, "wheel controller loop max calculation time: %i µs", max_elapsed_time);
			//          log_message(log_debug, "wheel jitter: %li µs", max_elapsed_time - 3000);
			//        }
			//        start_time = __HAL_TIM_GET_COUNTER(&htim13); // get time in microseconds since start
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
