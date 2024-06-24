#pragma once

#include <mecarover/RTOS.h>
#include <mecarover/hal/stm_hal.h>
#include <mecarover/mrcpptypes.h>
#include <mecarover/rtos_config.h>

#include "VehicleController.h"

extern "C"
{
void call_pose_control_task(void* arg);
void call_wheel_control_task(void* arg);

static inline int get_ref_pose(Pose_t p)
{
	// ASK has to be implemented
	// get reference pose from interpolator
	return 0;
}
}

namespace imsl::vehiclecontrol
{

enum class CtrlMode : int {
	OFF, // disable motors
	ESTOP, // enable motors, v = 0
	TWIST, // cmd_vel: velocity command from ros
	POSE // cmd_pose: pose command from ros
};

template<typename T>
class ControllerTask {
private:
	/* Uebergabe der Drehzahlsollwerte -> Drehzahlregler in rad/s */
	T RadSollWert[4] = {0.0, 0.0, 0.0, 0.0};
	RT_Mutex SollwMutex; // Mutex fuer Drehzahlsollwert Lageregler -> Drehzahlregler

	// Threads, Semaphores and Mutexes
	RT_Task lageregler_thread;
	RT_Task drehzahlregler_thread;

	RT_Semaphore LgrSchedSem; // Semphore zum Wecken des Lagereglers

	Pose<T> PosAkt; /* Aktuelle Position des Roboters, alle Sensoren */
	Pose<T> OPosAkt; /* Aktuelle Position nur Odometrie */
	Pose<T> GOPosAkt; /* Aktuelle Position Odometrie mit Gyroskop Fusion */
	vPose<T> rframe_vel_actual; /* Aktuelle Geschwindigkeit in Roboterkoordinaten */
	RT_Mutex PosAktMut; // Mutex fuer die Posen

	Heading<T> OdoHeading; // Heading of the vehicle calculated by odometry only
	RT_Mutex OdoHeadingMutex; // Mutex fuer OdoHeading

	bool UseWheelControllerTask = false;

	RT_Mutex ControllerModeMut;
	//    mrc_mode ControllerMode = MRC_OFF;
	//    MutexObject<CtrlMode> ControllerMode = CtrlMode::OFF;
	CtrlMode ControllerMode = CtrlMode::OFF;

	RT_Mutex ControllerErrorStatusMut;
	mrc_stat Fz_Stoerung = MRC_NOERR;

	RT_Mutex ManuRefVelMut;
	vPose<T> ManuRefVel;

protected:
	int NumbWheels = 0;
	int DegFreed = 0;
	ReglerParam_t Regler;
	Abtastzeit_t Ta;

	virtual void PoseControlInterface(const Pose<T>& pose_sp, const Pose<T>& actual_pose, T* vel_wheel_sp) = 0;
	virtual void WheelControlInterface(const T* ReferenceVel, const T* ActualVel, T* CorrectingVel) = 0;
	virtual void OdometryInterface(const T* RadDeltaPhi, dPose<T>& robot_vel) = 0;
	virtual void ManualModeInterface(vPose<T>& vel_rframe_sp, vPose<T>& vel_rframe_old, Pose<T>& pose_manual) = 0;
	virtual void PoseUpdate(dPose<T> Delta, unsigned int Divisor) = 0;
	virtual void HeadingUpdate(T Delta, unsigned int Divisor) = 0;

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
		//      log_message(log_debug,"vel x: %f, y: %f, theta: %f", vel.vx, vel.vy, vel.omega);
		ManuRefVelMut.lock();
		ManuRefVel = vel;
		// printf("SetManuRef: x: %f, y: %f, omega: %f\n", ManuRefVel.vx, ManuRefVel.vy, ManuRefVel.omega);
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

	virtual int Init(Fahrzeug_t* fz, ReglerParam_t Regler, Abtastzeit_t Ta)
	{
		log_message(log_info, "ControllerTask Init");
		ControllerMode = CtrlMode::OFF;
		this->Regler = Regler;
		this->Ta = Ta;

		UseWheelControllerTask = Ta.FzDreh > 0.0;

		if (!PosAktMut.create()) {
			log_message(log_error, "%s: Can init mutex", __FUNCTION__);
			return -1;
		}

		if (!OdoHeadingMutex.create()) {
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

		// log_message(log_debug, "main task old priority: %li", uxTaskPriorityGet(NULL));
		// probably don't need this line below
		// vTaskPrioritySet(NULL, static_cast<osPriority_t>(MAIN_TASK_PRIORITY));

		if (UseWheelControllerTask && !drehzahlregler_thread.create(call_wheel_control_task, "WheelControllerTask", STACK_SIZE, this, WHEEL_CONTROLLER_PRIORITY)) {
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
			// printf("POSETask: %ld\n", uwTick);
			LgrSchedSem.wait(); // wait for signal from wheel controller
								//        int64_t stop_time = __HAL_TIM_GET_COUNTER(&htim13); // get time in microseconds since start
								//        uint32_t elapsed_time = stop_time - start_time; // time needed for control loop
								//        if (elapsed_time > max_elapsed_time) {
								//          max_elapsed_time = elapsed_time;
								////          log_message(log_debug, "pose controller loop max calculation time: %i µs", max_elapsed_time);
								//          log_message(log_debug, "pose controller jitter: %li µs", max_elapsed_time - 15000);
								//        }
			// ASK why wait two times?
			LgrSchedSem.wait(); // wait for signal from wheel controller
								//        start_time = __HAL_TIM_GET_COUNTER(&htim13); // get time in microseconds since start

			// get actual pose
			actual_pose = getPose();
			oldmode = controllerMode;
			controllerMode = GetControllerMode(); // get actual controller mode from regelung.c

			switch (controllerMode) {
			case CtrlMode::ESTOP:
				//          hal_amplifiers_enable(); // active stopping !
				// set wheel velocities to zero
				for (int i = 0; i < 4; i++) {
					vWheelref[i] = 0;
				}
				break;
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
				// ASK: why count to 100 before logging
				if (count++ > 100) {
					count = 0;
					log_message(log_debug, "manual pose via velocity x: %f, y: %f, theta: %f",
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
			case CtrlMode::POSE:
				get_ref_pose(pose_sp); // TODO function to be implemented
										// read reference pose from interpolator
				break;
			}

			if ((controllerMode != CtrlMode::OFF)
				&& (controllerMode != CtrlMode::ESTOP)) {
				PoseControlInterface(pose_sp, actual_pose, vWheelref);

				if (UseWheelControllerTask) {
					mr_radsollwert_set(vWheelref);
				} else { // no wheel controller task, set velocities via CAN
					hal_wheel_vel_set(vWheelref);
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

		size_t free_heap = xPortGetMinimumEverFreeHeapSize(); // ESP.getMinFreeHeap(); //lowest level of free heap since boot
		uint32_t free_stack = lageregler_thread.getStackHighWaterMark();
		log_message(log_info, "wheel controller initialized running into control loop, free heap: %d, free stack: %ld",
			free_heap, free_stack);
		//      uint32_t max_elapsed_time = 10; // time needed for control loop
		//      int64_t start_time = __HAL_TIM_GET_COUNTER(&htim13); // get time in microseconds since start

		RT_PeriodicTimer WheelControllerTimer(Ta.FzDreh * 1000); // periode in ms
		int estopCounter = 0;
		while (true) {
			controllerMode = GetControllerMode();
			// printf("WheelTask: %ld\n", uwTick);

			// read E-Stop input - useless for the mecanum robot for now
			if (hal_get_estop() && controllerMode != CtrlMode::OFF) {
				controllerMode = CtrlMode::ESTOP;
				SetControllerMode(controllerMode);
			}

			switch (controllerMode) {
			case CtrlMode::ESTOP:
				//            hal_amplifiers_enable(); // active stopping !
				// set wheel velocities to zero
				for (int i = 0; i < 4; i++) { // drehzahlregler soll arbeiten
					Stellgroesse[i] = 0.0;
				}
				if (estopCounter++ > 10) {
					SetControllerMode(CtrlMode::OFF); // switch off motors, no active stopping
													  //            hal_amplifiers_disable();
					estopCounter = 0;
				}
				break;
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
			case CtrlMode::POSE:
			case CtrlMode::TWIST:
				/*
				 * Die Drehzahlsollwerte vom Lageregler lesen.
				 */
				mr_radsollwert_get(sollw);

				Sollwert = sollw; /* Sollwert-Zeiger darauf setzen */
				//				log_message(log_info, "Sollwert: %d\n", sollw);

				break;
			}

			/* Istwerte (Zaehlerstand seit dem letzten Lesen) von den Encodern lesen */
			hal_encoder_read(RadDeltaPhi);

			for (int i = 0; i < NumbWheels; i++) {
				// Radgeschwindigkeiten in rad/s fuer Client-Programme und Drehzahlregler
				radgeschw[i] = RadDeltaPhi[i] / Ta.FzDreh;
			}

			if ((controllerMode != CtrlMode::OFF)
				&& (controllerMode != CtrlMode::ESTOP)) {
				/*
				 * Drehzahlreglerfunktion aufrufen.
				 */
				WheelControlInterface(Sollwert, radgeschw, Stellgroesse);
			} else if (controllerMode != CtrlMode::OFF) {
				//          hal_amplifiers_disable();
			}

			//			printf("Stellgroesse2: %.2f\n", Stellgroesse[1]);
			hal_wheel_vel_set(Stellgroesse);

			// Bei allen Stoerungen Steller aus
			if (GetControllerErrorStatus()) {
				hal_wheel_vel_set(DZRNullwert);
				//          hal_amplifiers_disable();
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

	// TODO delete: unused
	int SetPose(Pose_t* pose, int delta)
	{
		PosAktMut.lock();

		if (delta) { // relative pose
			PosAkt.x -= pose->x;
			PosAkt.y -= pose->y;
			PosAkt.theta -= pose->theta;
		} else { // absolute pose
			PosAkt.x = pose->x;
			PosAkt.y = pose->y;
			PosAkt.theta = pose->theta;
		}

		PosAktMut.unlock();
		return 0;
	}

	int GetOPose(Pose_t* pose)
	{
		PosAktMut.lock();

		pose->x = OPosAkt.x;
		pose->y = OPosAkt.y;
		pose->theta = OPosAkt.theta;

		PosAktMut.unlock();
		return 0;
	}

	int GetGOPose(Pose_t* pose)
	{
		PosAktMut.lock();

		pose->x = GOPosAkt.x;
		pose->y = GOPosAkt.y;
		pose->theta = GOPosAkt.theta;

		PosAktMut.unlock();
		return 0;
	}

	int SetOPose(Pose_t* pose)
	{
		PosAktMut.lock();

		OPosAkt.x = pose->x;
		OPosAkt.y = pose->y;
		OPosAkt.theta = pose->theta;

		PosAktMut.unlock();

		return 0;
	}

	int SetGOPose(Pose_t* pose)
	{
		PosAktMut.lock();

		GOPosAkt.x = pose->x;
		GOPosAkt.y = pose->y;
		GOPosAkt.theta = pose->theta;

		PosAktMut.unlock();

		return 0;
	}

	int GetRFVel(Pose_t* vel)
	{
		PosAktMut.lock();

		vel->x = rframe_vel_actual.vx;
		vel->y = rframe_vel_actual.vy;
		vel->theta = rframe_vel_actual.omega;

		PosAktMut.unlock();
		return 0;
	}

	T GetSamplingTime(void)
	{
		return Ta.FzLage;
	}

	// TODO pruefen !!!
	int getOdometryHeading(T* odoHeading)
	{
		T oh;

		OdoHeadingMutex.lock();
		oh = OdoHeading;
		OdoHeadingMutex.unlock();

		*odoHeading = oh;
		return 0;
	}

	// unused
	void OdometrieCallback()
	{
		T RadDeltaPhi[4]; /* Geschwindigkeits-Istwert der Raeder in Rad  */
		T timediff = Ta.FzLage / Ta.FzLageZuDreh; // calculate cycle time of odometry

		if (hal_encoder_read(RadDeltaPhi) < 0) {
			SetControllerMode(CtrlMode::OFF);
			log_message(log_error, "%s/%s: Can not read encoder values",
				__FILE__, __FUNCTION__);
			// log_message(log_error, "error reading encoder values, switching controller off");
			// Fehlerbehandlung: Keine Encoder-Werte über CAN-Bus empfangen
		}

		Odometry(RadDeltaPhi, timediff);

		// wake up pose controller
		/*
		 * Den Lageregler im Teilerverhaeltnis Lage/Dreh mittels Event wecken
		 * Inkrement und Mudo-Operation müssen getrennt bleiben, sonst ist das
		 * Verhalten laut C-Standard undefiniert!
		 */
		static int counter = 0;
		++counter;
		counter = counter % Ta.FzLageZuDreh;
		if (counter == 0) {
			LgrSchedSem.signal();
		}
	}

private:
	void mr_radsollwert_set(const T* sollw)
	{
		SollwMutex.lock();
		for (int i = 0; i < NumbWheels; i++)
			RadSollWert[i] = sollw[i];
		SollwMutex.unlock();
	}

	int mr_radsollwert_get(T* sollw)
	{
		SollwMutex.lock();
		for (int i = 0; i < NumbWheels; i++) {
			sollw[i] = RadSollWert[i];
		}
		SollwMutex.unlock();
		return 0;
	}

	void Odometry(const T* RadDeltaPhi, T timediff)
	{
		dPose<T> wfm, robot_vel;

		// call subclass
		OdometryInterface(RadDeltaPhi, robot_vel);

		rframe_vel_actual.vx = robot_vel.x / timediff;
		rframe_vel_actual.vy = robot_vel.y / timediff;
		rframe_vel_actual.omega = robot_vel.theta / timediff;

		// update odometry heading
		OdoHeadingMutex.lock();
		OdoHeading += robot_vel.theta;
		OdoHeadingMutex.unlock();

		PosAktMut.lock();

		// new odometry pose
		// movements in world frame with odo pose
		wfm = dRF2dWF<T>(robot_vel, OPosAkt.theta + robot_vel.theta / T(2.0));
		OPosAkt.x += wfm.x;
		OPosAkt.y += wfm.y;
		OPosAkt.theta += wfm.theta;

		// new gyro odo pose
		// movements in world frame with gyro odo pose
		wfm = dRF2dWF<T>(robot_vel, GOPosAkt.theta + robot_vel.theta / T(2.0));
		GOPosAkt.x += wfm.x;
		GOPosAkt.y += wfm.y;
		GOPosAkt.theta += wfm.theta;

		// movement with actual heading
		wfm = dRF2dWF<T>(robot_vel, PosAkt.theta + robot_vel.theta / T(2.0));

		PosAkt.vx = wfm.x / timediff;
		PosAkt.vy = wfm.y / timediff;
		PosAkt.omega = wfm.theta / timediff;

		PosAktMut.unlock();
	}
};

}
