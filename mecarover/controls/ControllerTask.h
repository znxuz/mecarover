#pragma once

#include <RTOS.h>
#include <mecarover/hal/stm_hal.h>
#include <mrcpptypes.h>
#include <rtos_config.h>

#include "VehicleController.h"

extern "C"
{
void call_pose_control_task(void *arg);
void call_wheel_control_task(void *arg);

static inline int get_ref_pose(PoseV_t *p)
{
	// TODO has to be implemented
	// get reference pose from interpolator
	return 0;
}
}

namespace imsl::vehiclecontrol
{

enum class CtrlMode {
	OFF, // disable motors
	ESTOP, // enable motors, v = 0
	TWIST, // cmd_vel: velocity command from ros
	POSE // cmd_pose: pose command from ros
};

template <typename T>
class ControllerTask {
private:
	/* Uebergabe der Drehzahlsollwerte -> Drehzahlregler in rad/s */
	T RadSollWert[4] = { 0.0, 0.0, 0.0, 0.0 };
	RT_Mutex SollwMutex; // Mutex fuer Drehzahlsollwert Lageregler -> Drehzahlregler

	// Threads, Semaphores and Mutexes
	RT_Task lageregler_thread;
	RT_Task drehzahlregler_thread;

	RT_Semaphore LgrSchedSem; // Semphore zum Wecken des Lagereglers

	PoseV<T> PosAkt; /* Aktuelle Position des Roboters, alle Sensoren */
	Pose<T> OPosAkt; /* Aktuelle Position nur Odometrie */
	Pose<T> GOPosAkt; /* Aktuelle Position Odometrie mit Gyroskop Fusion */
	vPose<T> RKSVAkt; /* Aktuelle Geschwindigkeit in Roboterkoordinaten */
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

	void getPose(Pose<T> &pose)
	{ // private interface with C++-type
		PosAktMut.lock();
		pose.x = PosAkt.x;
		pose.y = PosAkt.y;
		pose.theta = PosAkt.theta;
		PosAktMut.unlock();
	}

	void setPose(const Pose<T> &pose)
	{ // private interface with C++-type
		PosAktMut.lock();
		PosAkt.x = pose.x;
		PosAkt.y = pose.y;
		PosAkt.theta = pose.theta;
		PosAktMut.unlock();
	}

	// Interfaces for sub classes
	virtual int PoseControlInterface(PoseV_t reference, PoseV_t actual, vPose<T> &RKSGeschw, T *CorrectingVel) = 0;
	virtual void WheelControlInterface(T *ReferenceVel, T *ActualVel, T *CorrectingVel) = 0;
	virtual void OdometryInterface(T *RadDeltaPhi, dPose<T> &VehicleMovement) = 0;
	virtual void ManualModeInterface(vPose<T> &vRFref, vPose<T> &vRFold, Pose<T> &ManuPose) = 0;
	virtual void PoseUpdate(dPose<T> Delta, unsigned int Divisor) = 0;
	virtual void HeadingUpdate(T Delta, unsigned int Divisor) = 0;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // eigenlib 16 Byte alignement

	// manual mode
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

	virtual int Init(Fahrzeug_t *fz, ReglerParam_t Regler, Abtastzeit_t Ta)
	{
		log_message(log_info, "ControllerTask Init");
		ControllerMode = CtrlMode::OFF;
		this->Regler = Regler;
		this->Ta = Ta;

		if (Ta.FzDreh > 0.0) {
			UseWheelControllerTask = true;
		} else {
			UseWheelControllerTask = false;
		}

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
		vTaskPrioritySet(NULL, static_cast<osPriority_t>(MAIN_TASK_PRIORITY));

		if (UseWheelControllerTask) {
			if (!drehzahlregler_thread.create(call_wheel_control_task,
					"WheelControllerTask", STACK_SIZE, this,
					WHEEL_CONTROLLER_PRIORITY)) {
				log_message(log_error, "Can not create drehzahlregler thread");
				return -1;
			}
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

	/*
	   int CleanUp() {

	   return 0;
	   } // end of method CleanUp

	   int Join(void **retval) {
	   if (UseWheelControllerTask) {
	//        pthread_join(drehzahlregler_thread, retval);
	}
	return; // pthread_join(lageregler_thread, retval);
	} // end of method Join
	*/

	void PoseControlTask()
	{
		Pose<T> ManuPose; // pose for manual mode
		vPose<T> wksSollV, RKSGeschwAlt;
		vPose<T> RKSGeschw; // Geschwindigkeiten in Roboterkoordinaten
		vPose<T> vRFref; // reference velocity of the pose in robot frame
		PoseV_t wksSoll = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, // wegen der Warnungen
			aktPose = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		//      PoseV<T> wksSollAlt;
		vPose<T> rksSollV; // reference pose and velocity in robot frame
		T vWheelref[4] = { 0.0, 0.0, 0.0, 0.0 }; // reference velocities for wheel control

		// unsigned int wksSollinit = 0;
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
			printf("POSETask: %ld\n", uwTick);
			LgrSchedSem.wait(); // wait for signal from wheel controller
								//        int64_t stop_time = __HAL_TIM_GET_COUNTER(&htim13); // get time in microseconds since start
								//        uint32_t elapsed_time = stop_time - start_time; // time needed for control loop
								//        if (elapsed_time > max_elapsed_time) {
								//          max_elapsed_time = elapsed_time;
								////          log_message(log_debug, "pose controller loop max calculation time: %i µs", max_elapsed_time);
								//          log_message(log_debug, "pose controller jitter: %li µs", max_elapsed_time - 15000);
								//        }
			LgrSchedSem.wait(); // wait for signal from wheel controller
								//        start_time = __HAL_TIM_GET_COUNTER(&htim13); // get time in microseconds since start

			// get actual pose
			GetPose(&aktPose); /* aktuelle Pose bestimmen */
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
				wksSoll.x = aktPose.x;
				wksSoll.y = aktPose.y;
				wksSoll.theta = aktPose.theta;
				wksSoll.vx = 0.0;
				wksSoll.vy = 0.0;
				wksSoll.omega = 0.0;
				ManuPose = aktPose;
				/*
				   ManuPose.x = aktPose.x;
				   ManuPose.y = aktPose.y;
				   ManuPose.theta = aktPose.theta;
				   */
				RKSGeschwAlt.vx = 0.0;
				RKSGeschwAlt.vy = 0.0;
				RKSGeschwAlt.omega = 0.0;
				break;
			case CtrlMode::TWIST:
				/* First time after mode switch */
				if (oldmode != CtrlMode::TWIST) {
					ManuPose = aktPose;
				}

				/* Manuelle Vorgabe der Geschwindiglkeiten im RKS */
				/* Werte von manueller RKS-Vorgabe lesen */
				vRFref = GetManuRef();

				// Sollwert auf maximale Beschleunigung beschraenken
				ManualModeInterface(vRFref, RKSGeschwAlt, ManuPose);

				if (count++ > 100) {
					count = 0;
					log_message(log_debug, "ManuPose x: %f, y: %f, theta: %f",
						ManuPose.x, ManuPose.y, T(ManuPose.theta));
				}

				RKSGeschwAlt = vRFref; // for manual mode velocity filter
				rksSollV = vRFref;

				/* V von RKS -> WKS transformieren */
				wksSollV = vRF2vWF<T>(vRFref, aktPose.theta);

				wksSoll.x = ManuPose.x;
				wksSoll.y = ManuPose.y;
				wksSoll.theta = ManuPose.theta;
				wksSoll.vx = wksSollV.vx;
				wksSoll.vy = wksSollV.vy;
				wksSoll.omega = wksSollV.omega;
				break;
			case CtrlMode::POSE:
				get_ref_pose(&wksSoll); // read reference pose from interpolator
				break;
			}

			if ((controllerMode != CtrlMode::OFF)
				&& (controllerMode != CtrlMode::ESTOP)) {
				PoseControlInterface(wksSoll, aktPose, RKSGeschw, vWheelref);

				// check if wheel controller task is active
				if (UseWheelControllerTask) {
					// set the velocities for the wheel controller
					if (mr_radsollwert_set(vWheelref) != 0) { // TODO HAL
															  // error in setting velocities, turn off pose control
															  //              hal_amplifiers_disable();
						SetControllerErrorStatus(MRC_LAGEERR);
						SetControllerMode(CtrlMode::OFF);
						log_message(log_error, "%s: Can not set RadSollWert",
							__FUNCTION__);
					}
				} else { // no wheel controller task, set velocities via CAN
					hal_wheel_vel_set(vWheelref); // TODO CAN-HAL
				}

				// set RKS vel to zero if error
				if (GetControllerErrorStatus() != MRC_NOERR) {
					RKSGeschw.vx = RKSGeschw.vy = RKSGeschw.omega = 0;
				}

				/* Alte Werte werden das erste mal gespeichert, notwendig weil direkt von MRC_OFF in MRC_HOLD gewechselt werden kann */
				// wksSollinit = 1;
			} // end of if ((controllerMode != CtrlMode::OFF) && (controllerMode != CtrlMode::ESTOP))

			//        wksSollAlt = wksSoll;     /* Für PositionHalten speichern */
		}
	}

	void WheelControlTask()
	{
		int counter;
		int i;

		T RadDeltaPhi[4]; /* DeltaPhi-Wert der Raeder in rad zwischen zwei Abtastschritten */
		T sollw[4]; /* lokaler Drehzahlsollwert der Achsen in rad/s */

		T IstwAlt[4]; /* vorhergehender Wert in rad/s */
		T SollwAlt[4]; /* vorhergehender Sollwert in rad/s */
		T DZRNullwert[4]; /* Nullwert fuer Drehzahlregler */
		T Stellgroesse[4]; /* aufintegrierter Wert aus Regelabweichung  */
		T *Sollwert; // Zeiger auf den aktuellen Sollwert-Vektor Umschalten 0, werte
		T radgeschw[4]; /* Current wheel speeds */

		/* Make compiler happy */
		(void)SollwAlt;
		(void)IstwAlt;

		CtrlMode controllerMode;

		for (i = 0; i < 4; i++) {
			Stellgroesse[i] = 0.0;
			DZRNullwert[i] = 0.0;
			SollwAlt[i] = 0.0;
			IstwAlt[i] = 0.0;
		}

		/* Sollwert-Zeiger auf die Nullwerte setzen */
		Sollwert = DZRNullwert;

		hal_encoder_read(RadDeltaPhi); // Initialisierung der Werte

		counter = Ta.FzLageZuDreh; // Initialisierung des Teilers fuer den Lagetakt

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
			printf("WheelTask: %ld\n", uwTick);

			//        // read E-Stop input
			if (hal_get_estop()) { // E-Stop is pressed
				if (controllerMode != CtrlMode::OFF) {
					controllerMode = CtrlMode::ESTOP;
					SetControllerMode(controllerMode); // switch off motors, no active stopping
				}
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
				for (i = 0; i < 4; i++) {
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
			} // end of switch (controllerMode)

			/* Istwerte (Zaehlerstand seit dem letzten Lesen) von den Encodern lesen */
			hal_encoder_read(RadDeltaPhi);

			for (i = 0; i < NumbWheels; i++) {
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
			counter++;
			counter = counter % Ta.FzLageZuDreh;
			if (counter == 0) {
				LgrSchedSem.signal();
			}

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

	int GetPose(PoseV_t *pose)
	{
		PosAktMut.lock();

		pose->x = PosAkt.x;
		pose->y = PosAkt.y;
		pose->theta = PosAkt.theta;
		pose->vx = PosAkt.vx;
		pose->vy = PosAkt.vy;
		pose->omega = PosAkt.omega;

		PosAktMut.unlock();
		return 0;
	}

	int SetPose(Pose_t *pose, int delta)
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

	int GetOPose(Pose_t *pose)
	{
		PosAktMut.lock();

		pose->x = OPosAkt.x;
		pose->y = OPosAkt.y;
		pose->theta = OPosAkt.theta;

		PosAktMut.unlock();
		return 0;
	}

	int GetGOPose(Pose_t *pose)
	{
		PosAktMut.lock();

		pose->x = GOPosAkt.x;
		pose->y = GOPosAkt.y;
		pose->theta = GOPosAkt.theta;

		PosAktMut.unlock();
		return 0;
	}

	int SetOPose(Pose_t *pose)
	{
		PosAktMut.lock();

		OPosAkt.x = pose->x;
		OPosAkt.y = pose->y;
		OPosAkt.theta = pose->theta;

		PosAktMut.unlock();

		return 0;
	}

	int SetGOPose(Pose_t *pose)
	{
		PosAktMut.lock();

		GOPosAkt.x = pose->x;
		GOPosAkt.y = pose->y;
		GOPosAkt.theta = pose->theta;

		PosAktMut.unlock();

		return 0;
	}

	int GetRFVel(Pose_t *vel)
	{
		PosAktMut.lock();

		vel->x = RKSVAkt.vx;
		vel->y = RKSVAkt.vy;
		vel->theta = RKSVAkt.omega;

		PosAktMut.unlock();
		return 0;
	}

	T GetSamplingTime(void)
	{
		return Ta.FzLage;
	}

	// TODO pruefen !!!
	int getOdometryHeading(T *odoHeading)
	{
		T oh;

		OdoHeadingMutex.lock();
		oh = OdoHeading;
		OdoHeadingMutex.unlock();

		*odoHeading = oh;
		return 0;
	}

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
		counter++;
		counter = counter % Ta.FzLageZuDreh;
		if (counter == 0) {
			LgrSchedSem.signal();
		}
	}

private:
	int mr_radsollwert_set(T *sollw)
	{
		SollwMutex.lock();
		for (int i = 0; i < NumbWheels; i++) {
			RadSollWert[i] = sollw[i];
		}
		SollwMutex.unlock();
		return 0;
	}

	int mr_radsollwert_get(T *sollw)
	{
		SollwMutex.lock();
		for (int i = 0; i < NumbWheels; i++) {
			sollw[i] = RadSollWert[i];
		}
		SollwMutex.unlock();
		return 0;
	}

	void Odometry(T *RadDeltaPhi, T timediff)
	{
		dPose<T> wfm, rfm;
		// rks_move_t RKS_delta;              // Movement in RKS frame  TODO wofür

		// call subclass
		OdometryInterface(RadDeltaPhi, rfm);

		RKSVAkt.vx = rfm.x / timediff;
		RKSVAkt.vy = rfm.y / timediff;
		RKSVAkt.omega = rfm.theta / timediff;

		// update odometry heading
		OdoHeadingMutex.lock();
		OdoHeading += rfm.theta;
		OdoHeadingMutex.unlock();

		PosAktMut.lock();

		// new odometry pose
		// movements in world frame with odo pose
		wfm = dRF2dWF<T>(rfm, OPosAkt.theta + rfm.theta / T(2.0));
		OPosAkt.x += wfm.x;
		OPosAkt.y += wfm.y;
		OPosAkt.theta += wfm.theta;

		// new gyro odo pose
		// movements in world frame with gyro odo pose
		wfm = dRF2dWF<T>(rfm, GOPosAkt.theta + rfm.theta / T(2.0));
		GOPosAkt.x += wfm.x;
		GOPosAkt.y += wfm.y;
		GOPosAkt.theta += wfm.theta;

		// movement with actual heading
		wfm = dRF2dWF<T>(rfm, PosAkt.theta + rfm.theta / T(2.0));

		PosAkt.vx = wfm.x / timediff;
		PosAkt.vy = wfm.y / timediff;
		PosAkt.omega = wfm.theta / timediff;

		PosAktMut.unlock();
	}
};

}
