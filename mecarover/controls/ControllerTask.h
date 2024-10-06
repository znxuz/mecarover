#pragma once

#include <algorithm>
#include <array>
#include <cstring>
#include <utility>

#include <mecarover/controls/MecanumController.h>
#include <mecarover/hal/stm_hal.hpp>
#include <mecarover/mrcpptypes.h>
#include <mecarover/mrtypes.h>
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

template<typename T> class ControllerTask {
private:
	using VelWheel = typename MecanumController<T>::VelWheel;
	using VelRF = typename MecanumController<T>::VelRF;

	MecanumController<T> controller;
	ctrl_param_t ctrl_params;
	sampling_time_t sampling_times;

	RT_Task lageregler_thread{call_pose_control_task, "PoseControllerTask",
							  MAIN_TASK_STACK_SIZE, this,
							  POSE_CONTROLLER_PRIORITY};
	RT_Task drehzahlregler_thread{call_wheel_control_task,
								  "WheelControllerTask", MAIN_TASK_STACK_SIZE,
								  this, WHEEL_CONTROLLER_PRIORITY};
	RT_Semaphore pose_ctrl_sem{10, 0};

	MutexObject<std::array<T, 4>> vel_wheel_sp;
	MutexObject<mrc_stat> error_status{MRC_NOERR};

public:
	MutexObject<CtrlMode> ctrl_mode;
	MutexObject<Pose<T>> pose_current;
	MutexObject<vPose<T>> ref_vel_manual;

	void init(const robot_param_t* robot_params, ctrl_param_t ctrl_params,
			  sampling_time_t sampling_times)
	{
		log_message(log_info, "controller tasks init");

		this->controller.init(robot_params, ctrl_params, sampling_times);
		this->ctrl_params = ctrl_params;
		this->sampling_times = sampling_times;
	}

	void PoseControlTask()
	{
		vPose<T> vel_rframe_sp;
		vPose<T> vel_rframe_prev;
		vPose<T> vel_wframe_sp;
		Pose<T> pose_sp;
		Pose<T> pose_current;
		Pose<T> pose_manual;

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
			pose_ctrl_sem
				.wait(); // FIXME: without the second wait the vel changes too
						 // fast so that the motors get locked up

			oldmode = std::exchange(controllerMode, ctrl_mode.get());
			pose_current = this->pose_current.get();

			if (controllerMode == CtrlMode::OFF) {
				pose_sp = pose_manual = pose_current;
				pose_sp.velocity = vel_rframe_prev = vPose<T>{};
				continue;
			}

			/* First time after mode switch */
			if (oldmode != CtrlMode::TWIST)
				pose_manual = pose_current;

			vel_rframe_sp = ref_vel_manual.get();

			// set pose_manual from velocity setpoint
			vel_rframe_sp
				= controller.velocityFilter(vel_rframe_sp, vel_rframe_prev);
			vPose<T> vel_wframe_sp
				= vRF2vWF<T>(vel_rframe_sp, this->pose_current.get().theta);
			pose_manual.x += vel_wframe_sp.vx * sampling_times.FzLage;
			pose_manual.y += vel_wframe_sp.vy * sampling_times.FzLage;
			pose_manual.theta += vel_wframe_sp.omega * sampling_times.FzLage;

			static int count = 0;
			if (count++ > 100) {
				count = 0;
				log_message(log_info,
							"manual pose via velocity x: %f, y: %f, theta: %f",
							pose_manual.x, pose_manual.y, T(pose_manual.theta));
			}
			vel_rframe_prev = vel_rframe_sp;

			/* V von RKS -> WKS transformieren */
			vel_wframe_sp = vRF2vWF<T>(vel_rframe_sp, pose_current.theta);

			pose_sp.x = pose_manual.x;
			pose_sp.y = pose_manual.y;
			pose_sp.theta = pose_manual.theta;
			pose_sp.velocity = vel_wframe_sp;

			try {
				// calculate reference velocities of the wheels
				VelWheel vel_wheel_sp_mtx = controller.vRF2vWheel(
					controller.poseControl(pose_sp, pose_current));
				auto vel_wheel_sp = std::array<T, N_WHEEL>{};
				std::copy(std::begin(vel_wheel_sp_mtx),
						  std::end(vel_wheel_sp_mtx), begin(vel_wheel_sp));
				this->vel_wheel_sp.set(vel_wheel_sp);
			} catch (mrc_stat e) {
				ctrl_mode.set(CtrlMode::OFF);
				error_status.set(MRC_LAGEERR);
				log_message(log_error,
							"deviation position controller too large");
			}
		}
	}

	void WheelControlTask()
	{
		std::array<T, N_WHEEL> encoder_deltas{};
		std::array<T, N_WHEEL> vel_wheel_sp{};
		std::array<T, N_WHEEL> vel_wheel_actual{};
		std::array<T, N_WHEEL> vel_wheel_corrected{};

		CtrlMode controllerMode;

		int counter = sampling_times.FzLageZuDreh;

		size_t free_heap = xPortGetMinimumEverFreeHeapSize();
		uint32_t free_stack = lageregler_thread.getStackHighWaterMark();
		log_message(log_info,
					"wheel controller initialized running into control loop, "
					"free heap: %d, free stack: %ld",
					free_heap, free_stack);

		RT_PeriodicTimer WheelControllerTimer(sampling_times.FzDreh * MS_TO_S);
		while (true) {
			controllerMode = ctrl_mode.get();

			switch (controllerMode) {
			case CtrlMode::OFF:
				std::fill(begin(vel_wheel_corrected), end(vel_wheel_corrected),
						  0);
				std::fill(begin(vel_wheel_sp), end(vel_wheel_sp), 0);
				break;
			case CtrlMode::TWIST:
				vel_wheel_sp = this->vel_wheel_sp.get();
				break;
			}

			encoder_deltas = hal_encoder_delta();
			std::transform(begin(encoder_deltas), end(encoder_deltas),
						   begin(vel_wheel_actual), [this](T encoder_delta) {
							   return encoder_delta / sampling_times.FzDreh;
						   });

			if (controllerMode != CtrlMode::OFF) {
				VelWheel correcting_vel_matrix = controller.wheelControl(
					VelWheel(vel_wheel_sp.data()),
					VelWheel(vel_wheel_actual.data()));
				std::copy(std::begin(correcting_vel_matrix),
						  std::end(correcting_vel_matrix),
						  begin(vel_wheel_corrected));
			}
			if (error_status.get())
				std::fill(begin(vel_wheel_corrected), end(vel_wheel_corrected),
						  0);

			hal_wheel_vel_set_pwm(vel_wheel_corrected);

			// Odometry func
			VelRF vel_rframe_matrix
				= controller.vWheel2vRF(VelWheel(encoder_deltas.data()));
			auto oldPose = this->pose_current.get();
			auto newPose = controller.odometry(oldPose, vel_rframe_matrix);
			pose_current.set(newPose);
			dPose<T> vel_rframe{vel_rframe_matrix(0), vel_rframe_matrix(1),
								vel_rframe_matrix(2)};

			// TODO: instead of dividing the omega by 2, multiply it with
			// the sampling frequency to get theta anle
			dPose<T> delta_wframe
				= dRF2dWF<T>(vel_rframe,
							 this->pose_current.get().theta
								 + vel_rframe.d_theta / static_cast<T>(2.0));
			T vx = delta_wframe.dx / sampling_times.FzDreh;
			T vy = delta_wframe.dy / sampling_times.FzDreh;
			T omega = delta_wframe.d_theta / sampling_times.FzDreh;
			pose_current.set(Pose<T>{newPose.x, newPose.y, newPose.theta,
									 vPose<T>{vx, vy, omega}});

			/* ++counter % n is undefined? */
			++counter;
			counter = counter % sampling_times.FzLageZuDreh;
			if (!counter)
				pose_ctrl_sem.signal();

			WheelControllerTimer.wait();
		}
	}
};

}
