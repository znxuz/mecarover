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

	MecanumController<T> controller{};

	RtosTask lageregler_thread{call_pose_control_task, "PoseControllerTask",
							   MAIN_TASK_STACK_SIZE, this,
							   POSE_CONTROLLER_PRIORITY};
	RtosTask drehzahlregler_thread{call_wheel_control_task,
								   "WheelControllerTask", MAIN_TASK_STACK_SIZE,
								   this, WHEEL_CONTROLLER_PRIORITY};
	RtosSemaphore pose_ctrl_sem{10, 0}; // TODO: why even 10 to begin with

	RtosMutexObject<std::array<T, N_WHEEL>> vel_wheel_sp;
	RtosMutexObject<mrc_stat> error_status{MRC_NOERR};

public:
	RtosMutexObject<CtrlMode> ctrl_mode;
	RtosMutexObject<Pose<T>> pose_current;
	RtosMutexObject<vPose<T>> ref_vel_manual;

	void PoseControlTask()
	{
		vPose<T> vel_rframe_sp;
		vPose<T> vel_rframe_prev;
		vPose<T> vel_wframe_sp;
		Pose<T> pose_sp;
		Pose<T> pose_cur;
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
			pose_cur = this->pose_current.get();

			if (controllerMode == CtrlMode::OFF) {
				pose_sp = pose_manual = pose_cur;
				pose_sp.velocity = vel_rframe_prev = vPose<T>{};
				continue;
			}

			/* First time after mode switch */
			if (oldmode != CtrlMode::TWIST)
				pose_manual = pose_cur;

			vel_rframe_sp = ref_vel_manual.get();

			// set pose_manual from velocity setpoint
			vel_rframe_sp
				= controller.velocityFilter(vel_rframe_sp, vel_rframe_prev);
			vPose<T> vel_wframe_sp = vRF2vWF<T>(vel_rframe_sp, pose_cur.theta);
			pose_manual.x += vel_wframe_sp.vx * sampling_times.dt_pose_ctrl;
			pose_manual.y += vel_wframe_sp.vy * sampling_times.dt_pose_ctrl;
			pose_manual.theta += vel_wframe_sp.omega *
				sampling_times.dt_pose_ctrl;

			static int count = 0;
			if (count++ > 100) {
				count = 0;
				log_message(log_info,
							"manual pose via velocity x: %f, y: %f, theta: %f",
							pose_manual.x, pose_manual.y, T(pose_manual.theta));
			}
			vel_rframe_prev = vel_rframe_sp;

			/* V von RKS -> WKS transformieren */
			vel_wframe_sp = vRF2vWF<T>(vel_rframe_sp, pose_cur.theta);

			pose_sp.x = pose_manual.x;
			pose_sp.y = pose_manual.y;
			pose_sp.theta = pose_manual.theta;
			pose_sp.velocity = vel_wframe_sp;

			try {
				// calculate reference velocities of the wheels
				VelWheel vel_wheel_sp_mtx = controller.vRF2vWheel(
					controller.poseControl(pose_sp, pose_cur));
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
		std::array<T, N_WHEEL> encoders_delta_rad{};
		std::array<T, N_WHEEL> vel_wheel_sp{};
		std::array<T, N_WHEEL> vel_wheel_actual{};
		std::array<T, N_WHEEL> vel_wheel_corrected{};

		CtrlMode controllerMode;

		int counter = sampling_times.ratio_pose_wheel;

		size_t free_heap = xPortGetMinimumEverFreeHeapSize();
		uint32_t free_stack = lageregler_thread.getStackHighWaterMark();
		log_message(log_info,
					"wheel controller initialized running into control loop, "
					"free heap: %d, free stack: %ld",
					free_heap, free_stack);

		RtosPeriodicTimer WheelControllerTimer(sampling_times.dt_wheel_ctrl
											   * MS_TO_S);
		while (true) {
			controllerMode = ctrl_mode.get();

			switch (controllerMode) {
			case CtrlMode::OFF:
				std::fill(begin(vel_wheel_sp), end(vel_wheel_sp), 0);
				std::fill(begin(vel_wheel_corrected), end(vel_wheel_corrected),
						  0);
				break;
			case CtrlMode::TWIST:
				vel_wheel_sp = this->vel_wheel_sp.get();
				break;
			}

			encoders_delta_rad = hal_encoder_delta_rad();

			/*
			 * wheel PID control: values passed as wheel angular vel delta
			 */
			std::transform(
				begin(encoders_delta_rad), end(encoders_delta_rad),
				begin(vel_wheel_actual),
				[dt = sampling_times.dt_wheel_ctrl](T encoder_delta_rad) {
					return encoder_delta_rad / dt;
				});

			if (error_status.get()) {
				std::fill(begin(vel_wheel_corrected), end(vel_wheel_corrected),
						  0);
			} else if (controllerMode != CtrlMode::OFF) {
				VelWheel correcting_vel_matrix = controller.wheel_pid_control(
					VelWheel(vel_wheel_sp.data()),
					VelWheel(vel_wheel_actual.data()));
				std::copy(std::begin(correcting_vel_matrix),
						  std::end(correcting_vel_matrix),
						  begin(vel_wheel_corrected));
			}

			hal_wheel_vel_set_pwm(vel_wheel_corrected);

			/*
			 * odometry: encoder delta gets feeded directly into the inverted
			 * jacobian matrix without dividing the dt for the reason being:
			 * d_encoder / dt = wheel_velin rad X inv_j = robot_vel * dt = dpose
			 * => dt can be spared because its unnecessary calculation
			 */
			VelRF dpose_rframe_matrix
				= controller.vWheel2vRF(VelWheel(encoders_delta_rad.data()));
			dPose<T> dpose_rframe{dpose_rframe_matrix(0),
								  dpose_rframe_matrix(1),
								  dpose_rframe_matrix(2)};
			controller.update_epsilon(dpose_rframe_matrix(3));

			auto oldPose = this->pose_current.get();
			dPose<T> dpose_wframe = dRF2dWF<T>(
				dpose_rframe,
				oldPose.theta + dpose_rframe.d_theta / static_cast<T>(2));
			auto newPose = oldPose + dpose_wframe;
			newPose.velocity = dpose_wframe
				/ sampling_times
					  .dt_wheel_ctrl; // TODO: check why velocity is set here as
									  // well, its supposed to only care about
									  // the odometry here
			pose_current.set(newPose);

			/* ++counter % n is undefined? */
			++counter;
			counter = counter % sampling_times.ratio_pose_wheel;
			if (!counter)
				pose_ctrl_sem.signal();

			WheelControllerTimer.wait();
		}
	}
};

}
