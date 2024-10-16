#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <mecarover/mrcpptypes.h>
#include <mecarover/mrlogger/mrlogger.h>
#include <mecarover/robot_params.hpp>

namespace imsl::vehiclecontrol
{

template<typename T> class MecanumController {
public:
	static constexpr uint8_t DOF = 4;

	using VelWheel = Eigen::Matrix<T, N_WHEEL, 1>;
	using VelRF = Eigen::Matrix<T, DOF, 1>;
	using Robot2WheelMatrix = Eigen::Matrix<T, N_WHEEL, DOF>;
	using Wheel2RobotMatrix = Eigen::Matrix<T, DOF, N_WHEEL>;

private:
	Robot2WheelMatrix bt_matrix;
	Wheel2RobotMatrix ft_matrix;
	T epsilon1 = 0; // Verkopplungsfehler // TODO: what is this
	VelWheel prev_errors = VelWheel::Zero();
	VelWheel cumulated_integral = VelWheel::Zero();

public:
	// for data types VelWheel
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // eigenlib 16 Byte alignement

	MecanumController()
	{
		this->bt_matrix
			= Robot2WheelMatrix{{1.0, 1.0, robot_params.l_w_half, 1.0},
								{1.0, -1.0, -robot_params.l_w_half, 1.0},
								{1.0, 1.0, -robot_params.l_w_half, -1.0},
								{1.0, -1.0, robot_params.l_w_half, -1.0}};
		this->bt_matrix /= robot_params.wheel_radius;

		this->ft_matrix = Wheel2RobotMatrix{
			{1.0, 1.0, 1.0, 1.0},
			{1.0, -1.0, 1.0, -1.0},
			{1.0 / robot_params.l_w_half, -1.0 / robot_params.l_w_half,
			 -1.0 / robot_params.l_w_half, 1.0 / robot_params.l_w_half},
			{4.0 / robot_params.wheel_radius, 4.0 / robot_params.wheel_radius,
			 -4.0 / robot_params.wheel_radius,
			 -4.0 / robot_params.wheel_radius}};
		this->ft_matrix *= robot_params.wheel_radius / 4.0;
	}

	VelRF vWheel2vRF(const VelWheel& u) const { return ft_matrix * u; }

	VelWheel vRF2vWheel(const VelRF& v) const { return bt_matrix * v; }

	/* no idea what this epsilon is */
	void update_epsilon(T epsilon1) { this->epsilon1 = epsilon1; }

	VelRF pose_control_get_vel_rframe(const dPose<T>& pose_delta,
					  const Heading<T>& pose_cur_theta,
					  const vPose<T>& vel_wframe_cur) const
	{
		vPose<T> vel_diff = {
			pose_delta.dx / sampling_times.dt_pose_ctrl * ctrl_params.LageKv.x,
			pose_delta.dy / sampling_times.dt_pose_ctrl * ctrl_params.LageKv.y,
			pose_delta.d_theta / sampling_times.dt_pose_ctrl
				* ctrl_params.LageKv.theta};

		vPose<T> vel_rframe_sp
			= vWF2vRF<T>(vel_wframe_cur + vel_diff, pose_cur_theta);

		return VelRF{vel_rframe_sp.vx, vel_rframe_sp.vy, vel_rframe_sp.omega,
					 ctrl_params.Koppel * epsilon1};
	}

	VelWheel wheel_pid_control(const VelWheel& vel_wheel_sp,
							   const VelWheel& vel_wheel_actual)
	{
		VelWheel ctrl_output = vel_wheel_sp;

		for (int i = 0; i < N_WHEEL; ++i) {
			T error = vel_wheel_sp(i) - vel_wheel_actual(i);

			// I-Anteil
			cumulated_integral(i) = std::clamp(
				cumulated_integral(i) + ctrl_params.k_i * error,
				-ctrl_params.integral_limit, ctrl_params.integral_limit);

			// D-Anteil
			T derivative_gain = ctrl_params.k_d * (error - prev_errors(i));

			// ASK: why downscale the sum of the PID errors
			ctrl_output(i) += ctrl_params.k_ctrl_output
				* (error + cumulated_integral(i) + derivative_gain);

			prev_errors(i) = error;
		}

		return ctrl_output;
	}

	vPose<T> velocityFilter(const vPose<T>& vel_rframe_sp,
							const vPose<T>& vel_rframe_old) const
	{
		vPose<T> vel_rframe = vel_rframe_sp;

		T v_diff = robot_params.JoystickBeschl * sampling_times.dt_pose_ctrl;

		if (vel_rframe.vx - vel_rframe_old.vx > v_diff)
			vel_rframe.vx = vel_rframe_old.vx + v_diff;
		else if (vel_rframe_old.vx - vel_rframe.vx > v_diff)
			vel_rframe.vx = vel_rframe_old.vx - v_diff;

		if (vel_rframe.vy - vel_rframe_old.vy > v_diff)
			vel_rframe.vy = vel_rframe_old.vy + v_diff;
		else if (vel_rframe_old.vy - vel_rframe.vy > v_diff)
			vel_rframe.vy = vel_rframe_old.vy - v_diff;

		v_diff /= robot_params.l_w_half;

		if (vel_rframe.omega - vel_rframe_old.omega > v_diff)
			vel_rframe.omega = vel_rframe_old.omega + v_diff;
		else if (vel_rframe_old.omega - vel_rframe.omega > v_diff)
			vel_rframe.omega = vel_rframe_old.omega - v_diff;

		return vel_rframe;
	}
};

}
