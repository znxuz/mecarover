#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>
#include <mecarover/mrcpptypes.h>
#include <mecarover/mrlogger/mrlogger.h>
#include <mecarover/robot_params.hpp>

namespace imsl::vehiclecontrol
{

template<typename T>
class MecanumController {
public:
	static constexpr uint8_t DOF = 4;

	using VelWheel = Eigen::Matrix<T, N_WHEEL, 1>;
	using VelRF = Eigen::Matrix<T, DOF, 1>;
	using Jacobian = Eigen::Matrix<T, N_WHEEL, DOF>;
	using InvJacobian = Eigen::Matrix<T, DOF, N_WHEEL>;

private:
	T epsilon1 = 0; // Verkopplungsfehler
	T l_b_half; /* in mm 0.5 * (Breite + Laenge)   */
	T radius_wheel; /* in mm                           */
	T JoystickBeschl; /* maximale Jostickbeschleunigung in mm/s/s */
	sampling_time_t sampling_times; /* Abtastzeiten der Regler */
	ctrl_param_t ctrl_params;
	Jacobian j;
	InvJacobian inv_j;
	VelWheel RAbwAlt = VelWheel::Zero();
	VelWheel Integr = VelWheel::Zero();

public:
	// for data types VelWheel
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // eigenlib 16 Byte alignement

	MecanumController()
		= default;

	// forward transformation wheel -> robot velocity
	VelRF vWheel2vRF(const VelWheel& u) const { return inv_j * u; }

	// backward transformation robot -> wheel velocity
	VelWheel vRF2vWheel(const VelRF& v) const { return j * v; }

	void init(const robot_param_t* robot_params, ctrl_param_t ctrl_params,
			  sampling_time_t sampling_times)
	{
		l_b_half = robot_params->LplusBhalbe;
		radius_wheel = robot_params->Radradius;

		JoystickBeschl = robot_params->JoystickBeschl;
		this->ctrl_params = ctrl_params;
		this->sampling_times = sampling_times;

		this->j = Jacobian{{1.0, 1.0, l_b_half, 1.0},
						   {1.0, -1.0, -l_b_half, 1.0},
						   {1.0, 1.0, -l_b_half, -1.0},
						   {1.0, -1.0, l_b_half, -1.0}};
		this->j /= radius_wheel;

		this->inv_j = InvJacobian{
			{1.0, 1.0, 1.0, 1.0},
			{1.0, -1.0, 1.0, -1.0},
			{1.0 / l_b_half, -1.0 / l_b_half, -1.0 / l_b_half, 1.0 / l_b_half},
			{4.0 / radius_wheel, 4.0 / radius_wheel, -4.0 / radius_wheel,
			 -4.0 / radius_wheel}};
		this->inv_j *= radius_wheel / 4.0;
	}

	Pose<T> odometry(const Pose<T>& oldPose, const VelRF& vel_rframe_matrix)
	{
		dPose<T> delta_pose_rframe{vel_rframe_matrix(0), vel_rframe_matrix(1),
								   vel_rframe_matrix(2)};
		epsilon1 += vel_rframe_matrix(3); // coupling error (delta)

		dPose<T> delta_pose_wframe = dRF2dWF<T>(
			delta_pose_rframe,
			oldPose.theta + delta_pose_rframe.theta / static_cast<T>(2));

		auto newPose = oldPose + delta_pose_wframe;

		return newPose;
	}

	VelRF poseControl(const Pose<T>& pose_sp, const Pose<T>& actual_pose) const
	{
		dPose<T> pose_delta{pose_sp.x - actual_pose.x,
							pose_sp.y - actual_pose.y,
							Heading<T>(pose_sp.theta - actual_pose.theta)};

		using std::abs;
		if (abs(pose_delta.x) > ctrl_params.LageSchleppMax.x
			|| abs(pose_delta.y) > ctrl_params.LageSchleppMax.y
			|| abs(pose_delta.theta) > ctrl_params.LageSchleppMax.theta)
			[[unlikely]] {
			log_message(
				log_error,
				"%s, %s, deviation position controller too large: act.x: %f, "
				"ref.x: %f, act.y: %f, ref.y: %f, act.theta: %f, ref.theta: %f",
				__FILE__, __FUNCTION__, actual_pose.x, pose_sp.x, actual_pose.y,
				pose_sp.y, T(actual_pose.theta), T(pose_sp.theta));
			throw MRC_LAGEERR;
		}

		vPose<T> vel_wframe_sp;
		vel_wframe_sp.vx = pose_sp.vx + pose_delta.x * ctrl_params.LageKv.x;
		vel_wframe_sp.vy = pose_sp.vy + pose_delta.y * ctrl_params.LageKv.y;
		vel_wframe_sp.omega
			= pose_sp.omega + pose_delta.theta * ctrl_params.LageKv.theta;

		vPose<T> vel_rframe_sp = vWF2vRF<T>(vel_wframe_sp, actual_pose.theta);

		return VelRF{vel_rframe_sp.vx, vel_rframe_sp.vy, vel_rframe_sp.omega,
					 ctrl_params.Koppel * epsilon1};
	}

	VelWheel wheelControl(const VelWheel& refVel, const VelWheel& realVel)
	{
		T regelAbw, diff;
		VelWheel StellV;

		for (int i = 0; i < N_WHEEL; i++) {
			// Regelabweichung, Sollwert und Istwert in rad/s
			regelAbw = refVel(i) - realVel(i);

			// I-Anteil
			Integr(i) += ctrl_params.DzrTaDTn * regelAbw;
			if (Integr(i) > ctrl_params.DzrIntMax) {
				Integr(i) = ctrl_params.DzrIntMax;
			} else if (Integr(i) < -(ctrl_params.DzrIntMax)) {
				Integr(i) = -(ctrl_params.DzrIntMax);
			}

			// D-Anteil
			diff = ctrl_params.DzrTvDTa * (regelAbw - RAbwAlt(i));

			// Stellgroesse in rad/s
			StellV(i) = ctrl_params.DzrKv * (regelAbw + Integr(i) + diff);

			// Vorsteuerung addieren
			StellV(i) += refVel(i);

			// Anpassung an individuellen Verstaerker
			StellV(i) *= ctrl_params.DzrSkalierung[i];

			RAbwAlt(i) = regelAbw;
		}
		return StellV;
	}

	vPose<T> velocityFilter(const vPose<T>& vel_rframe_sp,
							const vPose<T>& vel_rframe_old) const
	{
		vPose<T> vel_rframe = vel_rframe_sp;

		T v_diff = JoystickBeschl * sampling_times.FzLage;

		if (vel_rframe.vx - vel_rframe_old.vx > v_diff)
			vel_rframe.vx = vel_rframe_old.vx + v_diff;
		else if (vel_rframe_old.vx - vel_rframe.vx > v_diff)
			vel_rframe.vx = vel_rframe_old.vx - v_diff;

		if (vel_rframe.vy - vel_rframe_old.vy > v_diff)
			vel_rframe.vy = vel_rframe_old.vy + v_diff;
		else if (vel_rframe_old.vy - vel_rframe.vy > v_diff)
			vel_rframe.vy = vel_rframe_old.vy - v_diff;

		v_diff /= l_b_half;

		if (vel_rframe.omega - vel_rframe_old.omega > v_diff)
			vel_rframe.omega = vel_rframe_old.omega + v_diff;
		else if (vel_rframe_old.omega - vel_rframe.omega > v_diff)
			vel_rframe.omega = vel_rframe_old.omega - v_diff;

		return vel_rframe;
	}
};

}
