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
	static constexpr uint8_t N_WHEEL = 4;
	static constexpr uint8_t DOF = 4;

	using VelWheel = Eigen::Matrix<T, N_WHEEL, 1>;
	using VelRF = Eigen::Matrix<T, DOF, 1>;
	using Jacobian = Eigen::Matrix<T, N_WHEEL, DOF>;
	using InvJacobian = Eigen::Matrix<T, DOF, N_WHEEL>;

private:
	T epsilon1 = 0; // Verkopplungsfehler
	T LplusBhalbe; /* in mm 0.5 * (Breite + Laenge)   */
	T Radradius; /* in mm                           */
	T JoystickBeschl; /* maximale Jostickbeschleunigung in mm/s/s */
	Abtastzeit_t Ta; /* Abtastzeiten der Regler */
	ReglerParam_t Regler;
	Jacobian j = Jacobian::Zero();
	InvJacobian inv_j = InvJacobian::Zero();
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

	void init(const Fahrzeug_t* Fz, ReglerParam_t Regler, Abtastzeit_t Ta)
	{
		LplusBhalbe = Fz->LplusBhalbe;
		Radradius = Fz->Radradius;

		JoystickBeschl = Fz->JoystickBeschl;
		this->Regler = Regler;
		this->Ta = Ta;

		this->j << 1.0, 1.0, LplusBhalbe, 1.0, 1.0, -1.0, -LplusBhalbe, 1.0,
			1.0, 1.0, -LplusBhalbe, -1.0, 1.0, -1.0, LplusBhalbe, -1.0;
		this->j /= Radradius;

		this->inv_j << 1.0, 1.0, 1.0, 1.0, 1.0, -1.0, 1.0, -1.0,
			1.0 / LplusBhalbe, -1.0 / LplusBhalbe, -1.0 / LplusBhalbe,
			1.0 / LplusBhalbe, 4.0 / Radradius, 4.0 / Radradius,
			-4.0 / Radradius, -4.0 / Radradius;
		this->inv_j *= Radradius / 4.0;
	}

	Pose<T> odometry(const Pose<T>& oldPose,
					 const VelWheel& rad_delta_phi_matrix)
	{
		VelRF vel_rframe = this->vWheel2vRF(rad_delta_phi_matrix);

		dPose<T> delta_pose_rframe;
		delta_pose_rframe.x = vel_rframe(0); // movement in x direction
		delta_pose_rframe.y = vel_rframe(1); // movement in y direction
		delta_pose_rframe.theta
			= vel_rframe(2); // rotation around z axis (theta)
		epsilon1 += vel_rframe(3); // coupling error (delta)

		dPose<T> delta_pose_wframe = dRF2dWF<T>(
			delta_pose_rframe,
			oldPose.theta + delta_pose_rframe.theta / static_cast<T>(2));

		auto newPose = oldPose + delta_pose_wframe;

		return newPose;
	}

	VelRF poseControl(const Pose<T>& pose_sp, const Pose<T>& actual_pose)
	{
		dPose<T> pose_delta{pose_sp.x - actual_pose.x,
							pose_sp.y - actual_pose.y,
							Heading<T>(pose_sp.theta - actual_pose.theta)};

		using std::abs;
		/* Uberpruefen des Schleppfehlers */
		if (abs(pose_delta.x) > Regler.LageSchleppMax.x
			|| abs(pose_delta.y) > Regler.LageSchleppMax.y
			|| abs(pose_delta.theta) > Regler.LageSchleppMax.theta) {
			log_message(
				log_error,
				"%s, %s, deviation position controller too large: act.x: %f, "
				"ref.x: %f, act.y: %f, ref.y: %f, act.theta: %f, ref.theta: %f",
				__FILE__, __FUNCTION__, actual_pose.x, pose_sp.x, actual_pose.y,
				pose_sp.y, T(actual_pose.theta), T(pose_sp.theta));
			log_message(log_error,
						"Max. Schleppabstand: %f %f %f Schleppabstand %f %f %f",
						Regler.LageSchleppMax.x, Regler.LageSchleppMax.y,
						Regler.LageSchleppMax.theta, pose_delta.x, pose_delta.y,
						pose_delta.theta);

			throw MRC_LAGEERR;
		}

		vPose<T> vel_wframe_sp;
		vel_wframe_sp.vx = pose_sp.vx + pose_delta.x * Regler.LageKv.x;
		vel_wframe_sp.vy = pose_sp.vy + pose_delta.y * Regler.LageKv.y;
		vel_wframe_sp.omega
			= pose_sp.omega + pose_delta.theta * Regler.LageKv.theta;

		vPose<T> vel_rframe_sp = vWF2vRF<T>(vel_wframe_sp, actual_pose.theta);
		VelRF v;
		v(0) = vel_rframe_sp.vx;
		v(1) = vel_rframe_sp.vy;
		v(2) = vel_rframe_sp.omega;
		v(3) = Regler.Koppel
			* epsilon1; // ASK: Koppel is the proportional scaler from PID?
		return v;
	}

	VelWheel wheelControl(const VelWheel& refVel, const VelWheel& realVel)
	{
		T regelAbw, diff;
		VelWheel StellV;

		for (int i = 0; i < N_WHEEL; i++) {
			// Regelabweichung, Sollwert und Istwert in rad/s
			regelAbw = refVel(i) - realVel(i);

			// I-Anteil
			Integr(i) += Regler.DzrTaDTn * regelAbw;
			if (Integr(i) > Regler.DzrIntMax) {
				Integr(i) = Regler.DzrIntMax;
			} else if (Integr(i) < -(Regler.DzrIntMax)) {
				Integr(i) = -(Regler.DzrIntMax);
			}
			// printf("I-Anteil für den %d. Motor: %f\n", i, Integr(i));

			// D-Anteil
			diff = Regler.DzrTvDTa * (regelAbw - RAbwAlt(i));

			// Stellgroesse in rad/s
			StellV(i) = Regler.DzrKv * (regelAbw + Integr(i) + diff);

			// Vorsteuerung addieren
			StellV(i) += refVel(i);

			// Anpassung an individuellen Verstaerker
			StellV(i) *= Regler.DzrSkalierung[i];

			RAbwAlt(i) = regelAbw;
		}
		return StellV;
	}

	vPose<T> velocityFilter(const vPose<T>& vel_rframe_sp,
							const vPose<T>& vel_rframe_old)
	{
		vPose<T> vel_rframe = vel_rframe_sp;

		T v_diff = JoystickBeschl * Ta.FzLage;

		if (vel_rframe.vx - vel_rframe_old.vx > v_diff)
			vel_rframe.vx = vel_rframe_old.vx + v_diff;
		else if (vel_rframe_old.vx - vel_rframe.vx > v_diff)
			vel_rframe.vx = vel_rframe_old.vx - v_diff;

		if (vel_rframe.vy - vel_rframe_old.vy > v_diff)
			vel_rframe.vy = vel_rframe_old.vy + v_diff;
		else if (vel_rframe_old.vy - vel_rframe.vy > v_diff)
			vel_rframe.vy = vel_rframe_old.vy - v_diff;

		v_diff /= LplusBhalbe;

		if (vel_rframe.omega - vel_rframe_old.omega > v_diff)
			vel_rframe.omega = vel_rframe_old.omega + v_diff;
		else if (vel_rframe_old.omega - vel_rframe.omega > v_diff)
			vel_rframe.omega = vel_rframe_old.omega - v_diff;

		return vel_rframe;
	}
};

}
