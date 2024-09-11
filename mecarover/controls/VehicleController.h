#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <mecarover/mrcpptypes.h>

namespace imsl::vehiclecontrol
{

/*-------         Beschreibung der Reglerparameter                -----*/
struct ReglerParam_t {
	double DzrKv;
	double DzrTaDTn;
	double DzrTvDTa;
	double DzrIntMax;
	double DzrSchleppMax;
	double DzrSkalierung[4];
	double DzrKoppel[4];
	Pose_t LageKv;
	Pose_t LageSchleppMax;
	double Koppel; /* Faktor für Ausregelung des Verkopplungsfehlers */
};

/*-------             Beschreibung der Zeitgroessen               -----*/
struct Abtastzeit_t {
	double Timer;
	double FzDreh;
	double FzLage;
	uint32_t FzLageZuDreh;
};

template <typename T, unsigned int N_WHEEL, unsigned int DOF>
class VehicleController {
public:
	using VelWheel = Eigen::Matrix<T, N_WHEEL, 1>;
	using VelRF = Eigen::Matrix<T, DOF, 1>;
	using Jacobian = Eigen::Matrix<T, N_WHEEL, DOF>;
	using InvJacobian = Eigen::Matrix<T, DOF, N_WHEEL>;

protected:
	InvJacobian inv_j = InvJacobian::Zero();
	Jacobian j = Jacobian::Zero();

public:
	// for data types Jacobi InverseJacobi
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // eigenlib 16 Byte alignement

	VehicleController() = default;

	// forward kinematic from wheel velocity to robot velocity
	VelRF vWheel2vRF(const VelWheel &u) const
	{
		return inv_j * u;
	}

	// backward kinematic from robot velocity to wheel velocity
	VelWheel vRF2vWheel(const VelRF &v) const
	{
		return j * v;
	}

	virtual imsl::Pose<T> odometry(const Pose<T>&, const VelWheel &) = 0;
	virtual VelRF poseControl(const Pose<T>& reference, const Pose<T>& actual) = 0;
	virtual VelWheel wheelControl(const VelWheel &setPoint, const VelWheel &contrInput) = 0;
	virtual imsl::vPose<T> velocityFilter(const vPose<T>& vel_rframe_sp, const vPose<T>& vel_rframe_old) = 0;
	virtual void poseUpdate(dPose<T> delta, unsigned int divisor) = 0;
	virtual void headingUpdate(T delta, unsigned int divisor) = 0;
};

}
