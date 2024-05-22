#pragma once

#include <Eigen/Eigen/Core>
#include <Eigen/Eigen/LU>
#include <mrcpptypes.h>

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

// Base class for all vehicle controllers
template <typename T, unsigned int NumbWheels, unsigned int DegFreed>
class VehicleController {
public:
	using WheelVel = Eigen::Matrix<T, NumbWheels, 1>;
	using VehicleVel = Eigen::Matrix<T, DegFreed, 1>;
	using Jacobi = Eigen::Matrix<T, NumbWheels, DegFreed>;
	using InverseJacobi = Eigen::Matrix<T, DegFreed, NumbWheels>;

protected:
	InverseJacobi iJ;
	Jacobi J;

public:
	// for data types Jacobi InverseJacobi
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // eigenlib 16 Byte alignement

		VehicleVel
		vWheel2vRF(const WheelVel &u) const
	{
		VehicleVel vRF;
		vRF = iJ * u;
		return vRF;
	}

	WheelVel vRF2vWheel(const VehicleVel &v) const
	{
		WheelVel u;
		u = J * v;
		return u;
	}

	VehicleController()
		: iJ(InverseJacobi::Zero())
		, J(Jacobi::Zero())
	{
	}

	virtual imsl::Pose<T> odometry(Pose<T>, const WheelVel &) = 0;
	virtual VehicleVel poseControl(PoseV<T> reference, PoseV<T> actual) = 0;
	virtual WheelVel wheelControl(const WheelVel &setPoint, const WheelVel &contrInput) = 0;
	virtual imsl::vPose<T> velocityFilter(vPose<T> vRFref, vPose<T> vRFold) = 0;
	virtual void poseUpdate(dPose<T> delta, unsigned int divisor) = 0;
	virtual void headingUpdate(T delta, unsigned int divisor) = 0;
};

}
