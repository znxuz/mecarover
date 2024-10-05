#pragma once

#include <cmath>
#include <limits>

namespace imsl
{

constexpr double inf = std::numeric_limits<double>::infinity();
constexpr double eps = std::numeric_limits<double>::epsilon();

template<typename T> class dPose {
public:
	T dx{};
	T dy{};
	T d_theta{};

	dPose() = default;

	dPose(T dx, T dy, T d_theta)
		: dx{dx}
		, dy{dy}
		, d_theta{d_theta}
	{ }

	bool operator==(const dPose<T>& rhs) const
	{
		return this->dx == rhs.dx && this->dy == rhs.dy && this->d_theta ==
			rhs.d_theta;
	}
};

/* velocity (vx = dx/dt, vy = dy/dt, omega = dtheta/dt) of a mobile robot */
template<typename T> class vPose {
public:
	T vx{};
	T vy{};
	T omega{};

	vPose() = default;

	vPose(T vx, T vy, T omega)
		: vx{vx}
		, vy{vy}
		, omega{omega}
	{ }

	bool operator==(const vPose<T>& rhs) const
	{
		return this->vx == rhs.vx && this->vy == rhs.vy
			&& this->omega == rhs.omega;
	}
};

/* transformation of small movements from robot frame into world frame  */
template<typename T> dPose<T> dRF2dWF(dPose<T> dRF, T theta)
{
	dPose<T> dWF;
	dWF.dx = dRF.dx * cos(theta) - dRF.dy * sin(theta);
	dWF.dy = dRF.dx * sin(theta) + dRF.dy * cos(theta);
	dWF.d_theta = dRF.d_theta;
	return dWF;
}

/* transformation of small movements from world frame into robot frame  */
template<typename T> dPose<T> dWF2dRF(dPose<T> dWF, T theta)
{
	dPose<T> dRF;
	dRF.dx = dWF.dx * cos(theta) + dWF.dy * sin(theta);
	dRF.dy = -dWF.dx * sin(theta) + dWF.dy * cos(theta);
	dRF.d_theta = dWF.d_theta;
	return dRF;
}

/* transformation of velocities from robot frame into world frame  */
template<typename T> vPose<T> vRF2vWF(vPose<T> vRF, T theta)
{
	vPose<T> vWF;
	vWF.vx = vRF.vx * cos(theta) - vRF.vy * sin(theta);
	vWF.vy = vRF.vx * sin(theta) + vRF.vy * cos(theta);
	vWF.omega = vRF.omega;
	return vWF;
}

/* transformation of velocities from world frame into robot frame  */
template<typename T> vPose<T> vWF2vRF(vPose<T> vWF, T theta)
{
	vPose<T> vRF;
	vRF.vx = vWF.vx * cos(theta) + vWF.vy * sin(theta);
	vRF.vy = -vWF.vx * sin(theta) + vWF.vy * cos(theta);
	vRF.omega = vWF.omega;
	return vRF;
}

/* Heading of a mobile robot in the range of -pi ... +pi */
template<typename T> class Heading {
private:
	T th{};

	T maxPI(T theta) const
	{
		while (theta > T(M_PI))
			theta -= T(2 * M_PI);
		while (theta < T(-M_PI))
			theta += T(2 * M_PI);
		return theta;
	}

public:
	Heading() = default;

	Heading(T theta) { th = maxPI(theta); }

	Heading& operator=(const T& theta)
	{
		th = maxPI(theta);
		return *this;
	}

	Heading operator+(const T& d) const
	{
		Heading t = maxPI(th + d);
		return t;
	}

	Heading& operator+=(const T& d)
	{
		th = maxPI(th + d);
		return *this;
	}

	Heading operator-(const T& d) const
	{
		Heading t = maxPI(th - d);
		return t;
	}

	Heading& operator-=(const T& d)
	{
		th = maxPI(th - d);
		return *this;
	}

	operator T() const { return th; }
};

/* Pose (x, y, theta) and velocity (vx, vy, omega) of a mobile robot, theta is
 * in the range of -pi ... +pi */
template<typename T> class Pose {
public:
	T x{};
	T y{};
	Heading<T> theta{};
	vPose<T> velocity{};

	Pose() = default;

	Pose(T x, T y, Heading<T> theta, vPose<T> velocity)
		: x{x}
		, y{y}
		, theta{theta}
		, velocity{velocity}
	{ }

	Pose(T x, T y, Heading<T> theta, T vx, T vy, T omega)
		: Pose(x, y, theta, vPose<T>{vx, vy, omega})
	{ }

	bool operator==(const Pose<T>& rhs) const
	{
		return this->x == rhs.x && this->y == rhs.y && this->theta == rhs.theta
			&& this->velocity == rhs.velocity;
	}
};

template<typename T> dPose<T> operator-(const Pose<T>& p1, const Pose<T>& p2)
{
	dPose<T> p;
	p.dx = p1.x - p2.x;
	p.dy = p1.y - p2.y;
	p.d_theta = p1.theta - p2.theta;
	return p;
}

template<typename T>
dPose<T> operator/(const dPose<T>& p1, const unsigned int divisor)
{
	dPose<T> p;
	p.dx = p1.dx / divisor;
	p.dy = p1.dy / divisor;
	p.d_theta = p1.d_theta / divisor;
	return p;
}

template<typename T> Pose<T> operator+(const Pose<T>& p, const dPose<T>& d)
{
	Pose<T> n;
	n.x = p.x + d.dx;
	n.y = p.y + d.dy;
	n.theta = p.theta + d.d_theta; // MAXPI(p.theta + d.theta);
	return n;
}

template<typename T> Pose<T> operator-(const Pose<T>& p, const dPose<T>& d)
{
	Pose<T> n;
	n.x = p.x - d.dx;
	n.y = p.y - d.dy;
	n.theta = p.theta - d.d_theta; // MAXPI(p.theta - d.theta);
	return n;
}

}
