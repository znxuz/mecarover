#pragma once

/* C++ header files */
#include <exception> // for std::exception
#include <string>    // for std::string
#include <limits>    // for std::numeric_limits

/* local header file */
#include <mrtypes.h> // datatypes for C

#define PI_L  3.141592653589793238462643383279502884L

namespace imsl
{
// error: variable templates only available with -std=c++14 or -std=gnu++14 [-Werror]
//template<class T>
//constexpr T PI_T = T(3.14159265358979323846L);  // PI template
//constexpr real_t eps = DBL_EPSILON;
//constexpr real_t inf = INFINITY;
constexpr double inf = std::numeric_limits<double>::infinity();
constexpr double eps = std::numeric_limits<double>::epsilon();

template<class T>
int signum(T t)
{
	if(t < T(0)) {
		return -1;
	}
	return 1;
}

/* movement (dx, dy, dtheta) of a mobile robot in 2D */
template<typename T>
class dPose
{
public:
	T x;
	T y;
	T theta;

	dPose() {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
	}

	dPose(const dPose &p) {
		x = p.x;
		y = p.y;
		theta = p.theta;
	}

	dPose& operator=(const dPose &p) {
		x = p.x;
		y = p.y;
		theta = p.theta;
		return *this;
	}

	dPose(Pose_t p) {
		x = p.x;
		y = p.y;
		theta = p.theta;
	}

	operator Pose_t() const {
		Pose_t p;
		p.x = this->x;
		p.y = this->y;
		p.theta = this->theta;
		return p;
	}
};

/* velocity (vx = dx/dt, vy = dy/dt, omega = dtheta/dt) of a mobile robot in 2D */
template<typename T>
class vPose
{
public:
	T vx;
	T vy;
	T omega;

	vPose() {
		vx = 0.0;
		vy = 0.0;
		omega = 0.0;
	}

	vPose(const vPose &vp) {
		vx = vp.vx;
		vy = vp.vy;
		omega = vp.omega;
	}

	vPose& operator=(const vPose &vp) {
		vx = vp.vx;
		vy = vp.vy;
		omega = vp.omega;
		return *this;
	}

	// compatibility to old C code
	operator Pose_t() const {
		Pose_t p;
		p.x = this->vx;
		p.y = this->vy;
		p.theta = this->omega;
		return p;
	}

	// compatibility to old C code
	vPose& operator=(const Pose_t &p) {
		vx = p.x;
		vy = p.y;
		omega = p.theta;
		return *this;
	}
};

/* transformation of small movements from robot frame into world frame  */
template<typename T>
dPose<T> dRF2dWF(dPose<T> dRF, T theta)
{
	dPose<T> dWF;
	dWF.x = dRF.x * cos(theta) - dRF.y * sin(theta);
	dWF.y = dRF.x * sin(theta) + dRF.y * cos(theta);
	dWF.theta = dRF.theta;
	return dWF;
}

/* transformation of small movements from world frame into robot frame  */
template<typename T>
dPose<T> dWF2dRF(dPose<T> dWF, T theta) {
	dPose<T> dRF;
	dRF.x = dWF.x * cos(theta) + dWF.y * sin(theta);
	dRF.y = -dWF.x * sin(theta) + dWF.y * cos(theta);
	dRF.theta = dWF.theta;
	return dRF;
}

/* transformation of velocities from robot frame into world frame  */
template<typename T>
vPose<T> vRF2vWF(vPose<T> vRF, T theta) {
	vPose<T> vWF;
	vWF.vx = vRF.vx * cos(theta) - vRF.vy * sin(theta);
	vWF.vy = vRF.vx * sin(theta) + vRF.vy * cos(theta);
	vWF.omega = vRF.omega;
	return vWF;
}

/* transformation of velocities from world frame into robot frame  */
template<typename T>
vPose<T> vWF2vRF(vPose<T> vWF, T theta) {
	vPose<T> vRF;
	vRF.vx = vWF.vx * cos(theta) + vWF.vy * sin(theta);
	vRF.vy = -vWF.vx * sin(theta) + vWF.vy * cos(theta);
	vRF.omega = vWF.omega;
	return vRF;
}

/* Heading of a mobile robot in the range of -pi ... +pi */
template<typename T>
class Heading {
private:
	T th;

	T maxPI(T theta) const {
		while(theta > T(PI_L)) theta -= T(2*PI_L);
		while(theta < T(-PI_L)) theta += T(2*PI_L);
		return theta;
	}

public:
	Heading() {
		th = 0.0;
	}

	Heading(T theta) {
		th = maxPI(theta);
	}

	Heading& operator=(const T &theta) {
		th = maxPI(theta);
		return *this;
	}

	Heading operator+(const T &d) const {
		Heading t = maxPI(th + d);
		return t;
	}

	Heading& operator+=(const T &d) {
		th = maxPI(th + d);
		return *this;
	}

	Heading operator-(const T &d) const {
		Heading t = maxPI(th - d);
		return t;
	}

	Heading& operator-=(const T &d) {
		th = maxPI(th - d);
		return *this;
	}

	operator T() const {
		return th;
	}
};

/* Pose (x, y, theta) of a mobile robot, theta is in the range of -pi ... +pi */
template<typename T>
class Pose {
public:
	T x;
	T y;
	Heading<T> theta;

	Pose() {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
	}

	Pose(const Pose<T> &p) {
		x = p.x;
		y = p.y;
		theta = p.theta; 
	}

	Pose& operator=(const Pose<T> &p) {
		x = p.x;
		y = p.y;
		theta = p.theta; 
		return *this;
	}

	Pose(Pose_t p) {
		x = p.x;
		y = p.y;
		theta = MAXPI(p.theta);
	}

	Pose(PoseV_t p) {
		x = p.x;
		y = p.y;
		theta = MAXPI(p.theta);
	}

	operator Pose_t() const {
		Pose_t p;
		p.x = this->x;
		p.y = this->y;
		p.theta = this->theta; 
		return p;
	}
};

/* Pose (x, y, theta) and velocity (vx, vy, omega) of a mobile robot, theta is in the range of -pi ... +pi */
template<typename T>
class PoseV {
public:
	T x;
	T y;
	Heading<T> theta;
	T vx;
	T vy;
	T omega;

	PoseV() {
		x = 0.0; y = 0.0; theta = 0.0;
		vx = 0.0; vy = 0.0; omega = 0.0;
	}

	PoseV(const PoseV<T> &p) {
		x = p.x; y = p.y;
		theta = p.theta; 
		vx = p.vx; vy = p.vy; omega = p.omega;
	}

	PoseV& operator=(const PoseV<T> &p) {
		x = p.x; y = p.y;
		theta = p.theta; 
		vx = p.vx; vy = p.vy; omega = p.omega;
		return *this;
	}

	PoseV(const PoseV_t &p) {
		x = p.x;
		y = p.y;
		theta = p.theta; 
		vx = p.vx; vy = p.vy; omega = p.omega;
	}

	operator PoseV_t() const {
		PoseV_t p;
		p.x = this->x;
		p.y = this->y;
		p.theta = MAXPI(this->theta);
		p.vx = this->vx; p.vy = this->vy; p.omega = this->omega;
		return p;
	}

	bool operator== (PoseV<T> &rhs){
		bool ret = false;
		if((this->x == rhs.x) && (this->y == rhs.y) && (this->theta == rhs.theta)) // TODO  eps
			ret = true;
		return ret;
	}
};

template<typename T>
dPose<T> operator-(const Pose<T> &p1, const Pose<T> &p2) {
	dPose<T> p;
	p.x = p1.x - p2.x;
	p.y = p1.y - p2.y;
	p.theta = p1.theta - p2.theta;
	return p;
}

template<typename T>
dPose<T> operator/(const dPose<T> &p1, const unsigned int divisor) {
	dPose<T> p;
	p.x = p1.x / divisor;
	p.y = p1.y / divisor;
	p.theta = p1.theta / divisor;
	return p;
}

template<typename T>
Pose<T> operator+(const Pose<T> &p, const dPose<T> &d) {
	Pose<T> n;
	n.x = p.x + d.x;
	n.y = p.y + d.y;
	n.theta = p.theta + d.theta; //MAXPI(p.theta + d.theta);
	return n;
}

template<typename T>
Pose<T> operator-(const Pose<T> &p, const dPose<T> &d) {
	Pose<T> n;
	n.x = p.x - d.x;
	n.y = p.y - d.y;
	n.theta = p.theta - d.theta; //MAXPI(p.theta - d.theta);
	return n;
}

class IMSLex: public std::exception {
private:
	std::string s;
	int err;    

public:
	IMSLex() {}

	IMSLex(int err,std::string s) {this->s = s; this->err = err;}

	virtual ~IMSLex() throw() {}

	virtual const char * what() const throw()
	{return s.c_str();}

	int geterrno()
	{return err;}
};


}
