#include <cap_gait/contrib/Limp.h>
#include <cap_gait/contrib/Globals.h>
#include <nimbro_utils/math_funcs.h>
#include <math.h>
#include <QDebug>

using namespace nimbro_utils;

namespace margait_contrib
{

/*
This is a one dimensional linear inverted pendulum model.
The name LIMP is intended to be a pun. The state of the
limp consists of a gravitational constant C, the current
location x0 and the current velocity v0. These are public
variables, just set them as you like. This class offers
methods to:

- predict a future pendulum state given the current state
and a time t in the future relative to now using either
Euler integration simUpdate(t) or an analytic function
update(t).

- query the orbital energy(), which stays constant on an
undisturbed trajectory.

- find out at which point in time relative to the
current state the pendulum will reach a given location
tLoc(x), or a given velocity tVel(vx). Both of these
questions can have 0, 1 or 2 answers.

- find out at which location a certain velocity will be
reached x(vx) or what velocity the pendulum will have
at a given location vx(x). Both of these questions can
have 0, 1 or 2 answers.

- calculate a new pendulum origin() to reach a given
state (x,vx).
*/

Limp::Limp()
{
	C = 1;
	x0 = 0.0;
	v0 = 0.0;
}

Limp::Limp(double x, double v, double C)
{
	x0 = x;
	v0 = v;
	this->C = C;
}

// Resets the state to 0.
void Limp::reset()
{
	x0 = 0.0;
	v0 = 0.0;
}

// Sets the pendulum into a given state (x,v) leaving the gravitational constant C unchanged.
void Limp::set(double x, double v)
{
	x0 = x;
	v0 = v;
}

// Sets the pendulum into a given state (x,v) with a gravitational constant C.
void Limp::set(double x, double v, double C)
{
	x0 = x;
	v0 = v;
	this->C = C;
}

// Calculates a future state of the limp in t seconds from now using Euler
// integration of the differential equations of the Linear Inverted Pendulum
// Model. The future state will be written into the state variables x0 and vx0.
void Limp::simUpdate(double time)
{
	double timestep = 0.0001;
	double ax;
	double t = 0.0;
	while (t < time)
	{
		ax = C*x0;
		x0 += timestep*v0;
		v0 += timestep*ax;
		t += timestep;
	}
}

// Calculates a future state of the limp using the analytical solution
// of the Linear Inverted Pendulum Model. The given time parameter should
// be relative to now. The future state will be written into the state
// variables x0 and vx0. Negative times are allowed.
void Limp::update(double t)
{
	double sC = sqrt(C);
	double c1 = 0.5*(x0 + v0/sC);
	double c2 = 0.5*(x0 - v0/sC);
	x0 = c1 * exp(sC*t) + c2 * exp(-sC*t);
	v0 = c1 * sC * exp(sC*t) - c2 * sC * exp(-sC*t);
}

// Returns a predicted pendulum state at time t in the future.
Limp Limp::predict(double t)
{
	Limp l(x0, v0, C);
	l.update(t);
	return l;
}

// Returns the current orbital energy of the pendulum.
double Limp::energy()
{
	return 0.5*(v0*v0 - C*x0*x0);
}

// Returns the orbital energy for the provided pendulum parameters.
double Limp::energy(double x, double v, double C)
{
	return 0.5*(v*v - C*x*x);
}

// Given the current state of the limp and assuming t = 0, at what time t
// is the limp going to reach location x? Please note that there can be 0
// (the queried state is never reached), 1 (the queried state is reached
// exactly once), or 2 (the queried state is reached twice) solutions.
// Also, any number of the possible solutions can be negative, which means
// that the queried location has been reached once or twice in the past.
// In case there are no solutions, the function will return NaN.
// Otherwise the function returns either the one solution found, or the
// greater of two possible solutions, which can still be negative.
double Limp::tLoc(double x)
{
	// Note to self:
	// To obtain the smaller solution, -sqrt() has to be used.

	double sC = sqrt(C);
	double c1 = (x0 + v0/sC);
	double c2 = (x0 - v0/sC);
	double t = log(x/c1 + sqrt( (x*x)/(c1*c1) - c2/c1)) / sC;
	return t;
}


// Given the current state of the limp and assuming t = 0, at what time t
// is the limp going to reach velocity vx? Please note that there can be 0
// (the queried state is never reached), 1 (the queried state is reached
// exactly once), or 2 (the queried state is reached twice) solutions.
// Also, any number of the possible solutions can be negative, which means
// that the queried velocity has been reached once or twice in the past.
// In case there are no solutions, the function will return NaN.
// Otherwise the function returns either the one solution found, or the
// greater of two possible solutions, which can still be negative.
double Limp::tVel(double vx)
{
	// Note to self:
	// To obtain the smaller solution, -sqrt() has to be used.

	double sC = sqrt(C);
	double c1 = (x0 + v0/sC);
	double c2 = (x0 - v0/sC);
	double t = log(vx/(c1*sC) + sqrt( (vx*vx)/(c1*c1*C) + c2/c1)) / sC;
	return t;
}

// Given the current state of the limp, what velocity is it going to have or
// did it have at location x? Again, there are 0, 1 or 2 possible solutions to
// this question, where in case of two solutions their absolute values are equal
// and only the signs are different. In case there is no solution, the function
// will return NaN. If there is only one solution, it will return it with the
// correct sign. In case there are two solutions, it always returns the positive
// result. Now of course you don't know if you got a single positive solution,
// that cannot be sign flipped, or a positive solution as one of two possible
// solutions. Live with it.
double Limp::vx(double x)
{
	// This calculation is based on the energy formula (constant energy assumption)
	// and is therefore not suitable to take b into account.

	// The most special case has to be handled explicitely.
	if (x0 == 0 and v0 == 0 and x != 0)
		return sqrt(-1); // NaN

	// If the pendulum does not cross the origin, the current location and the
	// query location are not allowed to have opposing signs.
	if (v0*v0 - C*x0*x0 <= 0 and x0*x < 0)
		return sqrt(-1); // NaN

	// If the pendulum crosses the origin, there is exactly one solution for
	// any query point and the sign of the solution can be determined.
	if (v0*v0 - C*x0*x0 > 0)
		return (v0 > 0 ? 1.0 : -1.0) * sqrt(v0*v0 + C*(x*x - x0*x0));

	// Otherwise there are either zero or two solutions and either NaN or
	// the positive result is returned.
	return sqrt(v0*v0 + C*(x*x - x0*x0));
}

// Given the current state of the limp, at what location is it going to reach or
// did reach the velocity vx? Again, there are 0, 1 or 2 possible solutions to
// this question, where in case of two solutions their absolute values are equal
// and only the signs are different. In case there is no solution, the function
// will return NaN. If there is only one solution, it will return it with the
// correct sign. In case there are two solutions, it always returns the positive
// result. Now of course you don't know if you got a single positive solution,
// that cannot be sign flipped, or a positive solution as one of two possible
// solutions. I'm sorry but I'm not ready to complicate the interface until the
// need arises to handle this case.
double Limp::x(double vx)
{
	// The most special case has to be handled explicitely.
	if (x0 == 0 and v0 == 0 and vx != 0)
		return sqrt(-1); // NaN

	// If the pendulum crosses the origin, the current velocity and the
	// query velocity are not allowed to have opposing signs.
	if (v0*v0 - C*x0*x0 > 0 and v0*vx < 0)
		return sqrt(-1); // NaN

	// If the pendulum does not cross the origin, there is exactly one solution
	// for any query velocity and the sign of the solution can be determined.
	if (v0*v0 - C*x0*x0 > 0)
		return (x0 > 0 ? 1.0 : -1.0) * sqrt(x0*x0 + (vx*vx - v0*v0)/C);

	// Otherwise there are either zero or two solutions and either NaN or
	// the positive result is returned.
	return sqrt(x0*x0 + (vx*vx - v0*v0)/C);
}

// Given the current state of the limp, where do I have to place the pendulum
// origin to reach the given velocity vx at the given location x? The location x
// is interpreted relative to the new pendulum origin. This function is useful to
// compute step locations, for example so that the CoM will go through alpha with
// a zero velocity during the next step. The returned value is relative to the
// current pendulum location. The current pivot point does not play a role. If the
// target energy is negative, there is always exactly one solution. If the energy
// is positive, there is either no solution or there are two symmetrical solutions
// with opposing signs. In this case, the "more immediate" solution is returned
// that has the same sign as the desired location x.
double Limp::origin(double x, double vx)
{
	// Filter invalid cases.
	// Check the general case where the sqrt does not return a valid result.
	if ((v0*v0 - vx*vx)/C + x*x < 0)
	{
//		qDebug() << "invalid 1";
		return sqrt(-1); // NaN
	}

	// Filter invalid tasks.
	// If the target energy is positive, the velocities must not have opposing signs.
	if (vx*vx - C*x*x > 0 and v0*vx < 0)
	{
//		qDebug() << "invalid 2";
		return sqrt(-1); // NaN
	}

	// Now calculate the new pendulum origin so that the CoM trajectory passes
	// through the desired state. The energy formula always gives a positive result.
	double x00 = sqrt((v0*v0 - vx*vx)/C + x*x);

	// The sign of the solution is equal to the sign of the desired location.
	// If the energy is positive, two symmetrical solutions exist with the same
	// value and opposing signs.

	return x == 0 ? sign0(v0)*x00 : sign(x)*x00;
}

// Returns the zero moment point offset z that is needed to reach x at time T.
// x is expressed relative to the current pivot point.
double Limp::z(double x, double T)
{
	double sC = sqrt(C);
	double a = exp(sC*T);
	double b = exp(-sC*T);
	double d = v0/sC;
	double z = ((a+b)*x0 + (a-b)*d - 2.0*x) / (a+b-2.0);
	return z;
}

Limp operator*(double scalar, const Limp& l)
{
	return Limp(scalar * l.x0, scalar * l.v0, l.C);
}

Limp operator*(const Limp& l, const double scalar)
{
	return Limp(scalar * l.x0, scalar * l.v0, l.C);
}

Limp operator/(double scalar, const Limp& l)
{
	return Limp(l.x0 / scalar, l.v0 / scalar, l.C);
}

Limp operator/(const Limp& l, const double scalar)
{
	return Limp(l.x0 / scalar, l.v0 / scalar, l.C);
}

QDebug operator<<(QDebug dbg, const Limp &l)
{
	dbg.nospace() << "(" << l.x0 << ", " << l.v0 << ")";

	return dbg.space();
};

}