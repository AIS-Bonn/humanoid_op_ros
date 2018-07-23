// Feedback gait keypoint trajectory generation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/trajectory/keypoint/feed_keypoint_traj.h>

// Namespaces
using namespace feed_gait;
using namespace feed_gait::keypoint_traj;

//
// KeypointTrajConfig class
//

// Constants
const double KeypointTrajConfig::MinPhaseSep = 0.05; // Must be greater than 0 and less than pi/8 ~= 0.3927!

//
// SplineCoeff struct
//

// Reset function
void SplineCoeff::reset()
{
	// Reset the data members
	T = 0.0;
	a0 = a1 = a2 = a3 = 0.0;
}

// Set function with target position
void SplineCoeff::set(double T, double x0, double xT, double v0, double vT)
{
	// Calculate the mean velocity and pass on the work to the other overload
	double vbar = (xT - x0)/T;
	setMV(T, x0, vbar, v0, vT);
}

// Set function with mean velocity
void SplineCoeff::setMV(double T, double x0, double vbar, double v0, double vT)
{
	// Transcribe the nominal t range
	this->T = T;

	// Calculate the required spline coefficients
	a0 = x0;
	a1 = v0;
	a2 = (3.0*vbar - 2.0*v0 - vT) / T;
	a3 = (v0 + vT - 2.0*vbar) / (T*T);
}

//
// SuppCoeffVars struct
//

// Reset function
void SuppCoeffVars::reset()
{
	// Reset the data members
	centrePtRise = -M_PI_2;
	centrePtFall = M_PI_2;
	transitionSlope = 1.0;
	transitionMargin = 0.5;
}
// EOF