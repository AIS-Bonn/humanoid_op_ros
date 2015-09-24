#include <cap_gait/contrib/KalmanFilter.h>
#include <Eigen/LU>

using namespace margait_contrib;

KalmanFilter::KalmanFilter()
{
	lastX = 0;
	lastV = 0;
	smoothing = 0.1;
	timeStep = 0.010;
	x = 0;
	v = 0;
	a = 0;
	px = 0;
	pv = 0;
	pa = 0;

	// When the gain is adaptive, the smoothing config parameters don't make any difference.
	adaptiveGain = true;

	init();
}

// Prepares (resets) the noise matrices and the kalman gain, but preserves the current state.
// This is good for changing the smoothing on the fly without messing with the state estimate.
void KalmanFilter::init()
{
	// Process noise.
	Q = Eigen::Matrix3d::Identity();

	// Measurement noise.
	R = (0.001 + smoothing*100.0) * Eigen::Matrix3d::Identity();

	// Passive linear system dynamics matrix A.
	A << 1.0, timeStep,   0.0   ,
	     0.0,   1.0   , timeStep,
	     0.0,   0.0   ,   1.0   ;

	// State and measurement vectors.
	X << x, v, a;
	Z.setZero();

	// Other matrices
	H = I = K = Eigen::Matrix3d::Identity();
	P.setZero();
}

// Sets the smoothing parameter.
void KalmanFilter::setSmoothing(double s)
{
	smoothing = s;
	init();
}

// Sets the time interval between the updates. It's set to 10 milliseconds by default.
void KalmanFilter::setTimeStep(double t)
{
	timeStep = t;
	init();
}

// Resets the state to the provided parameters, but doesn't change the noise and the kalman gain.
// This is good to set the state estimate on the fly without messing with the Kalman gain.
void KalmanFilter::reset(double xx, double vv, double aa)
{
	x = xx;
	v = vv;
	a = aa;

	X << x, v, a;

	lastX = x;
	lastV = v;
}

// Updates the process. Provide a position measurement z.
void KalmanFilter::update(double z)
{
	double Zv = (z - lastX)/timeStep;
	Z << z, Zv, (Zv - lastV)/timeStep;
	lastX = Z(0);
	lastV = Z(1);

	if (adaptiveGain)
	{
		X = A*X;
		P = A*P*A.transpose() + Q;
		K = P*H.transpose() * (H*P*H.transpose() + R).inverse();
		X = X + K*(Z - H*X);
		P = (I - K*H)*P;
	}
	else
	{
		X = A*X;
		K = Q*H.transpose() * (H*Q*H.transpose() + R).inverse();
		X = X + K*(Z - H*X);
	}

	x = X(0);
	v = X(1);
	a = X(2);

	px = P(0,0);
	pv = P(1,1);
	pa = P(2,2);
}
