#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

#include <Eigen/Core>

namespace margait_contrib
{

/*
This is a simple Kalman Filter implementation that estimates the state of a one
dimensional process. The state consists of the position x, the velocity v, and
the acceleration a. Instantiate it, provide a time step parameter (default is
0.01 seconds), and provide smoothing parameters for the x, v, and a dimensions
(default 0.1, 0.1, 0.1). Keep calling the update(z) function periodically to
provide position measurements z. The output will be written into the public x,
v, and a variables.

KalmanFilter kf;
kf.setTimeStep(0.01);
kf.setSmoothing(0.1, 0.1, 0.1);

for_every_ten_milliseconds
{
	double z = currentPositionMeasurement();
	kf.update(z);

	double position = kf.x;
	double velocity = kf.v;
	double acceleration = kf.a;
}

*/

class KalmanFilter
{
	Eigen::Vector3d X;
	Eigen::Vector3d Z;
	Eigen::Matrix3d A;
	Eigen::Matrix3d H;
	Eigen::Matrix3d K;
	Eigen::Matrix3d Q;
	Eigen::Matrix3d R;
	Eigen::Matrix3d P;
	Eigen::Matrix3d I;

	double smoothing;
	double timeStep;
	double lastX;
	double lastV;

	bool adaptiveGain;

public:

	double x;
	double v;
	double a;
	double px;
	double pv;
	double pa;

	KalmanFilter();
	~KalmanFilter(){};

	void init();
	void setTimeStep(double t);
	void setSmoothing(double s);
	void reset(double xx=0, double vv=0, double aa=0);
	void update(double z);
};

}

#endif // KALMANFILTER_H_
