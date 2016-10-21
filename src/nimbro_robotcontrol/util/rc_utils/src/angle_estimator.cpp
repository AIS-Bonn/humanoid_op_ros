// Angle estimator based on 2D gyro and 3D acc measurements
// Authors: Max Schwarz <max.schwarz@uni-bonn.de>
//          Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rc_utils/angle_estimator.h>
#include <math.h>

// Namespaces
using namespace stateestimation;

// Utility functions
inline double min(double a, double b) { return (a < b ? a : b); }
inline double max(double a, double b) { return (a > b ? a : b); }

//! AngleEstimator constructor
AngleEstimator::AngleEstimator()
 : m_acc_gain(0.005)
 , m_acc_smooth_decay(1.0)
 , m_acc_x(0)
 , m_acc_y(0)
 , m_acc_z(0)
 , m_lambda(0)
 , m_acc(0,0)
 , m_angle(0,0)
 , m_gyro(0,0)
 , m_gyroBias(0,0)
 , m_accBuffer(200)
 , m_gyroBuffer(200)
 , m_tickCount(0)
{
}

//! AngleEstimator destructor
AngleEstimator::~AngleEstimator()
{
}

//! Prediction step of the fused angle estimate
void AngleEstimator::predict(double dt, double gyroX, double gyroY)
{
	// Calculate the incremental gyro step
	double gyroXdt = gyroX * dt;
	double gyroYdt = gyroY * dt;

	// Predict the next fused angle based on the measured gyro velocities
	m_angle.x() += gyroXdt + m_gyroBias.x(); // Projected roll
	m_angle.y() += gyroYdt + m_gyroBias.y(); // Projected pitch

	// Unbiased integration of the gyro velocities
	m_gyro.x() += gyroXdt;
	m_gyro.y() += gyroYdt;

	// Estimate the gyro bias
	updateGyroBias();
}

//! Update step of the fused angle estimate. It is assumed that the acc vector is expressed in inertial form, meaning that at robot rest it points opposite to the direction of gravity.
void AngleEstimator::update(double accX, double accY, double accZ)
{
	// Low pass filter the accelerometer measurements
	m_acc_x = (1.0 - m_acc_smooth_decay) * m_acc_x + m_acc_smooth_decay * accX;
	m_acc_y = (1.0 - m_acc_smooth_decay) * m_acc_y + m_acc_smooth_decay * accY;
	m_acc_z = (1.0 - m_acc_smooth_decay) * m_acc_z + m_acc_smooth_decay * accZ;

	// Calculate the 'angle measurement' corresponding to the accelerometer value, assuming no inertial acceleration components (i.e. the acc vector points in the direction of gravity)
	m_acc << atan2( m_acc_y, m_acc_z), // Projected roll
	         atan2(-m_acc_x, m_acc_z); // Projected pitch

	// Update the projected angle estimate based on the accelerometer measurement
	m_angle = m_lambda * ((1.0 - m_acc_gain) * m_angle + m_acc_gain * m_acc) + (1.0 - m_lambda) * m_acc;
}

//! Update the gyro bias estimate
void AngleEstimator::updateGyroBias()
{
	// Calculate the lambda blending value for this step
	m_lambda = min(1.0, max(0.0, (m_tickCount - 200.0)/500.0));

	// If we have at least 200 data points in our history buffer then we can estimate a gyro bias
	if(m_tickCount > 200)
	{
		// Calculate the amount that the gyro- and acc-based angle estimates have deviated over the last 200 cycles as an average delta per cycle
		Eigen::Vector2d measGyroBias = ((m_acc - m_accBuffer[0]) - (m_gyro - m_gyroBuffer[0])) / 200.0;

		// Update the estimated gyro bias using the calculated deviation
		double gamma = (1.0 - m_lambda) * 0.02 + 0.001;     // This factor just makes the gyro bias learn faster initially...
		m_gyroBias += gamma * (measGyroBias  - m_gyroBias); // Low pass filter the measured gyro bias to obtain the estimated gyro bias
	}

	// Update our data history buffer
	m_accBuffer.push_back(m_acc);
	m_gyroBuffer.push_back(m_gyro);
	if(m_tickCount < 5000) m_tickCount++; // Saturate our tick count at some point to stop it running away from us and highly theoretically eventually looping back to zero (might take a while!)
}
// EOF