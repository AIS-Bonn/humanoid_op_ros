// Simple Kalman Filter implementation for one-dimensional processes
// Author: Marcell Missura <missura@ais.uni-bonn.de>
// Modified for Eigen by Max Schwarz <max.schwarz@uni-bonn.de>

#include <robotcontrol/model/kalmanfilter.h>

#include <Eigen/LU>

namespace robotcontrol
{

KalmanFilter::KalmanFilter()
 : m_smoothing(0.3)
 , m_timeStep(DefaultTimeStep)
 , m_lastZ(0)
 , m_px(0)
 , m_pv(0)
{
	init();
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::init()
{
	// Process noise
	m_Q = Eigen::Matrix2d::Identity();

	// Measurement noise
	m_R << 0.001 + m_smoothing*100.0, 0.0,
	       0.0,                       0.001 + m_smoothing*100.0;

	// Passive linear system dynamics matrix A
	m_A << 1.0, m_timeStep,
	       0.0, 1.0;

	// State and measurement vectors
	m_X << 0.0, 0.0;

	m_H = m_I = m_K = m_P = Eigen::Matrix2d::Identity();
}

void KalmanFilter::reset()
{
	m_K = Eigen::Matrix2d::Identity();
	m_P = Eigen::Matrix2d::Zero();
}

void KalmanFilter::update(double z)
{
	m_Z << z, (z - m_lastZ) / m_timeStep;

	m_X = m_A * m_X;
	m_P = m_A * m_P * m_A.transpose() + m_Q;
	m_K = m_P * m_H.transpose() * (m_H * m_P * m_H.transpose() + m_R).inverse();
	m_X = m_X + m_K * (m_Z - m_H * m_X);
	m_P = (m_I - m_K * m_H) * m_P;

	m_lastZ = z;

	m_px = m_P(0,0);
	m_pv = m_P(1,1);
}

void KalmanFilter::setState(double pos, double vel)
{
	m_X << pos, vel;
	m_lastZ = pos;
}

}

