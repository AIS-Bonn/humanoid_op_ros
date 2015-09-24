// Simple Kalman Filter implementation for one-dimensional processes
// Author: Marcell Missura <missura@ais.uni-bonn.de>
// Modified for Eigen by Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Core>

namespace robotcontrol
{

class KalmanFilter
{
public:
	static const double DefaultTimeStep = 0.010; // Default value used for the time step => Can be overridden using setTimeStep()

	KalmanFilter();
	~KalmanFilter();

	void init();
	void reset();
	void setState(double pos, double vel);
	void update(double z);

	void setTimeStep(double dT) { if(dT > 0.0) m_timeStep = dT; }

	inline double position() const
	{ return m_X[0]; }

	inline double velocity() const
	{ return m_X[1]; }
private:
	Eigen::Vector2d m_X;
	Eigen::Vector2d m_Z;

	Eigen::Matrix2d m_A;
	Eigen::Matrix2d m_H;
	Eigen::Matrix2d m_K;
	Eigen::Matrix2d m_Q;
	Eigen::Matrix2d m_R;
	Eigen::Matrix2d m_P;
	Eigen::Matrix2d m_I;

	double m_smoothing;
	double m_timeStep;
	double m_lastZ;
	double m_px;
	double m_pv;
};

template<int Rows>
class MultiKalmanFilter
{
public:
	typedef Eigen::Matrix<double, Rows, 1> Vector;
	typedef Eigen::Matrix<double, Rows, Rows> Matrix;

	MultiKalmanFilter() {}
	~MultiKalmanFilter() {}

	void setTimeStep(double dT)
	{
		if(dT > 0.0)
		{
			for(int i = 0; i < Rows; ++i)
				m_filters[i].setTimeStep(dT);
		}
	}

	void update(const Vector& z)
	{
		for(int i = 0; i < Rows; ++i)
			m_filters[i].update(z[i]);
	}

	Vector position() const
	{
		Vector ret;
		for(int i = 0; i < Rows; ++i)
			ret[i] = m_filters[i].position();

		return ret;
	}

	Vector velocity() const
	{
		Vector ret;
		for(int i = 0; i < Rows; ++i)
			ret[i] = m_filters[i].velocity();

		return ret;
	}

	void transform(const Matrix& E, const Vector& r)
	{
		Vector pos = position();
		Vector vel = velocity();

		pos = E * (pos - r);
		vel = E * vel;

		for(int i = 0; i < Rows; ++i)
		{
			m_filters[i].setState(pos[i], vel[i]);
			m_filters[i].reset();
		}
	}
private:
	KalmanFilter m_filters[Vector::RowsAtCompileTime];
};

}

#endif
