// Angle estimator based on 2D gyro and 3D acc measurements
// Authors: Max Schwarz <max.schwarz@uni-bonn.de>
//          Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef ANGLEESTIMATOR_H
#define ANGLEESTIMATOR_H

// Includes
#include <Eigen/Core>
#include <boost/circular_buffer.hpp>

// State estimation namespace
namespace stateestimation
{
	/**
	* @class AngleEstimator
	*
	* @brief Fuses 2D gyroscope and 3D accelerometer data into a projected pitch/roll estimate.
	**/
	class AngleEstimator
	{
	public:
		// Constructor/destructor
		AngleEstimator();
		virtual ~AngleEstimator();

		// Prediction/estimation functions
		void predict(double dt, double gyroX, double gyroY);
		void update(double accX, double accY, double accZ);

		// Get functions
		inline double accGain() const { return m_acc_gain; }
		inline double accSmoothDecay() const { return m_acc_smooth_decay; }
		inline double projPitch() const { return m_angle.y(); }
		inline double projRoll() const { return m_angle.x(); }

		// Set functions
		void setAccGain(double accGain) { m_acc_gain = accGain; }
		void setAccSmoothDecay(double accSmoothDecay) { m_acc_smooth_decay = accSmoothDecay; }

	private:
		// Bias estimation function
		void updateGyroBias();

		// Internal variables
		double m_acc_gain;
		double m_acc_smooth_decay;
		double m_acc_x;
		double m_acc_y;
		double m_acc_z;
		double m_lambda;
		Eigen::Vector2d m_acc;
		Eigen::Vector2d m_angle;
		Eigen::Vector2d m_gyro;
		Eigen::Vector2d m_gyroBias;

		// Data history buffer
		typedef boost::circular_buffer<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > VectorBuffer;
		VectorBuffer m_accBuffer;
		VectorBuffer m_gyroBuffer;
		unsigned int m_tickCount;
	};
}

#endif /* ANGLEESTIMATOR_H */
// EOF