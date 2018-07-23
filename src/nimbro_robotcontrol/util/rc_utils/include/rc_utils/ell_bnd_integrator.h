// Ellipsoidally bounded integrator class
// File: ell_bnd_integrator.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef ELL_BND_INTEGRATOR
#define ELL_BND_INTEGRATOR

// Includes
#include <rc_utils/math_vec_mat.h>
#include <boost/utility/enable_if.hpp>
#include <Eigen/Core>
#include <limits>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class EllBndIntegrator
	* 
	* @brief N-dimensional ellipsoidally bounded integrator.
	* 
	* The ellipsoidal bounding is performed in a soft manner using `coerceSoftEllC`,
	* and the ellipsoid semiaxes values are treated by absolute value.
	**/
	template<int N> class EllBndIntegrator
	{
	public:
		// Integrator dimension
		static_assert(N >= 1, "The dimension N of EllBndIntegrator must be a positive integer!");
		static const int ND;

		// Typedefs
		typedef Eigen::Matrix<double, N, 1> Vec;

		// Constructor
		EllBndIntegrator() { resetAll(); }

		// Reset functions
		void resetAll()
		{
			unsetEllBound();
			reset();
		}
		void reset()
		{
			m_integral.setZero();
			m_lastInput.setZero();
		}

		// Set ellipsoid parameters
		void setEllBound(const Vec& semiaxes, double buffer = 0.0) { m_semiaxes = semiaxes; m_buffer = buffer; }
		void unsetEllBound() { m_semiaxes.fill(std::numeric_limits<double>::infinity()); m_buffer = 0.0; }
		void setBuffer(double buffer) { m_buffer = buffer; }

		// Set integral
		void setIntegral(const Vec& value) { m_integral = value; }
		void setIntegralZero() { m_integral.setZero(); }
		template<int M = N> typename boost::enable_if_c<M >= 1 && N >= 1, void>::type setIntegralXZero() { m_integral.x() = 0.0; }
		template<int M = N> typename boost::enable_if_c<M >= 2 && N >= 2, void>::type setIntegralYZero() { m_integral.y() = 0.0; }
		template<int M = N> typename boost::enable_if_c<M >= 3 && N >= 3, void>::type setIntegralZZero() { m_integral.z() = 0.0; }
		void clearLastInput() { m_lastInput.setZero(); }

		// Get functions
		Vec semiaxes() const { return m_semiaxes; }
		double buffer() const { return m_buffer; }
		Vec integral() const { return m_integral; }

		// Update the integral with a new input
		Vec integrate(const Vec& input, double dt)
		{
			m_integral += (0.5*dt) * (input + m_lastInput);
			coerceSoftEllC(m_semiaxes, m_integral, m_buffer);
			m_lastInput = input;
			return m_integral;
		}

	private:
		// Data members
		Vec m_semiaxes;
		double m_buffer;
		Vec m_integral;
		Vec m_lastInput;
	};

	// EllBndIntegrator constants
	template<int N> const int EllBndIntegrator<N>::ND = N;

	// Typedefs
	typedef EllBndIntegrator<1> EllBndIntegrator1D;
	typedef EllBndIntegrator<2> EllBndIntegrator2D;
	typedef EllBndIntegrator<3> EllBndIntegrator3D;
	typedef EllBndIntegrator<4> EllBndIntegrator4D;
}

#endif
// EOF