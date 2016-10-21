// Exponentially weighted integrator class
// File: ew_integrator.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef EW_INTEGRATOR_H
#define EW_INTEGRATOR_H

// Includes
#include <rc_utils/math_funcs.h>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class EWIntegrator
	* 
	* @brief Integrate data using a continuous exponentially weighted integration method.
	* 
	* Updates to this class are assumed to be equally spaced temporally.
	* At each point in time the value of the integration is given by
	*   I[n] = x[n] + a*x[n-1] + a^2*x[n-2] + a^3*x[n-3] + ...
	* where `x` is the input data stream and `a` is the alpha factor, calculated from the desired half life time.
	* The half life time is the number of zero updates that it should take for the integrated value to decay to
	* half of what it was. Alternatively, the half life time can be viewed as the half life of the exponential that
	* governs the weighting of past data (older data gets weighted less in the integration).
	**/
	class EWIntegrator
	{
	public:
		// Constructor
		EWIntegrator() { resetAll(); }
		
		// Reset functions
		void resetAll() // Reset everything, including alpha
		{
			m_alpha = 1.0;
			m_integral = 0.0;
		}
		void reset() // Resets the integrated value to zero (doesn't modify alpha)
		{
			m_integral = 0.0;
		}
		
		// Set functions
		void setIntegral(double value) { m_integral = value; }
		void resetIntegral() { m_integral = 0.0; }
		void setAlpha(double alpha) { m_alpha = coerce(alpha, 0.0, 1.0); }
		void setHalfLife(double cycles) { m_alpha = (cycles <= 1e-6 ? 0.0 : std::pow(0.5, 1.0/cycles)); }
		
		// Get functions
		double alpha() const { return m_alpha; }
		double integral() const { return m_integral; }
		
		// Update the integral with a new value
		void integrate(double value) { m_integral = value + m_alpha*m_integral; }
		
	private:
		// Data members
		double m_alpha;    // Alpha factor that governs the exponential decay of the data point weighting
		double m_integral; // Current integrated value
	};
}

#endif
// EOF