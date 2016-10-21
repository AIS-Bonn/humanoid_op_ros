// Implements a class that applies a slope limit to a signal.
// File: slope_limiter.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef SLOPE_LIMITER_H
#define SLOPE_LIMITER_H

// Includes
#include <rc_utils/math_funcs.h>
#include <cmath>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class SlopeLimiter
	* 
	* @brief Add a slope limit to a signal.
	* 
	* @c maxDelta is the maximum magnitude of increment by which the value is allowed to change in a single call to @c put.
	**/
	class SlopeLimiter
	{
	public:
		// Constructor
		SlopeLimiter() { reset(); }
		explicit SlopeLimiter(double maxDelta, double value = 0.0) { set(maxDelta, value); }
		
		// Reset function
		void reset() { m_maxDelta = m_value = 0.0; }
		
		// Set functions
		void set(double maxDelta, double value = 0.0) { m_maxDelta = fabs(maxDelta); m_value = value; }
		void setValue(double value) { m_value = value; } // Note: This sets the raw value, overriding slope limiting
		
		// Get functions
		double maxDelta() const { return m_maxDelta; }
		double value() const { return m_value; }
		
		// Put function
		double put(double newValue) { m_value += coerceAbs(newValue - m_value, m_maxDelta); return m_value; }

		// Static evaluation function
		static double eval(double newValue, double oldValue, double maxDelta) { return oldValue + coerceAbs(newValue - oldValue, fabs(maxDelta)); }
		
	private:
		// Data members
		double m_maxDelta;
		double m_value;
	};
}

#endif
// EOF