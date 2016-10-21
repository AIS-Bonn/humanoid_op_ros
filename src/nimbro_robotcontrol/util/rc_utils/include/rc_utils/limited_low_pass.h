// Implements a class that applies a slope-limited low pass filter to a signal.
// File: limited_low_pass.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef LIMITED_LOW_PASS_H
#define LIMITED_LOW_PASS_H

// Includes
#include <rc_utils/low_pass_filter.h>
#include <rc_utils/math_funcs.h>
#include <cmath>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class LimitedLowPass
	* 
	* @brief Apply a slope-limited low pass filter to a signal.
	* 
	* @c maxDelta is the maximum magnitude of increment by which the value is allowed to change in a single call to @c put.
	**/
	class LimitedLowPass : public LowPassFilterT<double>
	{
	public:
		explicit LimitedLowPass(double Ts = 100.0, double maxDelta = 0.0) : LowPassFilterT(Ts) { setMaxDelta(maxDelta); }
		
		// Reset functions
		void resetAll(double Ts = 100.0, double maxDelta = 0.0) { LowPassFilterT::resetAll(Ts); setMaxDelta(maxDelta); }
		void reset() { LowPassFilterT::reset(); m_maxDelta = 0.0; }
		
		// Set functions
		void setMaxDelta(double maxDelta) { m_maxDelta = fabs(maxDelta); }
		void setParams(double Ts, double maxDelta) { setTs(Ts); setMaxDelta(maxDelta); }
		
		// Get functions
		double maxDelta() const { return m_maxDelta; }
		
		// Put functions
		void put(double value) { if(!m_freeze) m_value += coerceAbs(m_alpha*(value - m_value), m_maxDelta); }
		void put(double value, double maxDelta) { if(!m_freeze) m_value += coerceAbs(m_alpha*(value - m_value), maxDelta); }
		
	private:
		// Data members
		double m_maxDelta;
	};
}

#endif
// EOF