// Implements a class that filters spikes out of a signal.
// File: spike_filter.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef SPIKE_FILTER_H
#define SPIKE_FILTER_H

// Includes
#include <rc_utils/math_funcs.h>
#include <cmath>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class SpikeFilter
	* 
	* @brief Filter spikes out of a signal.
	* 
	* @c maxDelta is the maximum magnitude of increment by which the value is allowed to change in a single call to @c put.
	**/
	class SpikeFilter
	{
	public:
		// Constructor
		SpikeFilter() { reset(); }
		explicit SpikeFilter(double maxDelta, double value = 0.0) { set(maxDelta, value); }
		
		// Reset function
		void reset() { m_maxDelta = m_value = 0.0; m_hold = 0; }
		
		// Set functions
		void set(double maxDelta, double value = 0.0) { m_maxDelta = fabs(maxDelta); m_value = value; m_hold = 0; }
		void setValue(double value) { m_value = value; m_hold = 0; } // Note: This sets the raw value, ignoring spike filtering
		void setMaxDelta(double maxDelta) { m_maxDelta = fabs(maxDelta); }
		
		// Get functions
		double maxDelta() const { return m_maxDelta; }
		double value() const { return m_value; }
		int holdCount() const { return m_hold; }
		
		// Put function
		double put(double newValue)
		{
			if(fabs(newValue - m_value) > (m_hold + 1)*m_maxDelta)
				m_hold++;
			else
			{
				m_value = newValue;
				m_hold = 0;
			}
			return m_value;
		}
		
	private:
		// Data members
		double m_maxDelta;
		double m_value;
		int m_hold;
	};
}

#endif
// EOF