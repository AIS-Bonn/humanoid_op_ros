// Hold filter (max or min)
// File: hold_filter.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef HOLD_FILTER_H
#define HOLD_FILTER_H

// Includes
#include <boost/circular_buffer.hpp>
#include <cstddef>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class HoldFilter
	* 
	* @brief Hold filter (extremum of the last N data points).
	**/
	template<bool IsMax> class HoldFilter
	{
	public:
		// Typedefs
		typedef boost::circular_buffer<double> Buffer;
		
		// Constructor
		explicit HoldFilter(std::size_t numPoints = 0)
		{
			// Reset all data members
			resetAll(numPoints);
		}
		
		// Reset all function (also sets the capacity of the circular buffer)
		void resetAll(std::size_t numPoints = 0)
		{
			// Reset all data members
			resize(numPoints);
			reset();
		}
		
		// Reset function (does not touch the capacity of the circular buffer)
		void reset()
		{
			// Reset all data members
			m_value = 0.0;
			m_buf.clear();
			m_changed = true;
		}
		
		// Changes the number of data points that is used (the capacity of the internal circular buffer)
		void resize(std::size_t numPoints)
		{
			// Update the circular buffer capacity
			m_buf.set_capacity(numPoints);
			m_changed = true;
		}
		
		// Set the data in the buffer, newest data first
		template<typename Iterator> void setBuf(Iterator first, Iterator last)
		{
			// Set the data in the buffer and fill in the rest with zeros
			std::size_t i = 0;
			for(; first != last; ++first, ++i)
			{
				if(i >= m_buf.size()) break;
				m_buf[i] = *first;
			}
			for(; i < m_buf.size(); ++i)
				m_buf[i] = 0.0;
			m_changed = true;
		}
		
		// Put a new value in the buffer, possibly discarding an old one
		void put(double value)
		{
			// Add the required value to the buffer
			m_buf.push_front(value);
			m_changed = true;
		}
		
		// Get functions
		std::size_t numPoints() const { return m_buf.capacity(); }
		double value() { if(m_changed) calculateValue(); return m_value; }
		bool haveData() const { return !m_buf.empty(); }
		std::size_t numData() const { return m_buf.size(); }
		const Buffer& buf() const { return m_buf; }
		
		// Forced update function (recalculation of the value)
		void update() { calculateValue(); }
		double update(double value) { m_buf.push_front(value); calculateValue(); return m_value; } // Equivalent to put() followed by value()
		
	private:
		// Calculate the value of the data currently in the buffer
		void calculateValue()
		{
			// Calculate the value
			if(m_buf.empty())
				m_value = 0.0;
			else if(IsMax)
				m_value = *std::max_element(m_buf.begin(), m_buf.end());
			else
				m_value = *std::min_element(m_buf.begin(), m_buf.end());
			
			// Reset the changed flag
			m_changed = false;
		}
		
		// Buffered filter value
		double m_value;
		
		// Flag whether the buffer has changed
		bool m_changed;
		
		// Circular buffer for data history ([0] = Newest data, [N-1] = Oldest data)
		Buffer m_buf;
	};

	// Typedefs
	typedef HoldFilter<true> HoldMaxFilter;
	typedef HoldFilter<false> HoldMinFilter;
}

#endif
// EOF