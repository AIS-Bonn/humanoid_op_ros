// Mean filter (moving average)
// File: mean_filter.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef MEAN_FILTER_H
#define MEAN_FILTER_H

// Includes
#include <boost/circular_buffer.hpp>
#include <boost/circular_buffer/base.hpp>
#include <cstddef>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class MeanFilter
	* 
	* @brief Moving average filter (plain average of the last N data points).
	**/
	class MeanFilter
	{
	public:
		// Typedefs
		typedef boost::circular_buffer<double> Buffer;
		
		// Constructor
		explicit MeanFilter(size_t numPoints = 0) : m_mean(0.0), m_changed(true)
		{
			// Initialise the circular buffers
			resize(numPoints);
		}
		
		// Reset function (also zeros the capacity of the circular buffer)
		void resetAll()
		{
			// Reset all data members
			resize(0);
			reset();
		}
		
		// Reset function (clears all points out of the circular buffer but does not change its capacity, use resetAll() to also zero the capacity)
		void reset()
		{
			// Reset all data members
			m_mean = 0.0;
			setZero();
		}
		
		// Changes the number of data points that is used (the capacity of the internal circular buffer)
		void resize(size_t numPoints)
		{
			// Update the circular buffer capacity (if increasing the size we add zero data points, if decreasing the size we keep the last added data points)
			m_buf.set_capacity(numPoints);
			m_buf.resize(m_buf.capacity(), 0.0);
			m_changed = true;
		}
		void resize(size_t numPoints, double fillValue)
		{
			// Update the circular buffer capacity (if increasing the size we add fillValue data points, if decreasing the size we keep the last added data points)
			m_buf.set_capacity(numPoints);
			m_buf.resize(m_buf.capacity(), fillValue);
			m_changed = true;
		}
		
		// Set the data in the buffer, newest data first
		template<typename Iterator> void setBuf(Iterator first, Iterator last)
		{
			// Set the data in the buffer and fill in the rest with zeros
			size_t i = 0;
			for(; first != last; ++first, ++i)
			{
				if(i >= m_buf.size()) break;
				m_buf[i] = *first;
			}
			for(; i < m_buf.size(); ++i)
				m_buf[i] = 0.0;
			m_changed = true;
		}
		
		// Reset the contents of the buffer to zero or a particular value
		void setZero() { std::fill(m_buf.begin(), m_buf.end(), 0.0); m_changed = true; }
		void setValue(double value) { std::fill(m_buf.begin(), m_buf.end(), value); m_changed = true; }
		
		// Put a new value in the buffer, possibly discarding an old one
		void put(double value)
		{
			// Add the required value to the buffer
			m_buf.push_front(value);
			m_changed = true;
		}
		
		// Get functions
		size_t numPoints() const { return m_buf.capacity(); }
		double value() { if(m_changed) calculateMean(); return m_mean; }
		const Buffer& buf() const { return m_buf; }
		
		// Forced update function (recalculation of the mean)
		void update() { calculateMean(); }
		
	private:
		// Calculate the mean of the data currently in the buffer
		void calculateMean()
		{
			// Calculate the mean
			m_mean = 0.0;
			size_t N = m_buf.size();
			for(size_t i = 0; i < N; i++)
				m_mean += m_buf[i];
			if(N > 0)
				m_mean /= N;
			
			// Reset the changed flag
			m_changed = false;
		}
		
		// Buffered filter value
		double m_mean;
		
		// Flag whether the buffer has changed
		bool m_changed;
		
		// Circular buffer for data history ([0] = Newest data, [N-1] = Oldest data)
		Buffer m_buf;
	};
}

#endif
// EOF