// Mean filter (moving average)
// File: mean_filter.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef MEAN_FILTER_H
#define MEAN_FILTER_H

// Includes
#include <boost/circular_buffer.hpp>
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
		explicit MeanFilter(std::size_t len = 0)
		{
			// Initialise the data buffer
			resetAll(len);
		}

		// Reset function (including data buffer length)
		void resetAll(std::size_t len = 0)
		{
			// Reset all data members
			resize(len);
			reset();
		}

		// Reset function
		void reset()
		{
			// Reset all data members
			m_mean = 0.0;
			zeroBuf();
		}

		// Change the length of the data buffer
		void resize(std::size_t len, double value = 0.0)
		{
			// Update the data buffer length (Increase => Add value data points, Decrease => Keep latest data points)
			m_buf.set_capacity(len);
			m_buf.resize(m_buf.capacity(), value);
			m_changed = true;
		}

		// Fill the data buffer
		void zeroBuf() { std::fill(m_buf.begin(), m_buf.end(), 0.0); m_changed = true; }
		void fillBuf(double value) { std::fill(m_buf.begin(), m_buf.end(), value); m_changed = true; }

		// Set the data in the buffer, newest data first
		template<typename Iterator> void setBuf(Iterator first, Iterator last, double value = 0.0)
		{
			// Set the data in the buffer and fill in the rest with value
			std::size_t i = 0;
			for(; first != last && i < m_buf.size(); ++first, ++i)
				m_buf[i] = *first;
			for(; i < m_buf.size(); ++i)
				m_buf[i] = value;
			m_changed = true;
		}

		// Put a new value in the buffer, possibly discarding an old one
		void put(double value)
		{
			// Add the required value to the buffer
			m_buf.push_front(value);
			m_changed = true;
		}

		// Get data buffer
		const Buffer& buf() const { return m_buf; }
		bool changed() const { return m_changed; }

		// Get filter properties
		std::size_t len() const { return m_buf.capacity(); }
		bool lenZero() const { return m_buf.capacity() == 0; }

		// Get filtered output
		double mean() { if(m_changed) calculateMean(); return m_mean; } // Calls update() automatically if the buffer has changed
		double meanC() const { return m_mean; } // Need to manually call update() for this to be the latest value

		// Update functions
		MeanFilter& update() { calculateMean(); return *this; }
		double update(double value) { m_buf.push_front(value); calculateMean(); return m_mean; } // Equivalent to put(value) -> update() -> return mean()
		double updatedMean() { calculateMean(); return m_mean; } // Equivalent to update() -> return mean()

	private:
		// Calculate the mean of the data currently in the buffer
		void calculateMean()
		{
			// Calculate the mean
			m_mean = 0.0;
			std::size_t N = m_buf.size();
			for(std::size_t i = 0; i < N; i++)
				m_mean += m_buf[i];
			if(N > 0)
				m_mean /= N;

			// Reset the changed flag
			m_changed = false;
		}

		// Buffered filter value
		double m_mean;

		// Data buffer (0 = Newest data, N-1 = Oldest data)
		Buffer m_buf;
		bool m_changed;
	};
}

#endif
// EOF