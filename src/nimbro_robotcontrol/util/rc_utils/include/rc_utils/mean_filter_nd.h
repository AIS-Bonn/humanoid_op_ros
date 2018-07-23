// N-dimensional mean filter (moving average)
// File: mean_filter_nd.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef MEAN_FILTER_ND_H
#define MEAN_FILTER_ND_H

// Includes
#include <boost/circular_buffer.hpp>
#include <type_traits>
#include <Eigen/Core>
#include <cstddef>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class MeanFilterND
	* 
	* @brief N-dimensional moving average filter (plain average of the last N data points).
	**/
	template<int N> class MeanFilterND
	{
	public:
		// Filter dimension
		static_assert(N >= 1, "The dimension N of MeanFilterND must be a positive integer!");
		static const int ND;

		// Typedefs
		typedef typename std::conditional<N == 1, double, Eigen::Matrix<double, N, 1>>::type Vec;
		typedef typename std::conditional<N == 1, boost::circular_buffer<Vec>, boost::circular_buffer<Vec, Eigen::aligned_allocator<Vec>>>::type Buffer;

		// Constructor
		explicit MeanFilterND(std::size_t len = 0)
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
			m_mean = ZeroVec;
			zeroBuf();
		}

		// Change the length of the data buffer
		void resize(std::size_t len, const Vec& value = getZeroVec())
		{
			// Update the data buffer length (Increase => Add value data points, Decrease => Keep latest data points)
			m_buf.set_capacity(len);
			m_buf.resize(m_buf.capacity(), value);
			m_changed = true;
		}

		// Fill the data buffer
		void zeroBuf() { std::fill(m_buf.begin(), m_buf.end(), ZeroVec); m_changed = true; }
		void fillBuf(const Vec& value) { std::fill(m_buf.begin(), m_buf.end(), value); m_changed = true; }

		// Set the data in the buffer, newest data first
		template<typename Iterator> void setBuf(Iterator first, Iterator last, const Vec& value = getZeroVec())
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
		template<int M = N> typename std::enable_if<M == 2 && N == 2, void>::type put(double x, double y) { put(Vec(x, y)); }
		template<int M = N> typename std::enable_if<M == 3 && N == 3, void>::type put(double x, double y, double z) { put(Vec(x, y, z)); }
		void put(const Vec& value)
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
		Vec mean() { if(m_changed) calculateMean(); return m_mean; } // Calls update() automatically if the buffer has changed
		Vec meanC() const { return m_mean; } // Need to manually call update() for this to be the latest value

		// Update functions
		MeanFilterND<N>& update() { calculateMean(); return *this; }
		Vec updatedMean() { calculateMean(); return m_mean; } // Equivalent to update() -> return mean()
		Vec update(const Vec& value) { m_buf.push_front(value); calculateMean(); return m_mean; } // Equivalent to put(value) -> update() -> return mean()
		template<int M = N> typename std::enable_if<M == 2 && N == 2, Vec>::type update(double x, double y) { return update(Vec(x, y)); } // Equivalent to put(x, y) -> update() -> return mean()
		template<int M = N> typename std::enable_if<M == 3 && N == 3, Vec>::type update(double x, double y, double z) { return update(Vec(x, y, z)); } // Equivalent to put(x, y, z) -> update() -> return mean()

		// Zero vector constant
		static Vec getZeroVec() { return Vec::Zero(); }
		const Vec ZeroVec = getZeroVec();

	private:
		// Calculate the mean of the data currently in the buffer
		void calculateMean()
		{
			// Calculate the mean
			m_mean = ZeroVec;
			std::size_t len = m_buf.size();
			for(std::size_t i = 0; i < len; i++)
				m_mean += m_buf[i];
			if(len > 0)
				m_mean /= len;

			// Reset the changed flag
			m_changed = false;
		}

		// Buffered filter value
		Vec m_mean;

		// Data buffer (0 = Newest data, N-1 = Oldest data)
		Buffer m_buf;
		bool m_changed;
	};

	// MeanFilterND constants
	template<int N> const int MeanFilterND<N>::ND = N;

	// Zero vector constant
	template<> inline MeanFilterND<1>::Vec MeanFilterND<1>::getZeroVec() { return 0.0; }

	// Typedefs
	typedef MeanFilterND<1> MeanFilter1D;
	typedef MeanFilterND<2> MeanFilter2D;
	typedef MeanFilterND<3> MeanFilter3D;
	typedef MeanFilterND<4> MeanFilter4D;
}

#endif
// EOF