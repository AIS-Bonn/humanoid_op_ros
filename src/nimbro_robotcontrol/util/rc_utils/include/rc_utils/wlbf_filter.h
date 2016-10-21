// Weighted line of best fit filter
// File: wlbf_filter.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef WLBF_FILTER_H
#define WLBF_FILTER_H

// Includes
#include <boost/circular_buffer.hpp>
#include <boost/circular_buffer/base.hpp>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class WLBFFilter
	* 
	* @brief Dynamic weighted line of best fit filter that smooths and calculates the first order derivative.
	* 
	* A data point (x,y) with weight w is equivalent to the same data point added w^2 times with weight 1.
	* As such, using a negative weight is equivalent to using its absolute value.
	**/
	class WLBFFilter
	{
	public:
		// Typedefs
		typedef boost::circular_buffer<double> Buffer;
		
		// Constructor
		explicit WLBFFilter(size_t numPoints = 0) : m_A(0.0), m_B(0.0), m_xMean(0.0), m_yMean(0.0), m_changed(true)
		{
			// Initialise the circular buffers
			resize(numPoints);
		}
		
		// Reset function (also zeros the capacity of the circular buffers)
		void resetAll()
		{
			// Reset all data members
			resize(0);
			reset();
		}
		
		// Reset function (clears all points out of the circular buffers but does not change their capacity, use resetAll() to also zero the capacity)
		void reset()
		{
			// Reset all data members
			m_A = m_B = m_xMean = m_yMean = 0.0;
			setZero();
		}
		
		// Changes the number of data points that is used (the capacity of the internal circular buffers)
		void resize(size_t numPoints)
		{
			// Update the circular buffer capacities (if increasing the size we add zero data points, if decreasing the size we keep the last added data points)
			m_xBuf.set_capacity(numPoints);
			m_yBuf.set_capacity(numPoints);
			m_wBuf.set_capacity(numPoints);
			m_xBuf.resize(m_xBuf.capacity(), 0.0);
			m_yBuf.resize(m_yBuf.capacity(), 0.0);
			m_wBuf.resize(m_wBuf.capacity(), 0.0);
			m_changed = true;
		}
		
		// Set the data in any of the three buffers, newest data first
		template<typename Iterator> void setXBuf(Iterator first, Iterator last)
		{
			// Set the data in the buffer and fill in the rest with zeros
			size_t i = 0;
			for(; first != last; ++first, ++i)
			{
				if(i >= m_xBuf.size()) break;
				m_xBuf[i] = *first;
			}
			for(; i < m_xBuf.size(); ++i)
				m_xBuf[i] = 0.0;
			m_changed = true;
		}
		template<typename Iterator> void setYBuf(Iterator first, Iterator last)
		{
			// Set the data in the buffer and fill in the rest with zeros
			size_t i = 0;
			for(; first != last; ++first, ++i)
			{
				if(i >= m_yBuf.size()) break;
				m_yBuf[i] = *first;
			}
			for(; i < m_yBuf.size(); ++i)
				m_yBuf[i] = 0.0;
			m_changed = true;
		}
		template<typename Iterator> void setWBuf(Iterator first, Iterator last) // Note: The weights are squared, so a negative weight is equivalent to a positive one!
		{
			// Set the data in the buffer and fill in the rest with zeros
			size_t i = 0;
			for(; first != last; ++first, ++i)
			{
				if(i >= m_wBuf.size()) break;
				m_wBuf[i] = (*first) * (*first);
			}
			for(; i < m_wBuf.size(); ++i)
				m_wBuf[i] = 0.0;
			m_changed = true;
		}
		
		// Reset the contents of a buffer to all zero
		void setZero() { setZeroX(); setZeroY(); setZeroW(); }
		void setZeroX() { std::fill(m_xBuf.begin(), m_xBuf.end(), 0.0); m_changed = true; }
		void setZeroY() { std::fill(m_yBuf.begin(), m_yBuf.end(), 0.0); m_changed = true; }
		void setZeroW() { std::fill(m_wBuf.begin(), m_wBuf.end(), 0.0); m_changed = true; }
		
		// Set all weights to be equal
		void setEqualW() { std::fill(m_wBuf.begin(), m_wBuf.end(), 1.0); m_changed = true; }
		
		// Set an initial XY state of the filter (set all internal data points to be this state, leaving weights untouched)
		void setInitXY(double x, double y)
		{
			std::fill(m_xBuf.begin(), m_xBuf.end(), x);
			std::fill(m_yBuf.begin(), m_yBuf.end(), y);
			m_changed = true;
		}
		
		// Set an initial XYW state of the filter (set all internal data points to be this state)
		void setInitXYW(double x, double y, double w = 1.0)
		{
			std::fill(m_xBuf.begin(), m_xBuf.end(), x);
			std::fill(m_yBuf.begin(), m_yBuf.end(), y);
			std::fill(m_wBuf.begin(), m_wBuf.end(), w*w);
			m_changed = true;
		}
		
		// Add a weight (this keeps the buffer of (x,y) data points fixed and only adds the weight w for the existing newest data, every other point thereby receives a new weight, shifted by one place)
		void addW(double w) // Note: The weight w is squared, so a negative weight is equivalent to a positive one!
		{
			// Add the required weight to the w buffer
			m_wBuf.push_front(w*w);
			m_changed = true;
		}
		
		// Add a data point (this keeps the buffer of weights fixed and only adds a new (x,y) data point, every other point thereby receives a new weight, shifted by one place)
		void addXY(double x, double y)
		{
			// Add the required data point to the x and y buffers
			m_xBuf.push_front(x);
			m_yBuf.push_front(y);
			m_changed = true;
		}
		
		// Add a data point (this keeps (x,y) pairs locked with their corresponding weights w)
		void addXYW(double x, double y, double w = 1.0) // Note: The weight w is squared, so a negative weight is equivalent to a positive one!
		{
			// Add the required data point to all of the buffers
			m_xBuf.push_front(x);
			m_yBuf.push_front(y);
			m_wBuf.push_front(w*w);
			m_changed = true;
		}
		
		// Get functions
		bool empty() const { return (m_xBuf.capacity() == 0); }
		size_t size() const { return m_xBuf.capacity(); }
		double value() { if(m_changed) calculateWLBF(); return (m_A + m_yMean) + m_B*(m_xBuf.front() - m_xMean); }
		double deriv() { if(m_changed) calculateWLBF(); return m_B; }
		double getA() const { return m_A + m_yMean - m_B*m_xMean; }
		double getB() const { return m_B; }
		void updateAB() { calculateWLBF(); }
		const Buffer& xBuf() const { return m_xBuf; }
		const Buffer& yBuf() const { return m_yBuf; }
		const Buffer& wBuf() const { return m_wBuf; } // Note: The weights are the square of whatever was fed into the buffer
		
	private:
		// Calculate the parameters of the weighted line of best fit y = A + B*x
		void calculateWLBF()
		{
			// Retrieve the size and check it's not empty
			size_t N = m_xBuf.size();
			if(N == 0)
			{
				m_A = m_B = m_xMean = m_yMean = 0.0;
				m_changed = false;
				return;
			}
			
			// Calculate the mean x and y
			m_xMean = 0.0;
			m_yMean = 0.0;
			for(size_t i = 0; i < N; i++)
			{
				m_xMean += m_xBuf[i];
				m_yMean += m_yBuf[i];
			}
			m_xMean /= N;
			m_yMean /= N;
			
			// Precalculate terms
			double sumw = 0.0, sumwx = 0.0, sumwy = 0.0, sumwxx = 0.0, sumwxy = 0.0;
			for(size_t i = 0; i < N; i++)
			{
				double x = m_xBuf[i] - m_xMean;
				double y = m_yBuf[i] - m_yMean;
				double w = m_wBuf[i];
				double wx = w*x;
				sumw += w;
				sumwx += wx;
				sumwy += w*y;
				sumwxx += wx*x;
				sumwxy += wx*y;
			}
			
			// Calculate the slope and offset of the weighted line of best fit
			double denom = sumw*sumwxx - sumwx*sumwx;
			if(denom == 0.0)
				m_A = m_B = m_xMean = m_yMean = 0.0;
			else
			{
				m_A = (sumwy*sumwxx - sumwxy*sumwx) / denom;
				m_B = (sumw*sumwxy - sumwx*sumwy) / denom;
			}
			
			// Reset the changed flag
			m_changed = false;
		}
		
		// Line of best fit parameters
		double m_A;
		double m_B;
		double m_xMean;
		double m_yMean;
		
		// Flag whether the buffer has changed
		bool m_changed;
		
		// Circular buffers for data history ([0] = Newest data, [N-1] = Oldest data)
		Buffer m_xBuf;
		Buffer m_yBuf;
		Buffer m_wBuf;
	};
}

#endif
// EOF