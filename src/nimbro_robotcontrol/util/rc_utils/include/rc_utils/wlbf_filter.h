// Weighted line of best fit filter
// File: wlbf_filter.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef WLBF_FILTER_H
#define WLBF_FILTER_H

// Includes
#include <boost/circular_buffer.hpp>
#include <cstddef>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class WLBFFilter
	* 
	* @brief Weighted line of best fit filter that smooths and calculates the first order derivative.
	* 
	* The supplied weights are effectively squared, so adding a data point with weight w is equivalent to adding the
	* same data point w^2 times with weight 1. As such, negative weights are effectively treated by absolute value.
	* 
	* Unless the buffers are otherwise manually initialised, by default the buffers are initialised with all zeros,
	* including the weights.
	* 
	* Direct analytic formulas are used to compute the line of best fit in this class. This is the most computationally
	* efficient option, but can lead to numerical issues. If more numerical stability is required, it is recommended
	* to use a thin/reduced QR decomposition to solve the associated linear least squares problem instead. That is,
	* if in Matlab the least squares problem is given by `Y = XB`, where Y contains the y data, X contains a column of
	* ones and the x data, and B contains the line parameters to solve for, then the solution is normally given by
	* `B = X\Y`, but to be even more numerically stable:
	* @code
	* [Q, R] = qr(X, 0)
	* B = R \ (Q'*Y)
	* @endcode
	**/
	class WLBFFilter
	{
	public:
		// Typedefs
		typedef boost::circular_buffer<double> Buffer;

		// Constructor
		explicit WLBFFilter(std::size_t len = 0)
		{
			// Initialise the data buffers
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
			m_a = m_b = m_xmean = 0.0;
			zeroBuf();
		}

		// Change the length of the data buffers
		void resize(std::size_t len)
		{
			// Update the data buffer lengths (Increase => Add zero data points, Decrease => Keep latest data points)
			m_xbuf.set_capacity(len);
			m_ybuf.set_capacity(len);
			m_wbuf.set_capacity(len);
			m_xbuf.resize(m_xbuf.capacity(), 0.0);
			m_ybuf.resize(m_ybuf.capacity(), 0.0);
			m_wbuf.resize(m_wbuf.capacity(), 0.0);
			m_len = len;
		}

		// Zero the data buffers
		void zeroBuf() { zeroXBuf(); zeroYBuf(); zeroWBuf(); }
		void zeroXBuf() { std::fill(m_xbuf.begin(), m_xbuf.end(), 0.0); }
		void zeroYBuf() { std::fill(m_ybuf.begin(), m_ybuf.end(), 0.0); }
		void zeroWBuf() { std::fill(m_wbuf.begin(), m_wbuf.end(), 0.0); }

		// Set unit weights in the weight buffer
		void setUnitW() { std::fill(m_wbuf.begin(), m_wbuf.end(), 1.0); }

		// Initialise the data buffers to a constant y, with x values up to and including xinit
		void initConstY(double xinit, double dx, double y)
		{
			// Add regularly spaced data points, all at the desired constant y
			for(std::size_t i = 0; i < m_xbuf.size(); i++)
				m_xbuf[i] = xinit - i*dx;
			std::fill(m_ybuf.begin(), m_ybuf.end(), y);
		}

		// Initialise the data buffers to a constant y and weight, with x values up to and including xinit
		void initConstYW(double xinit, double dx, double y, double w = 1.0)
		{
			// Add regularly spaced data points, all at the desired constant y and weight
			initConstY(xinit, dx, y);
			std::fill(m_wbuf.begin(), m_wbuf.end(), w*w); // Note: Input weights are squared!
		}

		// Add a weight to the weight buffer (shifts the association of weights to data points by one index)
		WLBFFilter& addW(double w)
		{
			// Add the required weight
			m_wbuf.push_front(w*w); // Note: Input weights are squared!
			return *this;
		}

		// Add a data point to the x and y buffers (shifts the association of weights to data points by one index)
		WLBFFilter& addY(double x, double y)
		{
			// Add the required data point
			m_xbuf.push_front(x);
			m_ybuf.push_front(y);
			return *this;
		}

		// Add a data point and weight to the data buffers
		WLBFFilter& addYW(double x, double y, double w = 1.0)
		{
			// Add the required data point and weight
			addY(x, y);
			addW(w);
			return *this;
		}

		// Modify a weight in the weight buffer (0 = Newest data, N-1 = Oldest data)
		void setW(std::size_t index, double w)
		{
			// Modify the required weight
			if(index < m_len)
				m_wbuf[index] = w*w; // Note: Input weights are squared!
		}

		// Modify a data point in the x and y buffers (0 = Newest data, N-1 = Oldest data)
		void setY(std::size_t index, double x, double y)
		{
			// Modify the required data point
			if(index < m_len)
			{
				m_xbuf[index] = x;
				m_ybuf[index] = y;
			}
		}

		// Modify a data point and weight in the data buffers (0 = Newest data, N-1 = Oldest data)
		void setYW(std::size_t index, double x, double y, double w = 1.0)
		{
			// Modify the required data point and weight
			setY(index, x, y);
			setW(index, w);
		}

		// Set the data in the x buffer (newest data first)
		template<typename Iterator> void setXBuf(Iterator first, Iterator last)
		{
			// Set the data in the buffer and fill in the rest with zeros
			std::size_t i = 0;
			for(; first != last && i < m_xbuf.size(); ++first, ++i)
				m_xbuf[i] = *first;
			for(; i < m_xbuf.size(); ++i)
				m_xbuf[i] = 0.0;
		}

		// Set the data in the y buffer (newest data first)
		template<typename Iterator> void setYBuf(Iterator first, Iterator last)
		{
			// Set the data in the buffer and fill in the rest with zeros
			std::size_t i = 0;
			for(; first != last && i < m_ybuf.size(); ++first, ++i)
				m_ybuf[i] = *first;
			for(; i < m_ybuf.size(); ++i)
				m_ybuf[i] = 0.0;
		}

		// Set the data in the w buffer (newest data first)
		template<typename Iterator> void setWBuf(Iterator first, Iterator last)
		{
			// Set the data in the buffer and fill in the rest with zeros
			std::size_t i = 0;
			for(; first != last && i < m_wbuf.size(); ++first, ++i)
				m_wbuf[i] = (*first) * (*first); // Note: Input weights are squared!
			for(; i < m_wbuf.size(); ++i)
				m_wbuf[i] = 0.0;
		}

		// Get data buffers
		const Buffer& XBuf() const { return m_xbuf; }
		const Buffer& YBuf() const { return m_ybuf; }
		const Buffer& WBuf() const { return m_wbuf; } // Note: The weights in the buffer are the squares of the weights that were provided!

		// Get filter properties
		std::size_t len() const { return m_len; }
		bool lenZero() const { return (m_len == 0); }

		// Get line parameters: y = A + B*(x - X)
		double getA() const { return m_a; }
		double getB() const { return m_b; }
		double getX() const { return m_xmean; }

		// Get filtered outputs
		double value() const { return (m_xbuf.empty() ? m_a : m_a + m_b*(m_xbuf.front() - m_xmean)); }
		double valueAt(double x) const { return m_a + m_b*(x - m_xmean); }
		double deriv() const { return m_b; }
		double centreValue() const { return m_a; }
		double centreX() const { return m_xmean; }

		// Update functions
		WLBFFilter& update() { calculateWLBF(); return *this; }
		double updatedValue() { calculateWLBF(); return value(); }
		double updatedDeriv() { calculateWLBF(); return deriv(); }

	private:
		// Calculate the weighted line of best fit from the data buffers
		void calculateWLBF()
		{
			// Calculate the weighted means of the x and y data
			double xmean = 0.0, ymean = 0.0, sumw = 0.0;
			for(std::size_t i = 0; i < m_len; i++)
			{
				double w = m_wbuf[i];
				xmean += w*m_xbuf[i];
				ymean += w*m_ybuf[i];
				sumw += w;
			}
			if(sumw == 0.0) // Should only happen if every weight is zero...
			{
				m_a = m_b = m_xmean = 0.0;
				return;
			}
			else
			{
				xmean /= sumw;
				ymean /= sumw;
			}

			// Precalculate terms
			double sumwx = 0.0, sumwy = 0.0, sumwxx = 0.0, sumwxy = 0.0;
			for(std::size_t i = 0; i < m_len; i++)
			{
				double x = m_xbuf[i] - xmean;
				double y = m_ybuf[i] - ymean;
				double w = m_wbuf[i];
				double wx = w*x;
				sumwx += wx;
				sumwy += w*y;
				sumwxx += wx*x;
				sumwxy += wx*y;
			}

			// Calculate the parameters of the weighted line of best fit
			double denom = sumw*sumwxx - sumwx*sumwx;
			if(denom == 0.0) // Should only happen if all x values with non-zero weight are identical...
			{
				m_a = ymean;
				m_b = 0.0;
			}
			else
			{
				m_a = (sumwy*sumwxx - sumwxy*sumwx) / denom + ymean;
				m_b = (sumw*sumwxy - sumwx*sumwy) / denom;
			}
			m_xmean = xmean;
		}

		// Line parameters: y = m_a + m_b*(x - m_xmean)
		double m_a;
		double m_b;
		double m_xmean;

		// Data buffers (0 = Newest data, N-1 = Oldest data)
		Buffer m_xbuf;
		Buffer m_ybuf;
		Buffer m_wbuf;
		std::size_t m_len;
	};
}

#endif
// EOF