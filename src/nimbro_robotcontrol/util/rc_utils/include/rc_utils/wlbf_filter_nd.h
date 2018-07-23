// N-dimensional weighted line of best fit filter
// File: wlbf_filter_nd.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef WLBF_FILTER_ND_H
#define WLBF_FILTER_ND_H

// Includes
#include <boost/circular_buffer.hpp>
#include <type_traits>
#include <Eigen/Core>
#include <cstddef>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class WLBFFilterND
	* 
	* @brief N-dimensional weighted line of best fit filter that smooths and calculates the first order derivative.
	* 
	* The supplied weights are effectively squared, so adding a data point with weight w is equivalent to adding the
	* same data point w^2 times with weight 1. As such, negative weights are effectively treated by absolute value.
	* 
	* Unless the buffers are otherwise manually initialised, by default the buffers are initialised with all zeros,
	* including the weights.
	* 
	* Direct analytic formulas are used to compute the line of best fit in this class. This is the most computationally
	* efficient option, but can lead to numerical issues. If more numerical stability is required, it is recommended
	* to use a thin/reduced QR decomposition to solve the associated linear least squares problem instead. That is, if
	* in Matlab the least squares problem is given by `P = TB`, where P contains the point data, T contains a column of
	* ones and the time data, and B contains the line parameters to solve for, then the solution is normally given by
	* `B = T\P`, but to be even more numerically stable:
	* @code
	* [Q, R] = qr(T, 0)
	* B = R \ (Q'*P)
	* @endcode
	**/
	template<int N> class WLBFFilterND
	{
	public:
		// Filter dimension
		static_assert(N >= 1, "The dimension N of a WLBFFilterND must be a positive integer!");
		static const int ND;

		// Typedefs
		typedef typename std::conditional<N == 1, double, Eigen::Matrix<double, N, 1>>::type Vec;
		typedef typename std::conditional<N == 1, boost::circular_buffer<Vec>, boost::circular_buffer<Vec, Eigen::aligned_allocator<Vec>>>::type VecBuffer;
		typedef boost::circular_buffer<double> ScalarBuffer;

		// Constructor
		explicit WLBFFilterND(std::size_t len = 0)
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
			m_a = m_b = ZeroVec;
			m_tmean = 0.0;
			zeroBuf();
		}

		// Change the length of the data buffers
		void resize(std::size_t len)
		{
			// Update the data buffer lengths (Increase => Add zero data points, Decrease => Keep latest data points)
			m_tbuf.set_capacity(len);
			m_pbuf.set_capacity(len);
			m_wbuf.set_capacity(len);
			m_tbuf.resize(m_tbuf.capacity(), 0.0);
			m_pbuf.resize(m_pbuf.capacity(), ZeroVec);
			m_wbuf.resize(m_wbuf.capacity(), 0.0);
			m_len = len;
		}

		// Zero the data buffers
		void zeroBuf() { zeroTBuf(); zeroPBuf(); zeroWBuf(); }
		void zeroTBuf() { std::fill(m_tbuf.begin(), m_tbuf.end(), 0.0); }
		void zeroPBuf() { std::fill(m_pbuf.begin(), m_pbuf.end(), ZeroVec); }
		void zeroWBuf() { std::fill(m_wbuf.begin(), m_wbuf.end(), 0.0); }

		// Set unit weights in the weight buffer
		void setUnitW() { std::fill(m_wbuf.begin(), m_wbuf.end(), 1.0); }

		// Initialise the data buffers to a constant point, with times up to and including tinit
		void initConstP(double tinit, double dt, const Vec& p)
		{
			// Add regularly spaced data points, all at the desired constant point
			for(std::size_t i = 0; i < m_tbuf.size(); i++)
				m_tbuf[i] = tinit - i*dt;
			std::fill(m_pbuf.begin(), m_pbuf.end(), p);
		}

		// Initialise the data buffers to a constant point and weight, with times up to and including tinit
		void initConstPW(double tinit, double dt, const Vec& p, double w = 1.0)
		{
			// Add regularly spaced data points, all at the desired constant point and weight
			initConstP(tinit, dt, p);
			std::fill(m_wbuf.begin(), m_wbuf.end(), w*w); // Note: Input weights are squared!
		}

		// Add a weight to the weight buffer (shifts the association of weights to data points by one index)
		WLBFFilterND<N>& addW(double w)
		{
			// Add the required weight
			m_wbuf.push_front(w*w); // Note: Input weights are squared!
			return *this;
		}

		// Add a data point to the time and point buffers (shifts the association of weights to data points by one index)
		template<int M = N> typename std::enable_if<M == 2 && N == 2, WLBFFilterND<N>&>::type addP(double t, double x, double y) { return addP(t, Vec(x, y)); }
		template<int M = N> typename std::enable_if<M == 3 && N == 3, WLBFFilterND<N>&>::type addP(double t, double x, double y, double z) { return addP(t, Vec(x, y, z)); }
		WLBFFilterND<N>& addP(double t, const Vec& p)
		{
			// Add the required data point
			m_tbuf.push_front(t);
			m_pbuf.push_front(p);
			return *this;
		}

		// Add a data point and weight to the data buffers
		template<int M = N> typename std::enable_if<M == 2 && N == 2, WLBFFilterND<N>&>::type addPW(double t, double x, double y, double w = 1.0) { return addPW(t, Vec(x, y)); }
		template<int M = N> typename std::enable_if<M == 3 && N == 3, WLBFFilterND<N>&>::type addPW(double t, double x, double y, double z, double w = 1.0) { return addPW(t, Vec(x, y, z)); }
		WLBFFilterND<N>& addPW(double t, const Vec& p, double w = 1.0)
		{
			// Add the required data point and weight
			addP(t, p);
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

		// Modify a data point in the time and point buffers (0 = Newest data, N-1 = Oldest data)
		void setP(std::size_t index, double t, const Vec& p)
		{
			// Modify the required data point
			if(index < m_len)
			{
				m_tbuf[index] = t;
				m_pbuf[index] = p;
			}
		}

		// Modify a data point and weight in the data buffers (0 = Newest data, N-1 = Oldest data)
		void setPW(std::size_t index, double t, const Vec& p, double w = 1.0)
		{
			// Modify the required data point and weight
			setP(index, t, p);
			setW(index, w);
		}

		// Set the data in the time buffer (newest data first)
		template<typename Iterator> void setTBuf(Iterator first, Iterator last)
		{
			// Set the data in the buffer and fill in the rest with zeros
			std::size_t i = 0;
			for(; first != last && i < m_tbuf.size(); ++first, ++i)
				m_tbuf[i] = *first;
			for(; i < m_tbuf.size(); ++i)
				m_tbuf[i] = 0.0;
		}

		// Set the data in the point buffer (newest data first)
		template<typename Iterator> void setPBuf(Iterator first, Iterator last)
		{
			// Set the data in the buffer and fill in the rest with zeros
			std::size_t i = 0;
			for(; first != last && i < m_pbuf.size(); ++first, ++i)
				m_pbuf[i] = *first;
			for(; i < m_pbuf.size(); ++i)
				m_pbuf[i] = ZeroVec;
		}

		// Set the data in the weight buffer (newest data first)
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
		const ScalarBuffer& TBuf() const { return m_tbuf; }
		const VecBuffer& PBuf() const { return m_pbuf; }
		const ScalarBuffer& WBuf() const { return m_wbuf; } // Note: The weights in the buffer are the squares of the weights that were provided!

		// Get filter properties
		std::size_t len() const { return m_len; }
		bool lenZero() const { return (m_len == 0); }

		// Get line parameters: p = A + B*(t - T)
		Vec getA() const { return m_a; }
		Vec getB() const { return m_b; }
		double getT() const { return m_tmean; }

		// Get filtered outputs
		Vec value() const { return (m_tbuf.empty() ? m_a : m_a + m_b*(m_tbuf.front() - m_tmean)); }
		Vec valueAt(double t) const { return m_a + m_b*(t - m_tmean); }
		Vec deriv() const { return m_b; }
		Vec centreValue() const { return m_a; }
		double centreTime() const { return m_tmean; }

		// Update functions
		WLBFFilterND<N>& update() { calculateWLBF(); return *this; }
		Vec updatedValue() { calculateWLBF(); return value(); }
		Vec updatedDeriv() { calculateWLBF(); return deriv(); }

		// Zero vector constant
		static Vec getZeroVec() { return Vec::Zero(); }
		const Vec ZeroVec = getZeroVec();

	private:
		// Calculate the weighted line of best fit from the data buffers
		void calculateWLBF()
		{
			// Calculate the weighted means of the time and point data
			double tmean = 0.0, sumw = 0.0;
			Vec pmean = ZeroVec;
			for(std::size_t i = 0; i < m_len; i++)
			{
				double w = m_wbuf[i];
				tmean += w*m_tbuf[i];
				pmean += w*m_pbuf[i];
				sumw += w;
			}
			if(sumw == 0.0) // Should only happen if every weight is zero...
			{
				m_a = m_b = ZeroVec;
				m_tmean = 0.0;
				return;
			}
			else
			{
				tmean /= sumw;
				pmean /= sumw;
			}

			// Precalculate terms
			double sumwt = 0.0, sumwtt = 0.0;
			Vec sumwp = ZeroVec, sumwtp = ZeroVec;
			for(std::size_t i = 0; i < m_len; i++)
			{
				double t = m_tbuf[i] - tmean;
				Vec p = m_pbuf[i] - pmean;
				double w = m_wbuf[i];
				double wt = w*t;
				sumwt += wt;
				sumwp += w*p;
				sumwtt += wt*t;
				sumwtp += wt*p;
			}

			// Calculate the parameters of the weighted line of best fit
			double denom = sumw*sumwtt - sumwt*sumwt;
			if(denom == 0.0) // Should only happen if all time values with non-zero weight are identical...
			{
				m_a = pmean;
				m_b = ZeroVec;
			}
			else
			{
				m_a = (sumwp*sumwtt - sumwtp*sumwt) / denom + pmean;
				m_b = (sumw*sumwtp - sumwt*sumwp) / denom;
			}
			m_tmean = tmean;
		}

		// Line parameters: p = m_a + m_b*(t - m_tmean)
		Vec m_a;
		Vec m_b;
		double m_tmean;

		// Data buffers (0 = Newest data, N-1 = Oldest data)
		ScalarBuffer m_tbuf;
		VecBuffer m_pbuf;
		ScalarBuffer m_wbuf;
		std::size_t m_len;
	};

	// WLBFFilterND constants
	template<int N> const int WLBFFilterND<N>::ND = N;

	// Zero vector constant
	template<> inline WLBFFilterND<1>::Vec WLBFFilterND<1>::getZeroVec() { return 0.0; }

	// Typedefs
	typedef WLBFFilterND<1> WLBFFilter1D;
	typedef WLBFFilterND<2> WLBFFilter2D;
	typedef WLBFFilterND<3> WLBFFilter3D;
	typedef WLBFFilterND<4> WLBFFilter4D;
}

#endif
// EOF