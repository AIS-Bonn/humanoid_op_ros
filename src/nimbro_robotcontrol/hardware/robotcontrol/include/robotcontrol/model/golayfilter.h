// Smoothing and first order derivative filter based on the Savitzky-Golay class of filters
// File: golayfilter.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef GOLAYFILTER_H
#define GOLAYFILTER_H

// Includes
#include <robotcontrol/model/golay.h>

// Robotcontrol namespace
namespace robotcontrol
{
	/**
	* @class GolayFilter
	*
	* @brief Filter that smooths data and calculates its first order derivative, based on the Savitzky-Golay class of filters
	**/
	template<int WindowSize = 9>
	class GolayFilter
	{
	public:
		//! Default constructor
		explicit GolayFilter(double dt = 1.0, double x = 0.0, double v = 0.0)
		{
			// Reset the filter
			reset(dt, x, v);
		}

		//! Reset function
		void reset(double dt = 1.0, double x = 0.0, double v = 0.0)
		{
			// Reset the internal golay filters
			m_smooth_x.reset();
			m_smooth_v.reset();

			// Set the internal time step
			m_dt = (dt > 0.0 ? dt : 1.0);

			// Generate fake past data to give the required initial conditions
			for(int i = 0; i < WindowSize; i++)
			{
				double fakex = x - (WindowSize - i - 1)*v*m_dt;
				m_smooth_x.put(fakex);
				m_smooth_v.put(fakex);
			}

			// Update the cached x and v
			updateValues();
		}

		//! Update the Golay filter with a new measurement
		void update(double xnew)
		{
			// Update the internal golay filters
			m_smooth_x.put(xnew);
			m_smooth_v.put(xnew);

			// Update the cached x and v
			updateValues();
		}

		//! Shift the current data points (and thereby also the smooth `x`) by a given increment @p shift, while leaving the velocity untouched
		void shiftPosBy(double shift)
		{
			// Retrieve the golay derivative buffers
			typename GolayDerivative<double, 0, WindowSize>::BufferType* pxBuf = m_smooth_x.buffer();
			typename GolayDerivative<double, 1, WindowSize>::BufferType* pvBuf = m_smooth_v.buffer();

			// Shift the elements in the golay filter buffers by the given amount
			for(int i = 0; i < WindowSize; i++)
			{
				m_smooth_x.put(pxBuf->front() + shift); // Note: front() is the oldest entry in the buffer
				m_smooth_v.put(pvBuf->front() + shift);
			}

			// Update the cached x and v
			updateValues();
		}

		//! Shift the current data points (and thereby also the smooth `x`) by an increment calculated such that the last passed data point is now @p xnew, while leaving the velocity untouched
		void shiftPosTo(double xnew) { shiftPosBy(xnew - m_smooth_x.buffer()->back()); } // Note: back() is the newest entry in the buffer

		//! Retrieve the time increment in use
		double dt() const { return m_dt; }

		//! Retrieve the current smoothed `x` value
		double x() const { return m_x; }

		//! Retrieve the current smoothed `v` value
		double v() const { return m_v; }

	private:
		// Update the cached x and v values
		void updateValues()
		{
			// Recalculate the x and v values
			m_x = m_smooth_x.value();
			m_v = m_smooth_v.value() / m_dt;
		}
		
		// Golay filters
		GolayDerivative<double, 0, WindowSize> m_smooth_x; // Note: The internal buffers of these two variables are
		GolayDerivative<double, 1, WindowSize> m_smooth_v; //       always kept identical, for obvious reasons.

		// Time step (constant time separation of golay data points)
		double m_dt;

		// Cached x and v values
		double m_x;
		double m_v;
	};
}

#endif /* GOLAYFILTER_H */
// EOF