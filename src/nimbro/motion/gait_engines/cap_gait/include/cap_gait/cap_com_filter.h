// Centre of mass (CoM) filter for the capture step gait
// File: cap_com_filter.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef CAP_COM_FILTER_H
#define CAP_COM_FILTER_H

// Includes
#include <robotcontrol/model/golayfilter.h>

// Capture step gait namespace
namespace cap_gait
{
	/**
	* @class ComFilter
	*
	* @brief Filters a 2D CoM position, and calculates and smooths an estimate of the CoM velocity.
	**/
	template<int WindowSize = 5>
	class ComFilter
	{
	public:
		//! Default constructor
		explicit ComFilter(double dt = 1.0, double x = 0.0, double y = 0.0, double vx = 0.0, double vy = 0.0)
		{
			// Reset the filter
			reset(dt, x, vx, y, vy);
		}

		//! Reset the CoM filter completely to a particular state
		void reset(double dt = 1.0, double x = 0.0, double y = 0.0, double vx = 0.0, double vy = 0.0)
		{
			// Reset the internal golay filters
			m_filter_x.reset(dt, x, vx);
			m_filter_y.reset(dt, y, vy);
		}

		//! Update the filter with a new measured CoM position
		void update(double xnew, double ynew)
		{
			// Update the internal golay filters
			m_filter_x.update(xnew);
			m_filter_y.update(ynew);
		}

		//! Shift the CoM position estimate by a given increment, leaving the velocity untouched (refer to `GolayFilter::shiftPosBy`)
		void shiftPosBy(double deltax, double deltay)
		{
			// Shift the internal golay filters as required
			m_filter_x.shiftPosBy(deltax);
			m_filter_y.shiftPosBy(deltay);
		}

		//! Shift the CoM position estimate to a given value, leaving the velocity untouched (refer to `GolayFilter::shiftPosTo`)
		void shiftPosTo(double xnew, double ynew)
		{
			// Shift the internal golay filters as required
			m_filter_x.shiftPosTo(xnew);
			m_filter_y.shiftPosTo(ynew);
		}

		// Get functions
		double dt() const { return m_filter_x.dt(); }
		double x() const { return m_filter_x.x(); }
		double y() const { return m_filter_y.x(); }
		double vx() const { return m_filter_x.v(); }
		double vy() const { return m_filter_y.v(); }
		void get(double& x, double& y, double& vx, double& vy) const
		{
			// Retrieve the current CoM filter state
			x = m_filter_x.x();
			y = m_filter_y.x();
			vx = m_filter_x.v();
			vy = m_filter_y.v();
		}

	private:
		// Internal golay filters
		robotcontrol::GolayFilter<WindowSize> m_filter_x;
		robotcontrol::GolayFilter<WindowSize> m_filter_y;
	};
}

#endif /* CAP_COM_FILTER_H */
// EOF