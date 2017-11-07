// Implements a class that adds smooth constant acceleration deadband to a signal.
// File: smooth_deadband.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef SMOOTH_DEADBAND_H
#define SMOOTH_DEADBAND_H

// Includes
#include <rc_utils/math_funcs.h>
#include <cmath>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class SharpDeadband
	* 
	* @brief Add deadband by the usual (sharp) definition to a signal.
	**/
	class SharpDeadband
	{
	public:
		// Constructor
		SharpDeadband() { reset(); }
		explicit SharpDeadband(double radius, double centre = 0.0) : m_radius(radius), m_centre(centre) {}
		
		// Reset function
		void reset() { m_radius = m_centre = 0.0; }
		
		// Set function
		void set(double radius, double centre = 0.0) { m_radius = radius; m_centre = centre; }
		
		// Get functions
		double radius() const { return m_radius; }
		double centre() const { return m_centre; }

		// Evaluate functions
		double eval(double x) const { return eval(x, m_radius, m_centre); }
		static double eval(double x, double radius, double centre = 0.0)
		{
			double relx = x - centre;
			double relxsign = sign(relx);
			if(fabs(relx) >= radius)
				return relx - relxsign*radius;
			else
				return 0.0;
		}
		
	private:
		// Data members
		double m_radius;
		double m_centre;
	};

	/**
	* @class SmoothDeadband
	* 
	* @brief Add deadband that is continuous in the first derivative to a signal.
	* 
	* At the centre the throughput is 0, at @p radius from the centre the throughput is `0.25*radius`,
	* and at twice @p radius from the centre the throughput is `radius` and rises with unit slope beyond that.
	**/
	class SmoothDeadband
	{
	public:
		// Constructor
		SmoothDeadband() { reset(); }
		explicit SmoothDeadband(double radius, double centre = 0.0) : m_radius(radius), m_centre(centre) {}
		
		// Reset function
		void reset() { m_radius = m_centre = 0.0; }
		
		// Set function
		void set(double radius, double centre = 0.0) { m_radius = radius; m_centre = centre; }
		
		// Get functions
		double radius() const { return m_radius; }
		double centre() const { return m_centre; }

		// Evaluate functions
		double eval(double x) const { return eval(x, m_radius, m_centre); }
		static double eval(double x, double radius, double centre = 0.0)
		{
			double relx = x - centre;
			double relxsign = sign(relx);
			if(fabs(relx) >= 2.0*radius)
				return relx - relxsign*radius;
			else
				return relxsign*relx*relx / (4.0*radius);
		}
		
	private:
		// Data members
		double m_radius;
		double m_centre;
	};

	/**
	* @class DeadSmoothDeadband
	* 
	* @brief Add deadband that is continuous in the first derivative, and with non-zero dead zone, to a signal.
	* 
	* At the centre and up to @p zeroRadius either side of it the throughput is 0. @p deadRadius beyond that
	* the throughput is `0.25*deadRadius`, and @p deadRadius further beyond that the throughput is `deadRadius`
	* and rises linearly with unit slope beyond that.
	**/
	class DeadSmoothDeadband
	{
	public:
		// Constructor
		DeadSmoothDeadband() { reset(); }
		explicit DeadSmoothDeadband(double zeroRadius, double deadRadius, double centre = 0.0) : m_zeroRadius(zeroRadius), m_deadRadius(deadRadius), m_centre(centre) {}
		
		// Reset function
		void reset() { m_zeroRadius = m_deadRadius = m_centre = 0.0; }
		
		// Set function
		void set(double zeroRadius, double deadRadius, double centre = 0.0) { m_zeroRadius = zeroRadius; m_deadRadius = deadRadius; m_centre = centre; }
		
		// Get functions
		double zeroRadius() const { return m_zeroRadius; }
		double deadRadius() const { return m_deadRadius; }
		double centre() const { return m_centre; }

		// Evaluate functions
		double eval(double x) const { return eval(x, m_zeroRadius, m_deadRadius, m_centre); }
		static double eval(double x, double zeroRadius, double deadRadius, double centre = 0.0)
		{
			double relx = x - centre;
			if(fabs(relx) <= zeroRadius)
				return 0.0;
			double relxsign = sign(relx);
			double relxdead = relx - relxsign*zeroRadius;
			if(fabs(relxdead) >= 2.0*deadRadius)
				return relxdead - relxsign*deadRadius;
			else
				return relxsign*relxdead*relxdead / (4.0*deadRadius);
		}
		
	private:
		// Data members
		double m_zeroRadius;
		double m_deadRadius;
		double m_centre;
	};
}

#endif
// EOF