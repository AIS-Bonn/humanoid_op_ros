// Implements class to compute a smooth constant acceleration transition between a half-sine wave and zero
// File: lin_sin_fillet.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef LIN_SIN_FILLET_H
#define LIN_SIN_FILLET_H

// Includes
#include <cmath>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class LinSinFillet
	* 
	* @brief Compute an offset to add in order to add a fillet between a sine wave hemisphere and zero.
	* 
	* It is assumed the sine wave is A*sin(B*t). The fillet is added from constant zero to the sine wave,
	* with constant acceleration, continuous positions and velocities, and merging into the sine wave at
	* time t = T.
	**/
	class LinSinFillet
	{
	public:
		// Constructor
		LinSinFillet() { reset(); }
		LinSinFillet(double A, double B, double T) { update(A, B, T); }
		LinSinFillet(double A, double B, double T, double maxPre) { updateExact(A, B, T, maxPre); }
		
		// Reset function
		void reset() { m_A = m_B = m_a = m_b = m_c = m_tPre = m_tPost = 0.0; }
		
		// Update function
		bool update(double A, double B, double T)
		{
			double BT = B*T;
			if(T < 1e-6 || fabs(B) < 1e-6 || fabs(BT) > M_PI_2 - 1e-6)
			{
				reset();
				return false;
			}
			m_A = A;
			m_B = B;
			if(fabs(A) < 1e-6)
			{
				m_a = m_b = m_c = 0.0;
				m_tPre = m_tPost = T;
				return true;
			}
			m_c = m_A*sin(BT);
			m_b = m_A*m_B*cos(BT);
			m_a = m_b*m_b/(4*m_c);
			m_tPre = T - 2*m_c/m_b;
			m_tPost = T;
			if(m_tPre > m_tPost) m_tPre = m_tPost; // Should never happen
			return true;
		}
		
		// Update functions with maximum allowed pre-time
		bool update(double A, double B, double T, double maxPre) // maxPre should be non-negative and be the largest acceptable pre-time (time from start of fillet to the mid-fillet lin-sin transition)
		{
			if(!update(A, B, T)) return false; // Calculate the fillet that results from using time T
			if(maxPre < 1e-6)
			{
				reset(); // Disable the fillet if maxPre is too small
				return true;
			}
			if(m_tPre >= -maxPre) return true;
			return update(A, B, T*(maxPre/-m_tPre)); // Reduce T to (conservatively) abide by the maxPre specification
		}
		bool updateExact(double A, double B, double T, double maxPre)
		{
			if(!update(A, B, T)) return false; // Calculate the fillet that results from using time T
			if(maxPre < 1e-6)
			{
				reset(); // Disable the fillet if maxPre is too small
				return true;
			}
			if(m_tPre >= -maxPre) return true;
			double newT = T;
			int count = 0;
			while(true)
			{
				if(fabs(m_tPre + maxPre) < 1e-6) return true;
				if(fabs(B*newT) > M_PI_2 - 1e-6) break;
				double secBT = 1 / cos(B*newT);
				newT -= (m_tPre + maxPre) / (1 - 2*secBT*secBT);
				if(!update(A, B, newT)) break;
				if(++count >= 20) break;
			}
			return update(A, B, T, maxPre);
		}

		// Return whether this fillet needs to be applied
		bool hasEffect(double t) const { return (t > m_tPre && t < m_tPost); }

		// Evaluate function (evaluates just the required waveform to add to get the fillet)
		double eval(double t) const
		{
			if(!hasEffect(t)) return 0.0;
			double dt = t - m_tPost;
			double parabola = m_c + dt*(m_b + dt*m_a);
			if(t <= 0.0) return parabola;
			else return parabola - m_A*sin(m_B*t);
		}
		
		// Static evaluation function
		static double eval(double t, double A, double B, double T) { LinSinFillet LSF(A, B, T); return LSF.eval(t); }
		static double eval(double t, double A, double B, double T, double maxPre) { LinSinFillet LSF(A, B, T, maxPre); return LSF.eval(t); }
		
		// Get functions
		double getA() const { return m_A; }
		double getB() const { return m_B; }
		double startTime() const { return m_tPre; } // For small fillets this is approx -T (and is always < -T)
		double endTime() const { return m_tPost; }  // For successful fillets this is always T
		
	private:
		// Members
		double m_A;
		double m_B;
		double m_a;
		double m_b;
		double m_c;
		double m_tPre;
		double m_tPost;
	};
}

#endif
// EOF