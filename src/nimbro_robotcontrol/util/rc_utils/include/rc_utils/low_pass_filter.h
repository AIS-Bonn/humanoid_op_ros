// A simple first order low pass filter
// File: low_pass_filter.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

// Includes
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_arithmetic.hpp>
#include <cmath>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class LowPassFilterT
	* 
	* @brief A simple first order low pass filter.
	* 
	* The 90% settling time @c Ts is in the time units of cycles, where it is assumed that `put()`
	* is called exactly once per cycle to add the latest input data point to the filter.
	**/
	template<class ValueType> class LowPassFilterT
	{
	public:
		//! Constructor
		explicit LowPassFilterT(double Ts = 100.0)
		{
			// Reset everything
			resetAll(Ts);
		}
		
		//! Reset function (also resets the configured 90% settling time)
		void resetAll(double Ts = 100.0)
		{
			// Reset everything
			reset();
			setTs(Ts);
		}
		
		//! Reset function (resets everything other than the configured 90% settling time)
		void reset()
		{
			// Reset all data members
			m_value = zero(ValueType());
			m_freeze = false;
		}
		
		//! Set the 90% settling time to use (in units of cycles, where each call to `put()` is a cycle)
		void setTs(double Ts)
		{
			// Update the settling time
			if(!std::isfinite(Ts))
			{
				m_Ts = INFINITY;
				m_alpha = 0.0;
			}
			else if(Ts <= 0.0)
			{
				m_Ts = 0.0;
				m_alpha = 1.0;
			}
			else
			{
				m_Ts = Ts;
				m_alpha = 1.0 - pow(0.10, 1.0/Ts); // The 0.10 comes from Ts being a 90% settling time
			}
		}
		
		//! Retrieve the 90% settling time currently in use
		double getTs() const { return m_Ts; }
		
		//! Retrieve the current update factor in use
		double getAlpha() const { return m_alpha; }
		
		//! Manually set the current filter value
		void setValue(const ValueType& value) { m_value = value; }
		
		//! Retrieve the current filter value
		ValueType value() const { return m_value; }
		
		//! Freeze the current filter value, disabling modification thereof by the putting of new data
		void freeze() { m_freeze = true; }
		
		//! Unfreeze the current filter value, enabling modification thereof by the putting of new data
		void unfreeze() { m_freeze = false; }
		
		//! Freeze or unfreeze the filter as required
		void setFrozen(bool freeze) { m_freeze = freeze; }
		
		//! Get the current freeze status
		bool isFrozen() const { return m_freeze; }
		
		//! Update the low pass filter with a new input data value
		void put(const ValueType& value)
		{
			// Update the low pass filter
			if(!m_freeze)
				m_value += m_alpha*(value - m_value);
		}
		
		//! Static function to compute an alpha value from a 90% settling time @p Ts based on a nominal time step @p dT
		static double computeAlpha(double Ts, double dT)
		{
			// Compute the required value of alpha
			return 1.0 - pow(0.10, dT/Ts); // The 0.10 comes from Ts being a 90% settling time
		}
		
	protected:
		// Protected internal variables
		ValueType m_value; // Current filter value
		double m_alpha;    // Current update factor corresponding to the filter settling time
		bool m_freeze;     // Flag whether updates to the filter are inhibited or not
		
	private:
		// Private internal variables
		double m_Ts;       // Current filter settling time (90%)
		
		// Value initialisation magic
		template<class T> typename boost::enable_if<boost::is_arithmetic<T>, T>::type zero(const T& dummy) { return 0; } // Note: If you're using Eigen as the type of this filter and get an error where you see this comment, then have you included Eigen *before* you included this header the first time?
#ifdef EIGEN_CORE_H
		template<class T, int Rows, int Cols> Eigen::Matrix<T, Rows, Cols> zero(const Eigen::Matrix<T, Rows, Cols>& dummy) { return Eigen::Matrix<T, Rows, Cols>::Zero(); }
#endif
	};
	
	//! Standard low pass filter operating on the double type
	typedef LowPassFilterT<double> LowPassFilter;
}

#endif
// EOF