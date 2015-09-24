// Utilities for simple math functions
// File: math_funcs.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef MATH_FUNCS_H
#define MATH_FUNCS_H

// Includes
#include <cmath>

// Defines
#define M_2PI (2.0*M_PI)

// Nimbro utilities namespace
namespace nimbro_utils
{
	/**
	* @name Math Functions (nimbro_utils/math_funcs.h)
	**/
	///@{

	//! @brief Wrap an angle to (-pi,pi]
	inline double picut(double angle)
	{
		// Return the required result
		return angle + M_2PI*std::floor((M_PI - angle)/M_2PI);
	}

	//! @brief Return the sign of a value (1 or -1)
	inline int sign(double x)
	{
		// Return -1 or 1 as appropriate
		if(x >= 0.0) return  1;
		else         return -1;
	}

	//! @brief Return the sign of a value (1, 0 or -1)
	inline int sign0(double x)
	{
		// Return -1, 0 or 1 as appropriate
		if     (x >  0.0) return  1;
		else if(x == 0.0) return  0;
		else              return -1;
	}

	//! @brief Return the sign of a value (1 or -1)
	inline int sign(int x)
	{
		// Return -1 or 1 as appropriate
		if(x >= 0) return  1;
		else       return -1;
	}

	//! @brief Return the sign of a value (1, 0 or -1)
	inline int sign0(int x)
	{
		// Return -1, 0 or 1 as appropriate
		if     (x >  0) return  1;
		else if(x == 0) return  0;
		else            return -1;
	}

	//! @brief Coerce the value @p x to be in the range `[min,max]`
	inline double coerce(double x, double min, double max) // Note: The max constraint is considered before the min constraint, in case they conflict (min > max)
	{
		// Coerce the value x to be in the range [min,max]
		if     (x >= max) return max;
		else if(x <= min) return min;
		else              return x;
	}

	//! @brief Coerce the value @p x to be in the range `[-maxAbs,maxAbs]`
	inline double coerceAbs(double x, double maxAbs)
	{
		// Coerce the value x to be in the range [-maxAbs,maxAbs]
		if     (x >=  maxAbs) return  maxAbs;
		else if(x <= -maxAbs) return -maxAbs;
		else                  return  x;
	}

	//! @brief Coerce the value @p x to be in the range `(-&infin;,max]`
	inline double coerceMax(double x, double max)
	{
		// Coerce the value x to (-Inf,max]
		return (x >= max ? max : x);
	}

	//! @brief Coerce the value @p x to be in the range `[min,&infin;)`
	inline double coerceMin(double x, double min)
	{
		// Coerce the value x to [min,Inf)
		return (x <= min ? min : x);
	}
	
	//! @brief Coerce the value @p x to be in the range `[min,max]` in a soft manner, rounding off exponentially within the soft range given by within @p buffer of the limits
	inline double coerceSoft(double x, double min, double max, double buffer)
	{
		// Error checking on the buffer range
		double maxBuf = 0.5*(max - min);
		if(buffer > maxBuf) buffer = maxBuf;
		if(buffer <= 0.0) return coerce(x, min, max);
		
		// Soft-coerce the value x as required
		if(x > max - buffer)
			return max - buffer*std::exp(-(x - (max - buffer)) / buffer);
		if(x < min + buffer)
			return min + buffer*std::exp((x - (min + buffer)) / buffer);
		return x;
	}

	//! @brief Coerce the value @p x to be in the range `[-max_abs,max_abs]` in a soft manner, rounding off exponentially within the soft range given by within @p buffer of the limits
	inline double coerceSoftAbs(double x, double maxAbs, double buffer)
	{
		// Error checking on the buffer range
		if(buffer > maxAbs) buffer = maxAbs;
		if(buffer <= 0.0) return coerceAbs(x, maxAbs);
		
		// Soft-coerce the value x as required
		double softLim = maxAbs - buffer;
		if(x > softLim)
			return maxAbs - buffer*std::exp(-(x - softLim) / buffer);
		if(x < -softLim)
			return -maxAbs + buffer*std::exp((x + softLim) / buffer);
		return x;
	}

	//! @brief Coerce the value @p x to be in the range `(-&infin;,max]` in a soft manner, rounding off exponentially within the soft range given by within @p buffer of the limits
	inline double coerceSoftMax(double x, double max, double buffer)
	{
		// Soft-coerce the value x as required
		double softLim = max - buffer;
		if(x > softLim)
			return max - buffer*std::exp(-(x - softLim) / buffer);
		return x;
	}

	//! @brief Coerce the value @p x to be in the range `[min,&infin;)` in a soft manner, rounding off exponentially within the soft range given by within @p buffer of the limits
	inline double coerceSoftMin(double x, double min, double buffer)
	{
		// Soft-coerce the value x as required
		double softLim = min + buffer;
		if(x < softLim)
			return min + buffer*std::exp((x - softLim) / buffer);
		return x;
	}
	///@}
}

#endif /* MATH_FUNCS_H */
// EOF