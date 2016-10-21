// Utilities for simple math functions
// File: math_funcs.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef MATH_FUNCS_H
#define MATH_FUNCS_H

// Includes
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_arithmetic.hpp>
#include <boost/type_traits/is_floating_point.hpp>
#include <cmath>

// Defines
#define M_2PI (2.0*M_PI)

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @name Math Functions (rc_utils/math_funcs.h)
	**/
	///@{

	//! @brief Wrap an angle to (-pi,pi]
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type picut(Scalar angle)
	{
		// Return the required result
		return angle + M_2PI*std::floor((M_PI - angle)/M_2PI);
	}

	//! @brief Return the sign of a value (1 or -1)
	template<typename Scalar> typename boost::enable_if<boost::is_arithmetic<Scalar>, int>::type sign(Scalar x)
	{
		// Return -1 or 1 as appropriate
		if(x >= 0) return  1;
		else       return -1;
	}

	//! @brief Return the sign of a value (1, 0 or -1)
	template<typename Scalar> typename boost::enable_if<boost::is_arithmetic<Scalar>, int>::type sign0(Scalar x)
	{
		// Return -1, 0 or 1 as appropriate
		if     (x >  0) return  1;
		else if(x == 0) return  0;
		else            return -1;
	}

	//! @brief Coerce the value @p x to be in the range `[min,max]`
	template<typename Scalar> typename boost::enable_if<boost::is_arithmetic<Scalar>, Scalar>::type coerce(Scalar x, Scalar min, Scalar max) // Note: The max constraint is considered before the min constraint, in case they conflict (min > max)
	{
		// Coerce the value x to be in the range [min,max]
		if     (x >= max) return max;
		else if(x <= min) return min;
		else              return x;
	}

	//! @brief Coerce the value @p x to be in the range `[-maxAbs,maxAbs]`
	template<typename Scalar> typename boost::enable_if<boost::is_arithmetic<Scalar>, Scalar>::type coerceAbs(Scalar x, Scalar maxAbs)
	{
		// Coerce the value x to be in the range [-maxAbs,maxAbs]
		if     (x >=  maxAbs) return  maxAbs;
		else if(x <= -maxAbs) return -maxAbs;
		else                  return  x;
	}

	//! @brief Coerce the value @p x to be in the range `(-&infin;,max]`
	template<typename Scalar> typename boost::enable_if<boost::is_arithmetic<Scalar>, Scalar>::type coerceMax(Scalar x, Scalar max)
	{
		// Coerce the value x to (-Inf,max]
		return (x >= max ? max : x);
	}

	//! @brief Coerce the value @p x to be in the range `[min,&infin;)`
	template<typename Scalar> typename boost::enable_if<boost::is_arithmetic<Scalar>, Scalar>::type coerceMin(Scalar x, Scalar min)
	{
		// Coerce the value x to [min,Inf)
		return (x <= min ? min : x);
	}
	
	//! @brief Coerce the value @p x to be in the range `[min,max]` in a soft manner, rounding off exponentially within the soft range given by within @p buffer of the limits
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceSoft(Scalar x, Scalar min, Scalar max, Scalar buffer)
	{
		// Error checking on the buffer range
		Scalar maxBuf = 0.5*(max - min);
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
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceSoftAbs(Scalar x, Scalar maxAbs, Scalar buffer)
	{
		// Error checking on the buffer range
		if(buffer > maxAbs) buffer = maxAbs;
		if(buffer <= 0.0) return coerceAbs(x, maxAbs);
		
		// Soft-coerce the value x as required
		Scalar softLim = maxAbs - buffer;
		if(x > softLim)
			return maxAbs - buffer*std::exp(-(x - softLim) / buffer);
		if(x < -softLim)
			return -maxAbs + buffer*std::exp((x + softLim) / buffer);
		return x;
	}

	//! @brief Coerce the value @p x to be in the range `(-&infin;,max]` in a soft manner, rounding off exponentially within the soft range given by within @p buffer of the limits
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceSoftMax(Scalar x, Scalar max, Scalar buffer)
	{
		// Error checking on the buffer range
		if(buffer <= 0.0) return coerceMax(x, max);
		
		// Soft-coerce the value x as required
		Scalar softLim = max - buffer;
		if(x > softLim)
			return max - buffer*std::exp(-(x - softLim) / buffer);
		return x;
	}

	//! @brief Coerce the value @p x to be in the range `[min,&infin;)` in a soft manner, rounding off exponentially within the soft range given by within @p buffer of the limits
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceSoftMin(Scalar x, Scalar min, Scalar buffer)
	{
		// Error checking on the buffer range
		if(buffer <= 0.0) return coerceMin(x, min);
		
		// Soft-coerce the value x as required
		Scalar softLim = min + buffer;
		if(x < softLim)
			return min + buffer*std::exp((x - softLim) / buffer);
		return x;
	}
	
	//! @brief Interpolate a @c y value from a given @c x value
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type interpolate(Scalar x1, Scalar x2, Scalar y1, Scalar y2, Scalar x)
	{
		// Perform the interpolation as required
		if(x1 == x2) return 0.5*(y1 + y2);
		return y1 + (y2 - y1)*(x - x1)/(x2 - x1);
	}
	
	//! @brief Interpolate a @c y value from a given @c u value (\f$u = 0\f$ yields an output of \f$y_1\f$, and \f$u = 1\f$ yields an output of \f$y_2\f$)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type interpolate(Scalar y1, Scalar y2, Scalar u)
	{
		// Perform the interpolation as required
		return y1 + (y2 - y1)*u;
	}
	
	//! @brief Interpolate a @c y value from a given @c x value, with the result being coerced to the interval \f$[y_1,y_2]\f$
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type interpolateCoerced(Scalar x1, Scalar x2, Scalar y1, Scalar y2, Scalar x)
	{
		// Perform the interpolation as required
		if(x1 == x2) return 0.5*(y1 + y2);
		Scalar u = (x - x1)/(x2 - x1);
		if(u < 0.0) u = 0.0;
		else if(u > 1.0) u = 1.0;
		return y1 + u*(y2 - y1);
	}
	
	//! @brief Interpolate a @c y value from a given @c u value, with the result being coerced to the interval \f$[y_1,y_2]\f$ (\f$u = 0\f$ yields an output of \f$y_1\f$, and \f$u = 1\f$ yields an output of \f$y_2\f$)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type interpolateCoerced(Scalar y1, Scalar y2, Scalar u)
	{
		// Perform the interpolation as required
		if(u < 0.0) u = 0.0;
		else if(u > 1.0) u = 1.0;
		return y1 + (y2 - y1)*u;
	}
	///@}
}

#endif /* MATH_FUNCS_H */
// EOF