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
		return angle + M_2PI*std::floor((M_PI - angle) / M_2PI);
	}

	//! @brief Wrap an angle to [0,2*pi)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type picutMod(Scalar angle)
	{
		// Return the required result
		return angle - M_2PI*std::floor(angle / M_2PI);
	}

	//! @brief Wrap an angle to (max-2*pi,max]
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type picutMax(Scalar angle, Scalar max)
	{
		// Return the required result
		return angle + M_2PI*std::floor((max - angle) / M_2PI);
	}

	//! @brief Wrap an angle to (min,min+2*pi]
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type picutMin(Scalar angle, Scalar min)
	{
		// Return the required result
		return angle + M_2PI*std::floor((min - angle) / M_2PI + 1.0);
	}

	//! @brief Wrap an angle to (around-pi,around+pi]
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type picutAround(Scalar angle, Scalar around)
	{
		// Return the required result
		return angle + M_2PI*std::floor((around + M_PI - angle) / M_2PI);
	}

	//! @brief Wrap an angle to (-pi,pi]
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, void>::type picutVar(Scalar& angle)
	{
		// Return the required result
		angle += M_2PI*std::floor((M_PI - angle) / M_2PI);
	}

	//! @brief Wrap an angle to [0,2*pi)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, void>::type picutVarMod(Scalar& angle)
	{
		// Return the required result
		angle -= M_2PI*std::floor(angle / M_2PI);
	}

	//! @brief Wrap an angle to (max-2*pi,max]
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, void>::type picutVarMax(Scalar& angle, Scalar max)
	{
		// Return the required result
		angle += M_2PI*std::floor((max - angle) / M_2PI);
	}

	//! @brief Wrap an angle to (min,min+2*pi]
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, void>::type picutVarMin(Scalar& angle, Scalar min)
	{
		// Return the required result
		angle += M_2PI*std::floor((min - angle) / M_2PI + 1.0);
	}

	//! @brief Wrap an angle to (around-pi,around+pi]
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, void>::type picutVarAround(Scalar& angle, Scalar around)
	{
		// Return the required result
		angle += M_2PI*std::floor((around + M_PI - angle) / M_2PI);
	}

	//! @brief Return the sign of a value (1 or -1)
	template<typename Scalar> typename boost::enable_if<boost::is_arithmetic<Scalar>, int>::type sign(Scalar x)
	{
		// Return -1 or 1 as appropriate
		if(x >= 0) return 1;
		else return -1;
	}

	//! @brief Return the sign of a value (1, 0 or -1)
	template<typename Scalar> typename boost::enable_if<boost::is_arithmetic<Scalar>, int>::type sign0(Scalar x)
	{
		// Return -1, 0 or 1 as appropriate
		if(x > 0) return 1;
		else if(x == 0) return 0;
		else return -1;
	}

	/**
	* @brief Helper struct for ellipse semi-axes lengths
	* 
	* An example of the intended use of this class is as follows:
	* @code
	* // Original code (without collapsing flat ellipses)
	* double R = ellipseRadius(configAxisA, configAxisB, theta);
	* 
	* // Can become...
	* EllipseAxesd EA(configAxisA, configAxisB, true);
	* double R = ellipseRadius(EA.a, EA.b, theta);
	* 
	* // Or alternatively...
	* EllipseAxesd EA = collapseFlatEllipse(configAxisA, configAxisB);
	* double R = ellipseRadius(EA.a, EA.b, theta);
	* @endcode
	**/
	template<typename Scalar> struct EllipseAxes
	{
		static_assert(boost::is_floating_point<Scalar>::value, "EllipseAxes only permits floating point scalar values!");
		EllipseAxes() = default;
		EllipseAxes(Scalar a, Scalar b) : a(a), b(b) {}
		EllipseAxes(Scalar a, Scalar b, bool collapse) : a(a), b(b) { if(collapse) collapseFlat(); }
		EllipseAxes<Scalar> collapsedFlat() const { EllipseAxes<Scalar> EA(*this); EA.collapseFlat(); return EA; }
		void collapseFlat()
		{
			if(a == 0.0)
				b = 0.0;
			else if(b == 0.0)
				a = 0.0;
		}
		Scalar a;
		Scalar b;
	};
	typedef EllipseAxes<float> EllipseAxesf;
	typedef EllipseAxes<double> EllipseAxesd;

	//! @brief Helper function to collapse flat ellipses down to a point, but otherwise leave every other ellipse untouched
	template<typename Scalar> EllipseAxes<Scalar> collapseFlatEllipse(Scalar a, Scalar b) { EllipseAxes<Scalar> EA(a, b); EA.collapseFlat(); return EA; }

	//! @brief Calculate the radius of an ellipse along the ray given by the polar angle \theta
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type ellipseRadius(Scalar a, Scalar b, Scalar theta)
	{
		// Calculate the equivalent point on the unit circle
		Scalar x = cos(theta);
		Scalar y = sin(theta);

		// Handle the case of a flat ellipse
		if(a == 0.0)
			return (x == 0.0 ? fabs(b) : 0.0);
		if(b == 0.0)
			return (y == 0.0 ? fabs(a) : 0.0);

		// General non-degenerate case
		Scalar xona = x / a;
		Scalar yonb = y / b;
		Scalar denom = xona*xona + yonb*yonb;
		return (denom == 0.0 ? 0.0 : sqrt(1.0 / denom));
	}

	//! @brief Calculate the radius of an ellipse along the ray given by the vector `(x,y)`
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type ellipseRadius(Scalar a, Scalar b, Scalar x, Scalar y)
	{
		// Handle the case of a flat ellipse
		if(a == 0.0)
			return (x == 0.0 ? fabs(b) : 0.0);
		if(b == 0.0)
			return (y == 0.0 ? fabs(a) : 0.0);

		// General non-degenerate case
		Scalar xona = x / a;
		Scalar yonb = y / b;
		Scalar num = x*x + y*y;
		Scalar denom = xona*xona + yonb*yonb;
		return (denom == 0.0 ? 0.0 : sqrt(num / denom));
	}

	//! @brief Coerce the value @p x to be in the range `[min,max]` (the mean of the two bounds is returned if min > max)
	template<typename Scalar> typename boost::enable_if<boost::is_integral<Scalar>, Scalar>::type coerce(Scalar x, Scalar min, Scalar max)
	{
		// Coerce the value x to be in the range [min,max]
		if(min > max) return (min + max) >> 1;
		else if(x >= max) return max;
		else if(x <= min) return min;
		else return x;
	}

	//! @brief Coerce the value @p x to be in the range `[min,max]` (the mean of the two bounds is returned if min > max)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerce(Scalar x, Scalar min, Scalar max)
	{
		// Coerce the value x to be in the range [min,max]
		if(min > max) return 0.5*(min + max);
		else if(x >= max) return max;
		else if(x <= min) return min;
		else return x;
	}

	//! @brief Coerce the value @p x to be in the range `[min,max]` and return whether coercion was necessary (the mean of the two bounds is returned if min > max)
	template<typename Scalar> typename boost::enable_if_c<boost::is_integral<Scalar>::value || boost::is_floating_point<Scalar>::value, Scalar>::type coerce(Scalar x, Scalar min, Scalar max, bool& coerced)
	{
		// Perform the required coercion
		Scalar y = coerce(x, min, max);
		coerced = (min > max || y != x);
		return y;
	}

	//! @brief Coerce the value @p x to be in the range `[-maxAbs,maxAbs]` (zero is returned if maxAbs < 0)
	template<typename Scalar> typename boost::enable_if<boost::is_arithmetic<Scalar>, Scalar>::type coerceAbs(Scalar x, Scalar maxAbs)
	{
		// Coerce the value x to be in the range [-maxAbs,maxAbs]
		if(maxAbs < 0) return 0;
		else if(x >= maxAbs) return maxAbs;
		else if(x <= -maxAbs) return -maxAbs;
		else return x;
	}

	//! @brief Coerce the value @p x to be in the range `[-maxAbs,maxAbs]` and return whether coercion was necessary (zero is returned if maxAbs < 0)
	template<typename Scalar> typename boost::enable_if<boost::is_arithmetic<Scalar>, Scalar>::type coerceAbs(Scalar x, Scalar maxAbs, bool& coerced)
	{
		// Perform the required coercion
		Scalar y = coerceAbs(x, maxAbs);
		coerced = (maxAbs < 0 || y != x);
		return y;
	}

	//! @brief Coerce the value @p x to be in the range `(-&infin;,max]`
	template<typename Scalar> typename boost::enable_if<boost::is_arithmetic<Scalar>, Scalar>::type coerceMax(Scalar x, Scalar max)
	{
		// Coerce the value x to (-Inf,max]
		return (x >= max ? max : x);
	}

	//! @brief Coerce the value @p x to be in the range `(-&infin;,max]` and return whether coercion was necessary
	template<typename Scalar> typename boost::enable_if<boost::is_arithmetic<Scalar>, Scalar>::type coerceMax(Scalar x, Scalar max, bool& coerced)
	{
		// Perform the required coercion
		Scalar y = coerceMax(x, max);
		coerced = (y != x);
		return y;
	}

	//! @brief Coerce the value @p x to be in the range `[min,&infin;)`
	template<typename Scalar> typename boost::enable_if<boost::is_arithmetic<Scalar>, Scalar>::type coerceMin(Scalar x, Scalar min)
	{
		// Coerce the value x to [min,Inf)
		return (x <= min ? min : x);
	}

	//! @brief Coerce the value @p x to be in the range `[min,&infin;)` and return whether coercion was necessary
	template<typename Scalar> typename boost::enable_if<boost::is_arithmetic<Scalar>, Scalar>::type coerceMin(Scalar x, Scalar min, bool& coerced)
	{
		// Perform the required coercion
		Scalar y = coerceMin(x, min);
		coerced = (y != x);
		return y;
	}

	//! @brief Coerce the polar point `(r,\theta)` to be in the ellipse defined by @p maxAbsX and @p maxAbsY (returns the coerced radius @p r)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceEllP(Scalar r, Scalar theta, Scalar maxAbsX, Scalar maxAbsY)
	{
		// Perform the required coercion
		return coerceAbs(r, ellipseRadius(maxAbsX, maxAbsY, theta));
	}

	//! @brief Coerce the polar point `(r,\theta)` to be in the ellipse defined by @p maxAbsX and @p maxAbsY (returns the coerced radius @p r and whether coercion was necessary)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceEllP(Scalar r, Scalar theta, Scalar maxAbsX, Scalar maxAbsY, bool& coerced)
	{
		// Perform the required coercion
		return coerceAbs(r, ellipseRadius(maxAbsX, maxAbsY, theta), coerced);
	}

	//! @brief Coerce the cartesian point `(x,y)` in place to be in the ellipse defined by @p maxAbsX and @p maxAbsY (returns the coerced point `(x,y)`)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, void>::type coerceEllC(Scalar& x, Scalar& y, Scalar maxAbsX, Scalar maxAbsY)
	{
		// Perform the required coercion
		Scalar r = sqrt(x*x + y*y);
		if(r <= 0.0) return;
		Scalar scale = coerceAbs(r, ellipseRadius<Scalar>(maxAbsX, maxAbsY, x, y)) / r;
		x *= scale;
		y *= scale;
	}

	//! @brief Coerce the cartesian point `(x,y)` in place to be in the ellipse defined by @p maxAbsX and @p maxAbsY (returns the coerced point `(x,y)` and whether coercion was necessary)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, void>::type coerceEllC(Scalar& x, Scalar& y, Scalar maxAbsX, Scalar maxAbsY, bool& coerced)
	{
		// Perform the required coercion
		Scalar r = sqrt(x*x + y*y);
		if(r <= 0.0) { coerced = false; return; }
		Scalar scale = coerceAbs(r, ellipseRadius<Scalar>(maxAbsX, maxAbsY, x, y), coerced) / r;
		x *= scale;
		y *= scale;
	}

	//! @brief Coerce the cartesian point `(x,y)` to be in the ellipse defined by @p maxAbsX and @p maxAbsY (returns the coerced point `(xout,yout)`)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, void>::type coerceEllC(Scalar x, Scalar y, Scalar maxAbsX, Scalar maxAbsY, Scalar& xout, Scalar& yout)
	{
		// Perform the required coercion
		coerceEllC<Scalar>(xout = x, yout = y, maxAbsX, maxAbsY);
	}

	//! @brief Coerce the cartesian point `(x,y)` to be in the ellipse defined by @p maxAbsX and @p maxAbsY (returns the coerced point `(xout,yout)` and whether coercion was necessary)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, void>::type coerceEllC(Scalar x, Scalar y, Scalar maxAbsX, Scalar maxAbsY, Scalar& xout, Scalar& yout, bool& coerced)
	{
		// Perform the required coercion
		coerceEllC<Scalar>(xout = x, yout = y, maxAbsX, maxAbsY, coerced);
	}

	//! @brief Coerce the value @p x to be in the range `[min,max]` in a soft manner, rounding off exponentially within the soft range given by within @p buffer of the limits
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceSoft(Scalar x, Scalar min, Scalar max, Scalar buffer)
	{
		// Error checking on the buffer range
		Scalar maxBuf = 0.5*(max - min);
		if(buffer <= 0.0 || maxBuf <= 0.0) return coerce(x, min, max);

		// Calculate the required centre slope
		Scalar slope = 1.0;
		if(buffer > maxBuf)
		{
			slope = std::exp(maxBuf/buffer - 1.0);
			buffer = maxBuf;
		}

		// Calculate the effective bounds
		Scalar boundL = min + buffer;
		Scalar boundU = max - buffer;

		// Soft-coerce the value x as required
		if(x > boundU)
			return max - buffer*std::exp(-(x - boundU) * slope / buffer);
		else if(x < boundL)
			return min + buffer*std::exp((x - boundL) * slope / buffer);
		else
			return x;
	}

	//! @brief Coerce the value @p x to be in the range `[min,max]` in a soft manner, rounding off exponentially within the soft range given by within @p buffer of the limits, and return whether coercion was necessary
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceSoft(Scalar x, Scalar min, Scalar max, Scalar buffer, bool& coerced)
	{
		// Error checking on the buffer range
		Scalar maxBuf = 0.5*(max - min);
		if(buffer <= 0.0 || maxBuf <= 0.0) return coerce(x, min, max, coerced);

		// Calculate the required centre slope
		Scalar slope = 1.0;
		if(buffer > maxBuf)
		{
			slope = std::exp(maxBuf/buffer - 1.0);
			buffer = maxBuf;
		}

		// Calculate the effective bounds
		Scalar boundL = min + buffer;
		Scalar boundU = max - buffer;

		// Soft-coerce the value x as required
		if(x > boundU)
		{
			coerced = true;
			return max - buffer*std::exp(-(x - boundU) * slope / buffer);
		}
		else if(x < boundL)
		{
			coerced = true;
			return min + buffer*std::exp((x - boundL) * slope / buffer);
		}
		else
		{
			coerced = false;
			return x;
		}
	}

	//! @brief Coerce the value @p x to be in the range `[-max_abs,max_abs]` in a soft manner, rounding off exponentially within the soft range given by within @p buffer of the limits
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceSoftAbs(Scalar x, Scalar maxAbs, Scalar buffer)
	{
		// Error checking on the buffer range
		Scalar maxBuf = maxAbs;
		if(buffer <= 0.0 || maxBuf <= 0.0) return coerceAbs(x, maxAbs);

		// Calculate the required centre slope
		Scalar slope = 1.0;
		if(buffer > maxBuf)
		{
			slope = std::exp(maxBuf/buffer - 1.0);
			buffer = maxBuf;
		}

		// Calculate the effective absolute bound
		Scalar bound = maxAbs - buffer;

		// Soft-coerce the value x as required
		if(x > bound)
			return maxAbs - buffer*std::exp(-(x - bound) * slope / buffer);
		else if(x < -bound)
			return -maxAbs + buffer*std::exp((x + bound) * slope / buffer);
		else
			return x;
	}

	//! @brief Coerce the value @p x to be in the range `[-max_abs,max_abs]` in a soft manner, rounding off exponentially within the soft range given by within @p buffer of the limits, and return whether coercion was necessary
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceSoftAbs(Scalar x, Scalar maxAbs, Scalar buffer, bool& coerced)
	{
		// Error checking on the buffer range
		Scalar maxBuf = maxAbs;
		if(buffer <= 0.0 || maxBuf <= 0.0) return coerceAbs(x, maxAbs, coerced);

		// Calculate the required centre slope
		Scalar slope = 1.0;
		if(buffer > maxBuf)
		{
			slope = std::exp(maxBuf/buffer - 1.0);
			buffer = maxBuf;
		}

		// Calculate the effective absolute bound
		Scalar bound = maxAbs - buffer;

		// Soft-coerce the value x as required
		if(x > bound)
		{
			coerced = true;
			return maxAbs - buffer*std::exp(-(x - bound) * slope / buffer);
		}
		else if(x < -bound)
		{
			coerced = true;
			return -maxAbs + buffer*std::exp((x + bound) * slope / buffer);
		}
		else
		{
			coerced = false;
			return x;
		}
	}

	//! @brief Coerce the value @p x to be in the range `(-&infin;,max]` in a soft manner, rounding off exponentially within the soft range given by within @p buffer of the limits
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceSoftMax(Scalar x, Scalar max, Scalar buffer)
	{
		// Error checking on the buffer range
		if(buffer <= 0.0) return coerceMax(x, max);

		// Calculate the effective bound
		Scalar boundU = max - buffer;

		// Soft-coerce the value x as required
		if(x > boundU)
			return max - buffer*std::exp(-(x - boundU) / buffer);
		else
			return x;
	}

	//! @brief Coerce the value @p x to be in the range `(-&infin;,max]` in a soft manner, rounding off exponentially within the soft range given by within @p buffer of the limits, and return whether coercion was necessary
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceSoftMax(Scalar x, Scalar max, Scalar buffer, bool& coerced)
	{
		// Error checking on the buffer range
		if(buffer <= 0.0) return coerceMax(x, max, coerced);

		// Calculate the effective bound
		Scalar boundU = max - buffer;

		// Soft-coerce the value x as required
		if(x > boundU)
		{
			coerced = true;
			return max - buffer*std::exp(-(x - boundU) / buffer);
		}
		else
		{
			coerced = false;
			return x;
		}
	}

	//! @brief Coerce the value @p x to be in the range `[min,&infin;)` in a soft manner, rounding off exponentially within the soft range given by within @p buffer of the limits
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceSoftMin(Scalar x, Scalar min, Scalar buffer)
	{
		// Error checking on the buffer range
		if(buffer <= 0.0) return coerceMin(x, min);

		// Calculate the effective bound
		Scalar boundL = min + buffer;

		// Soft-coerce the value x as required
		if(x < boundL)
			return min + buffer*std::exp((x - boundL) / buffer);
		else
			return x;
	}

	//! @brief Coerce the value @p x to be in the range `[min,&infin;)` in a soft manner, rounding off exponentially within the soft range given by within @p buffer of the limits, and return whether coercion was necessary
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceSoftMin(Scalar x, Scalar min, Scalar buffer, bool& coerced)
	{
		// Error checking on the buffer range
		if(buffer <= 0.0) return coerceMin(x, min, coerced);

		// Calculate the effective bound
		Scalar boundL = min + buffer;

		// Soft-coerce the value x as required
		if(x < boundL)
		{
			coerced = true;
			return min + buffer*std::exp((x - boundL) / buffer);
		}
		else
		{
			coerced = false;
			return x;
		}
	}

	//! @brief Coerce the polar point `(r,\theta)` in a soft manner to be in the ellipse defined by @p maxAbsX and @p maxAbsY (returns the coerced radius @p r)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceSoftEllP(Scalar r, Scalar theta, Scalar maxAbsX, Scalar maxAbsY, Scalar buffer)
	{
		// Perform the required coercion
		return coerceSoftAbs(r, ellipseRadius(maxAbsX, maxAbsY, theta), buffer);
	}

	//! @brief Coerce the polar point `(r,\theta)` in a soft manner to be in the ellipse defined by @p maxAbsX and @p maxAbsY (returns the coerced radius @p r and whether coercion was necessary)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, Scalar>::type coerceSoftEllP(Scalar r, Scalar theta, Scalar maxAbsX, Scalar maxAbsY, Scalar buffer, bool& coerced)
	{
		// Perform the required coercion
		return coerceSoftAbs(r, ellipseRadius(maxAbsX, maxAbsY, theta), buffer, coerced);
	}

	//! @brief Coerce the cartesian point `(x,y)` in place in a soft manner to be in the ellipse defined by @p maxAbsX and @p maxAbsY (returns the coerced point `(x,y)`)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, void>::type coerceSoftEllC(Scalar& x, Scalar& y, Scalar maxAbsX, Scalar maxAbsY, Scalar buffer)
	{
		// Perform the required coercion
		Scalar r = sqrt(x*x + y*y);
		if(r <= 0.0) return;
		Scalar scale = coerceSoftAbs(r, ellipseRadius<Scalar>(maxAbsX, maxAbsY, x, y), buffer) / r;
		x *= scale;
		y *= scale;
	}

	//! @brief Coerce the cartesian point `(x,y)` in place in a soft manner to be in the ellipse defined by @p maxAbsX and @p maxAbsY (returns the coerced point `(x,y)` and whether coercion was necessary)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, void>::type coerceSoftEllC(Scalar& x, Scalar& y, Scalar maxAbsX, Scalar maxAbsY, Scalar buffer, bool& coerced)
	{
		// Perform the required coercion
		Scalar r = sqrt(x*x + y*y);
		if(r <= 0.0) { coerced = false; return; }
		Scalar scale = coerceSoftAbs(r, ellipseRadius<Scalar>(maxAbsX, maxAbsY, x, y), buffer, coerced) / r;
		x *= scale;
		y *= scale;
	}

	//! @brief Coerce the cartesian point `(x,y)` in a soft manner to be in the ellipse defined by @p maxAbsX and @p maxAbsY (returns the coerced point `(xout,yout)`)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, void>::type coerceSoftEllC(Scalar x, Scalar y, Scalar maxAbsX, Scalar maxAbsY, Scalar buffer, Scalar& xout, Scalar& yout)
	{
		// Perform the required coercion
		coerceSoftEllC<Scalar>(xout = x, yout = y, maxAbsX, maxAbsY, buffer);
	}

	//! @brief Coerce the cartesian point `(x,y)` in a soft manner to be in the ellipse defined by @p maxAbsX and @p maxAbsY (returns the coerced point `(xout,yout)` and whether coercion was necessary)
	template<typename Scalar> typename boost::enable_if<boost::is_floating_point<Scalar>, void>::type coerceSoftEllC(Scalar x, Scalar y, Scalar maxAbsX, Scalar maxAbsY, Scalar buffer, Scalar& xout, Scalar& yout, bool& coerced)
	{
		// Perform the required coercion
		coerceSoftEllC<Scalar>(xout = x, yout = y, maxAbsX, maxAbsY, buffer, coerced);
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
		return y1 + (y2 - y1)*u;
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