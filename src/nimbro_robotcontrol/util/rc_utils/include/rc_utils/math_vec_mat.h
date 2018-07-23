// Utilities for operations related to vectors and matrices
// File: math_vec_mat.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef MATH_VEC_MAT_H
#define MATH_VEC_MAT_H

// Includes
#include <rc_utils/math_funcs.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <boost/utility/enable_if.hpp>
#include <Eigen/Core>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @name Simple Vector Operations (rc_utils/math_vec_mat.h)
	**/
	///@{

	//! @brief Eigen: Return the unit vector corresponding to the normalization of a particular vector (the zero vector is mapped to the supplied default vector, which is by default also the zero vector)
	template<typename Scalar, int Rows> inline Eigen::Matrix<Scalar, Rows, 1> eigenNormalized(const Eigen::Matrix<Scalar, Rows, 1>& vec, const Eigen::Matrix<Scalar, Rows, 1>& defaultIfZero = Eigen::Matrix<Scalar, Rows, 1>::Zero())
	{
		Scalar norm = vec.norm();
		if(norm > 0.0)
			return (vec / norm);
		else
			return defaultIfZero;
	}

	//! @brief Eigen: Return the unit vector corresponding to the normalization of a particular vector (the zero vector is mapped to the unit vector in the positive x direction)
	template<typename Scalar, int Rows> inline typename boost::enable_if_c<Rows >= 1, Eigen::Matrix<Scalar, Rows, 1> >::type eigenNormalizedDefX(const Eigen::Matrix<Scalar, Rows, 1>& vec)
	{
		return eigenNormalized<Scalar, Rows>(vec, Eigen::Matrix<Scalar, Rows, 1>::UnitX());
	}

	//! @brief Eigen: Return the unit vector corresponding to the normalization of a particular vector (the zero vector is mapped to the unit vector in the positive y direction)
	template<typename Scalar, int Rows> inline typename boost::enable_if_c<Rows >= 2, Eigen::Matrix<Scalar, Rows, 1> >::type eigenNormalizedDefY(const Eigen::Matrix<Scalar, Rows, 1>& vec)
	{
		return eigenNormalized<Scalar, Rows>(vec, Eigen::Matrix<Scalar, Rows, 1>::UnitY());
	}

	//! @brief Eigen: Return the unit vector corresponding to the normalization of a particular vector (the zero vector is mapped to the unit vector in the positive z direction)
	template<typename Scalar, int Rows> inline typename boost::enable_if_c<Rows >= 3, Eigen::Matrix<Scalar, Rows, 1> >::type eigenNormalizedDefZ(const Eigen::Matrix<Scalar, Rows, 1>& vec)
	{
		return eigenNormalized<Scalar, Rows>(vec, Eigen::Matrix<Scalar, Rows, 1>::UnitZ());
	}

	//! @brief Eigen: Normalize a vector in-place to become a unit vector (the zero vector is mapped to the supplied default vector, which is by default also the zero vector)
	template<typename Scalar, int Rows> inline void eigenNormalize(Eigen::Matrix<Scalar, Rows, 1>& vec, const Eigen::Matrix<Scalar, Rows, 1>& defaultIfZero = Eigen::Matrix<Scalar, Rows, 1>::Zero())
	{
		Scalar norm = vec.norm();
		if(norm > 0.0)
			vec /= norm;
		else
			vec = defaultIfZero;
	}

	//! @brief Eigen: Normalize a vector in-place to become a unit vector (the zero vector is mapped to the unit vector in the positive x direction)
	template<typename Scalar, int Rows> inline typename boost::enable_if_c<Rows >= 1, void>::type eigenNormalizeDefX(Eigen::Matrix<Scalar, Rows, 1>& vec)
	{
		eigenNormalize<Scalar, Rows>(vec, Eigen::Matrix<Scalar, Rows, 1>::UnitX());
	}

	//! @brief Eigen: Normalize a vector in-place to become a unit vector (the zero vector is mapped to the unit vector in the positive y direction)
	template<typename Scalar, int Rows> inline typename boost::enable_if_c<Rows >= 2, void>::type eigenNormalizeDefY(Eigen::Matrix<Scalar, Rows, 1>& vec)
	{
		eigenNormalize<Scalar, Rows>(vec, Eigen::Matrix<Scalar, Rows, 1>::UnitY());
	}

	//! @brief Eigen: Normalize a vector in-place to become a unit vector (the zero vector is mapped to the unit vector in the positive z direction)
	template<typename Scalar, int Rows> inline typename boost::enable_if_c<Rows >= 3, void>::type eigenNormalizeDefZ(Eigen::Matrix<Scalar, Rows, 1>& vec)
	{
		eigenNormalize<Scalar, Rows>(vec, Eigen::Matrix<Scalar, Rows, 1>::UnitZ());
	}

	//! @brief Eigen: Extend a 2D vector into a 3D vector by a zero z component
	template<typename Scalar> inline Eigen::Matrix<Scalar, 3, 1> zeroZ(const Eigen::Matrix<Scalar, 2, 1>& vec)
	{
		// Return the required 3D vector
		return Eigen::Matrix<Scalar, 3, 1>(vec.x(), vec.y(), 0.0);
	}

	//! @brief Eigen: Extend a 2D vector into a 3D vector by a given z component
	template<typename Scalar> inline Eigen::Matrix<Scalar, 3, 1> withZ(const Eigen::Matrix<Scalar, 2, 1>& vec, Scalar z)
	{
		// Return the required 3D vector
		return Eigen::Matrix<Scalar, 3, 1>(vec.x(), vec.y(), z);
	}

	///@}

	/**
	* @name Conversion Functions (rc_utils/math_vec_mat.h)
	**/
	///@{

	//! @brief Convert a 3D Eigen vector into a 3D TF vector
	inline tf::Vector3 eigenToTF(const Eigen::Vector3d& vec)
	{
		// Return the required tf vector
		return tf::Vector3(vec.x(), vec.y(), vec.z());
	}

	//! @brief Convert a 3x3 Eigen matrix into a 3x3 TF matrix
	inline tf::Matrix3x3 eigenToTF(const Eigen::Matrix3d& mat)
	{
		// Return the required tf matrix
		return tf::Matrix3x3(
			mat(0, 0), mat(0, 1), mat(0, 2),
			mat(1, 0), mat(1, 1), mat(1, 2),
			mat(2, 0), mat(2, 1), mat(2, 2)
		);
	}

	//! @brief Convert a 3D TF vector into a 3D Eigen vector
	inline Eigen::Vector3d tfToEigen(const tf::Vector3& vec)
	{
		// Return the required eigen vector
		return Eigen::Vector3d(vec.x(), vec.y(), vec.z());
	}

	//! @brief Convert a 3x3 TF matrix into a 3x3 Eigen matrix
	inline Eigen::Matrix3d tfToEigen(const tf::Matrix3x3& mat)
	{
		// Return the required eigen matrix
		Eigen::Matrix3d eigmat;
		eigmat << mat[0][0], mat[0][1], mat[0][2],
		          mat[1][0], mat[1][1], mat[1][2],
		          mat[2][0], mat[2][1], mat[2][2];
		return eigmat;
	}

	///@}

	/**
	* @name Eigen Math Functions (rc_utils/math_vec_mat.h)
	**/
	///@{

	//! @brief Calculate the angle in radians of a 2D vector @p vec, defined counterclockwise from the positive x-axis, in the range \f$[-\pi,\pi]\f$
	template<typename Scalar, int Rows> inline typename boost::enable_if_c<Rows >= 2, Scalar>::type eigenAngleOf(const Eigen::Matrix<Scalar, Rows, 1>& vec)
	{
		// Return the required angle
		return atan2(vec.y(), vec.x());
	}

	//! @brief Calculate the 2D unit vector that points a particular angle CCW from the positive x-axis
	template<typename Scalar> inline Eigen::Matrix<Scalar, 2, 1> eigenUnitVecOf(Scalar angle)
	{
		// Calculate the required unit vector
		return Eigen::Matrix<Scalar, 2, 1>(cos(angle), sin(angle));
	}

	//! @brief Calculate the 2D vector that points a particular angle CCW from the positive x-axis and is of a particular length
	template<typename Scalar> inline Eigen::Matrix<Scalar, 2, 1> eigenVecOf(Scalar angle, Scalar length)
	{
		// Calculate the required vector
		return Eigen::Matrix<Scalar, 2, 1>(length * cos(angle), length * sin(angle));
	}

	//! @brief Rotate a 2D vector @p vec clockwise in-place by an angle @p angle in radians
	template<typename Scalar> inline void eigenRotateCW(Eigen::Matrix<Scalar, 2, 1>& vec, double angle)
	{
		// Rotate the given vector in-place by the required angle
		double s = sin(angle), c = cos(angle);
		Scalar newX = c*vec.x() + s*vec.y();
		vec.y() = c*vec.y() - s*vec.x();
		vec.x() = newX;
	}

	//! @brief Rotate a 2D vector @p vec counterclockwise in-place by an angle @p angle in radians
	template<typename Scalar> inline void eigenRotateCCW(Eigen::Matrix<Scalar, 2, 1>& vec, double angle)
	{
		// Rotate the given vector in-place by the required angle
		double s = sin(angle), c = cos(angle);
		Scalar newX = c*vec.x() - s*vec.y();
		vec.y() = s*vec.x() + c*vec.y();
		vec.x() = newX;
	}

	//! @brief Return a vector corresponding to the 2D vector @p vec rotated clockwise by the angle @p angle in radians
	template<typename Scalar> inline Eigen::Matrix<Scalar, 2, 1> eigenRotatedCW(const Eigen::Matrix<Scalar, 2, 1>& vec, double angle)
	{
		// Return the given vector rotated by the required angle
		double s = sin(angle), c = cos(angle);
		return Eigen::Matrix<Scalar, 2, 1>(c*vec.x() + s*vec.y(), c*vec.y() - s*vec.x());
	}

	//! @brief Return a vector corresponding to the 2D vector @p vec rotated counterclockwise by the angle @p angle in radians
	template<typename Scalar> inline Eigen::Matrix<Scalar, 2, 1> eigenRotatedCCW(const Eigen::Matrix<Scalar, 2, 1>& vec, double angle)
	{
		// Return the given vector rotated by the required angle
		double s = sin(angle), c = cos(angle);
		return Eigen::Matrix<Scalar, 2, 1>(c*vec.x() - s*vec.y(), s*vec.x() + c*vec.y());
	}

	//! @brief Rotate a 2D vector @p vec clockwise in-place by 90 degrees
	template<typename Scalar> inline void eigenRotateCW90(Eigen::Matrix<Scalar, 2, 1>& vec)
	{
		// Rotate the given vector in-place by 90 degrees
		Scalar newX = vec.y();
		vec.y() = -vec.x();
		vec.x() = newX;
	}

	//! @brief Rotate a 2D vector @p vec counterclockwise in-place by 90 degrees
	template<typename Scalar> inline void eigenRotateCCW90(Eigen::Matrix<Scalar, 2, 1>& vec)
	{
		// Rotate the given vector in-place by 90 degrees
		Scalar newX = -vec.y();
		vec.y() = vec.x();
		vec.x() = newX;
	}

	//! @brief Return a vector corresponding to the 2D vector @p vec rotated clockwise by 90 degrees
	template<typename Scalar> inline Eigen::Matrix<Scalar, 2, 1> eigenRotatedCW90(const Eigen::Matrix<Scalar, 2, 1>& vec)
	{
		// Return the given vector rotated by 90 degrees
		return Eigen::Matrix<Scalar, 2, 1>(vec.y(), -vec.x());
	}

	//! @brief Return a vector corresponding to the 2D vector @p vec rotated counterclockwise by 90 degrees
	template<typename Scalar> inline Eigen::Matrix<Scalar, 2, 1> eigenRotatedCCW90(const Eigen::Matrix<Scalar, 2, 1>& vec)
	{
		// Return the given vector rotated by 90 degrees
		return Eigen::Matrix<Scalar, 2, 1>(-vec.y(), vec.x());
	}

	///@}

	/**
	* @name Ellipsoid Functions (rc_utils/math_vec_mat.h)
	**/
	///@{

	//! @brief Helper function to collapse flat ellipsoids down to a point, but otherwise leave every other ellipsoid untouched
	template<typename Scalar, int N> inline Eigen::Matrix<Scalar, N, 1> collapseFlatEllipsoid(const Eigen::Matrix<Scalar, N, 1>& semiaxes)
	{
		// Return a totally zero ellipsoid if any of the semiaxes are zero
		for(int n = 0; n < N; n++)
			if(semiaxes[n] == 0.0)
				return Eigen::Matrix<Scalar, N, 1>::Zero();
		return semiaxes;
	}
	template<typename Scalar> inline Eigen::Matrix<Scalar, 2, 1> collapseFlatEllipsoid(Scalar a, Scalar b)
	{
		// Return a totally zero ellipsoid if any of the semiaxes are zero
		if(a == 0.0 || b == 0.0)
			return Eigen::Matrix<Scalar, 2, 1>::Zero();
		else
			return Eigen::Matrix<Scalar, 2, 1>(a, b);
	}
	template<typename Scalar> inline Eigen::Matrix<Scalar, 3, 1> collapseFlatEllipsoid(Scalar a, Scalar b, Scalar c)
	{
		// Return a totally zero ellipsoid if any of the semiaxes are zero
		if(a == 0.0 || b == 0.0 || c == 0.0)
			return Eigen::Matrix<Scalar, 3, 1>::Zero();
		else
			return Eigen::Matrix<Scalar, 3, 1>(a, b, c);
	}

	//! @brief Calculate the radius of an N-dimensional ellipsoid along the ray given by the N-dimensional vector @p v
	template<typename Scalar, int N> inline typename boost::enable_if_c<(N == 1), Scalar>::type ellipsoidRadius(const Eigen::Matrix<Scalar, N, 1>& semiaxes, const Eigen::Matrix<Scalar, N, 1>& v) { return fabs(semiaxes.x()); }
	template<typename Scalar, int N> inline typename boost::enable_if_c<(N == 2), Scalar>::type ellipsoidRadius(const Eigen::Matrix<Scalar, N, 1>& semiaxes, const Eigen::Matrix<Scalar, N, 1>& v) { return ellipseRadius<Scalar>(semiaxes.x(), semiaxes.y(), v.x(), v.y()); }
	template<typename Scalar, int N> inline typename boost::enable_if_c<(N  > 2), Scalar>::type ellipsoidRadius(const Eigen::Matrix<Scalar, N, 1>& semiaxes, const Eigen::Matrix<Scalar, N, 1>& v)
	{
		// Calculate the required radius
		Scalar num = 0.0, denom = 0.0;
		for(int n = 0; n < N; n++)
		{
			// Retrieve the nth semiaxis length and vector component
			Scalar s = semiaxes[n];
			Scalar x = v[n];

			// Handle the case of flat ellipsoid dimensions
			if(s == 0.0)
			{
				if(x == 0.0)
					continue;
				else
					return 0.0;
			}

			// Account for the current dimension
			Scalar xons = x / s;
			num += x*x;
			denom += xons*xons;
		}
		return (num == 0.0 ? 0.0 : sqrt(num / denom));
	}

	//! @brief Coerce a cartesian vector @p v in place to be in the N-dimensional ellipsoid defined by @p semiaxes (returns the coerced point @p v)
	template<typename Scalar, int N> inline void coerceEllC(const Eigen::Matrix<Scalar, N, 1>& semiaxes, Eigen::Matrix<Scalar, N, 1>& v)
	{
		// Perform the required coercion
		Scalar r = v.norm();
		if(r <= 0.0) return;
		v *= coerceAbs(r, ellipsoidRadius<Scalar, N>(semiaxes, v)) / r;
	}

	//! @brief Coerce a cartesian vector @p v in place to be in the N-dimensional ellipsoid defined by @p semiaxes (returns the coerced point @p v and whether coercion was necessary)
	template<typename Scalar, int N> inline void coerceEllC(const Eigen::Matrix<Scalar, N, 1>& semiaxes, Eigen::Matrix<Scalar, N, 1>& v, bool& coerced)
	{
		// Perform the required coercion
		Scalar r = v.norm();
		if(r <= 0.0) { coerced = false; return; }
		v *= coerceAbs(r, ellipsoidRadius<Scalar, N>(semiaxes, v), coerced) / r;
	}

	//! @brief Coerce a cartesian vector @p v to be in the N-dimensional ellipsoid defined by @p semiaxes (returns the coerced point @p vout)
	template<typename Scalar, int N> inline void coerceEllC(const Eigen::Matrix<Scalar, N, 1>& semiaxes, const Eigen::Matrix<Scalar, N, 1>& v, Eigen::Matrix<Scalar, N, 1>& vout)
	{
		// Perform the required coercion
		coerceEllC<Scalar, N>(semiaxes, vout = v);
	}

	//! @brief Coerce a cartesian vector @p v to be in the N-dimensional ellipsoid defined by @p semiaxes (returns the coerced point @p vout and whether coercion was necessary)
	template<typename Scalar, int N> inline void coerceEllC(const Eigen::Matrix<Scalar, N, 1>& semiaxes, const Eigen::Matrix<Scalar, N, 1>& v, Eigen::Matrix<Scalar, N, 1>& vout, bool& coerced)
	{
		// Perform the required coercion
		coerceEllC<Scalar, N>(semiaxes, vout = v, coerced);
	}

	//! @brief Coerce a cartesian vector @p v in place in a soft manner to be in the N-dimensional ellipsoid defined by @p semiaxes (returns the coerced point @p v)
	template<typename Scalar, int N> inline void coerceSoftEllC(const Eigen::Matrix<Scalar, N, 1>& semiaxes, Eigen::Matrix<Scalar, N, 1>& v, Scalar buffer)
	{
		// Perform the required coercion
		Scalar r = v.norm();
		if(r <= 0.0) return;
		v *= coerceSoftAbs(r, ellipsoidRadius<Scalar, N>(semiaxes, v), buffer) / r;
	}

	//! @brief Coerce a cartesian vector @p v in place in a soft manner to be in the N-dimensional ellipsoid defined by @p semiaxes (returns the coerced point @p v and whether coercion was necessary)
	template<typename Scalar, int N> inline void coerceSoftEllC(const Eigen::Matrix<Scalar, N, 1>& semiaxes, Eigen::Matrix<Scalar, N, 1>& v, Scalar buffer, bool& coerced)
	{
		// Perform the required coercion
		Scalar r = v.norm();
		if(r <= 0.0) { coerced = false; return; }
		v *= coerceSoftAbs(r, ellipsoidRadius<Scalar, N>(semiaxes, v), buffer, coerced) / r;
	}

	//! @brief Coerce a cartesian vector @p v in a soft manner to be in the N-dimensional ellipsoid defined by @p semiaxes (returns the coerced point @p vout)
	template<typename Scalar, int N> inline void coerceSoftEllC(const Eigen::Matrix<Scalar, N, 1>& semiaxes, const Eigen::Matrix<Scalar, N, 1>& v, Scalar buffer, Eigen::Matrix<Scalar, N, 1>& vout)
	{
		// Perform the required coercion
		coerceSoftEllC<Scalar, N>(semiaxes, vout = v, buffer);
	}

	//! @brief Coerce a cartesian vector @p v in a soft manner to be in the N-dimensional ellipsoid defined by @p semiaxes (returns the coerced point @p vout and whether coercion was necessary)
	template<typename Scalar, int N> inline void coerceSoftEllC(const Eigen::Matrix<Scalar, N, 1>& semiaxes, const Eigen::Matrix<Scalar, N, 1>& v, Scalar buffer, Eigen::Matrix<Scalar, N, 1>& vout, bool& coerced)
	{
		// Perform the required coercion
		coerceSoftEllC<Scalar, N>(semiaxes, vout = v, buffer, coerced);
	}

	///@}
}

#endif /* MATH_VEC_MAT_H */
// EOF