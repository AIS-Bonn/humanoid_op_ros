// Utilities for operations related to vectors and matrices
// File: math_vec_mat.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef MATH_VEC_MAT_H
#define MATH_VEC_MAT_H

// Includes
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

	//! @brief Eigen: Return the unit vector corresponding to the normalization of a particular vector (the zero vector is mapped to the unit vector in the positive x direction)
	template<typename Scalar, int Rows> inline Eigen::Matrix<Scalar, Rows, 1> eigenNormalized(const Eigen::Matrix<Scalar, Rows, 1>& vec)
	{
		Scalar norm = vec.norm();
		if(norm > 0.0)
			return (vec / norm);
		else
			return Eigen::Matrix<Scalar, Rows, 1>::UnitX();
	}

	//! @brief Eigen: Normalize a vector in-place to become a unit vector (the zero vector is mapped to the unit vector in the positive x direction)
	template<typename Scalar, int Rows> inline void eigenNormalize(Eigen::Matrix<Scalar, Rows, 1>& vec)
	{
		Scalar norm = vec.norm();
		if(norm > 0.0)
			vec /= norm;
		else
			vec = Eigen::Matrix<Scalar, Rows, 1>::UnitX();
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

	//! @brief Rotate a 2D vector @p vec clockwise in place by an angle @p angle in radians
	template<typename Scalar> inline void eigenRotateCW(Eigen::Matrix<Scalar, 2, 1>& vec, double angle)
	{
		// Rotate the given vector in place by the required angle
		double s = sin(angle), c = cos(angle);
		Scalar newX = c*vec.x() + s*vec.y();
		vec.y() = c*vec.y() - s*vec.x();
		vec.x() = newX;
	}

	//! @brief Rotate a 2D vector @p vec counterclockwise in place by an angle @p angle in radians
	template<typename Scalar> inline void eigenRotateCCW(Eigen::Matrix<Scalar, 2, 1>& vec, double angle)
	{
		// Rotate the given vector in place by the required angle
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

	//! @brief Rotate a 2D vector @p vec clockwise in place by 90 degrees
	template<typename Scalar> inline void eigenRotateCW90(Eigen::Matrix<Scalar, 2, 1>& vec)
	{
		// Rotate the given vector in place by 90 degrees
		Scalar newX = vec.y();
		vec.y() = -vec.x();
		vec.x() = newX;
	}

	//! @brief Rotate a 2D vector @p vec counterclockwise in place by 90 degrees
	template<typename Scalar> inline void eigenRotateCCW90(Eigen::Matrix<Scalar, 2, 1>& vec)
	{
		// Rotate the given vector in place by 90 degrees
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
}

#endif /* MATH_VEC_MAT_H */
// EOF