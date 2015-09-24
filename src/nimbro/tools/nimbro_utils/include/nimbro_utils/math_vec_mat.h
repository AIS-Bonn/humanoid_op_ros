// Utilities for operations related to vectors and matrices
// File: math_vec_mat.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef MATH_VEC_MAT_H
#define MATH_VEC_MAT_H

// Includes
#include <tf/tf.h>
#include <Eigen/Core>

// Nimbro utilities namespace
namespace nimbro_utils
{
	/**
	* @name Conversion Functions (nimbro_utils/math_vec_mat.h)
	**/
	///@{

	//! @brief Eigen: Extend a 2D vector into a 3D vector by a zero z component
	inline Eigen::Vector3d zeroZ(const Eigen::Vector2d& vec)
	{
		// Return the required 3D vector
		return Eigen::Vector3d(vec.x(), vec.y(), 0.0);
	}
	//! @brief Eigen: Extend a 2D vector into a 3D vector by a given z component
	inline Eigen::Vector3d withZ(const Eigen::Vector2d& vec, double z)
	{
		// Return the required 3D vector
		return Eigen::Vector3d(vec.x(), vec.y(), z);
	}

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
}

#endif /* MATH_VEC_MAT_H */
// EOF