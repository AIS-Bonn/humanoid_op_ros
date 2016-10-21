// Implements functions that fit planes to data.
// File: planefit.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef PLANEFIT_H
#define PLANEFIT_H

// Includes
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class PlaneFit
	* 
	* @brief Container class for functions that fit planes to data.
	**/
	class PlaneFit
	{
	public:
		// Typedefs
		typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Points3D; //!< @brief A list of 3D data points.
		
		// Plane fitting
		static void fitPlane(const Points3D& P, Eigen::Vector4d& coeff);
		static void fitPlane(const Points3D& P, Eigen::Vector4d& coeff, Eigen::Vector3d& mean);
		static void fitPlane(const Points3D& P, Eigen::Vector3d& normal);
		static void fitPlane(const Points3D& P, Eigen::Vector3d& normal, Eigen::Vector3d& mean);
		
		// Plane fitting error
		static double fitPlaneError(const Points3D& P, const Eigen::Vector4d& coeff);
		static double fitPlaneError(const Points3D& P, const Eigen::Vector3d& normal, Eigen::Vector3d& point);
		
	private:
		// Constructor
		PlaneFit() {}
	};
}

#endif
// EOF