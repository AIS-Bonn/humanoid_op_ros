// Implements functions that fit planes to data.
// File: planefit.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rc_utils/planefit.h>
#include <Eigen/SVD>
#include <numeric>
#include <cmath>

// Namespaces
using namespace rc_utils;

//
// Plane fitting
//

// Fit a plane to 3D data
void PlaneFit::fitPlane(const Points3D& P, Eigen::Vector4d& coeff)
{
	// Calculate a point on the plane and the normal vector to it
	Eigen::Vector3d normal, mean;
	fitPlane(P, normal, mean);

	// Construct the coefficient vector (a,b,c,d) where the equation of the plane is ax + by + cz + d = 0
	coeff << normal, -normal.dot(mean);
}

// Fit a plane to 3D data
void PlaneFit::fitPlane(const Points3D& P, Eigen::Vector4d& coeff, Eigen::Vector3d& mean)
{
	// Calculate a point on the plane and the normal vector to it
	Eigen::Vector3d normal;
	fitPlane(P, normal, mean);

	// Construct the coefficient vector (a,b,c,d) where the equation of the plane is ax + by + cz + d = 0
	coeff << normal, -normal.dot(mean);
}

// Fit a plane to 3D data
void PlaneFit::fitPlane(const Points3D& P, Eigen::Vector3d& normal)
{
	// Calculate a point on the plane and the normal vector to it
	Eigen::Vector3d mean;
	fitPlane(P, normal, mean);
}

// Fit a plane to 3D data
void PlaneFit::fitPlane(const Points3D& P, Eigen::Vector3d& normal, Eigen::Vector3d& mean)
{
	// Retrieve how many data points there are
	size_t N = P.size();

	// Handle trivial cases
	if(N <= 0)
	{
		normal << 0.0, 0.0, 1.0;
		mean.setZero();
		return;
	}
	else if(N == 1)
	{
		normal << 0.0, 0.0, 1.0;
		mean = P[0];
		return;
	}

	// Calculate the mean of the data points (the plane of best fit always passes through the mean)
	mean = Eigen::Vector3d::Zero();
	if(N >= 1)
		mean = std::accumulate(P.begin(), P.end(), mean) / N;

	// Construct the matrix of zero mean data points
	Eigen::MatrixXd A(3, N);
	for(size_t i = 0; i < N; i++)
		A.col(i) = P[i] - mean;

	// Perform an SVD decomposition to determine the direction with the minimum variance
	unsigned int options = (N >= 3 ? Eigen::ComputeThinU : Eigen::ComputeFullU) | Eigen::ComputeThinV;
	normal = A.jacobiSvd(options).matrixU().col(2).normalized();
}

// Calculate the fitting error of a plane to certain 3D data
double PlaneFit::fitPlaneError(const Points3D& P, const Eigen::Vector4d& coeff)
{
	// Calculate the normalised equation coefficients (to put the resulting error in units of normal distance to plane)
	Eigen::Vector3d normal = coeff.head<3>();
	double norm = normal.norm();
	double scale = (norm > 0.0 ? norm : coeff.w());
	Eigen::Vector3d abc = normal / scale;
	double d = coeff.w() / scale;

	// Calculate the average distance to the plane of the data points
	double err = 0.0;
	size_t N = P.size();
	for(size_t i = 0; i < N; i++)
		err += fabs(abc.dot(P[i]) + d)/N;

	// Return the calculated error
	return err;
}

// Calculate the fitting error of a plane to certain 3D data
double PlaneFit::fitPlaneError(const Points3D& P, const Eigen::Vector3d& normal, Eigen::Vector3d& point)
{
	// Normalise the normal vector (to put the resulting error in units of normal distance to plane)
	Eigen::Vector3d n = normal.normalized();

	// Calculate the average distance to the plane of the data points
	double err = 0.0;
	size_t N = P.size();
	for(size_t i = 0; i < N; i++)
		err += fabs(n.dot(P[i] - point))/N;

	// Return the calculated error
	return err;
}
// EOF