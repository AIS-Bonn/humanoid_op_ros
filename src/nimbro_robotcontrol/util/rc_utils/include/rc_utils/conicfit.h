// Implements functions that fit conics to data using linear least squares methods.
// File: conicfit.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef CONICFIT_H
#define CONICFIT_H

// Includes
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class ConicFit
	* 
	* @brief Container class for functions that fit conics to data using linear least squares.
	**/
	class ConicFit
	{
	public:
		// Constants
		static const double ZeroTol;
		
		// Typedefs
		typedef std::vector<double> Weights; //!< @brief A list of data point weights.
		typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > Points2D; //!< @brief A list of 2D data points.
		typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Points3D; //!< @brief A list of 3D data points.
		typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > WeightedPoints2D; //!< @brief A list of weighted 2D data points (the z-component is the weight).
		
		// Circle fitting
		static void fitCircle(const Points2D& P, Eigen::Vector2d& centre, double& radius); //!< @brief Fit a circle to 2D data.
		static void fitCircle(const Points3D& P, Eigen::Vector3d& centre, double& radius); //!< @brief Fit a circle to the XY components of 3D data.
		static void fitCircleCentred(const Points2D& P, const Eigen::Vector2d& centre, double& radius); //!< @brief Fit a circle of known centre to 2D data.
		static void fitCircleCentred(const Points3D& P, const Eigen::Vector3d& centre, double& radius); //!< @brief Fit a circle of known centre to the XY components of 3D data.
		static double fitCircleError(const Points2D& P, const Eigen::Vector2d& centre, double radius); //!< @brief Calculate the fitting error of a circle to certain 2D data.
		static double fitCircleError(const Points3D& P, const Eigen::Vector3d& centre, double radius); //!< @brief Calculate the fitting error of a circle to certain 3D data (the z coordinate is ignored).
		
		// Weighted circle fitting
		static void fitCircleWeighted(const WeightedPoints2D& P, Eigen::Vector2d& centre, double& radius); //!< @brief Fit a weighted circle to 2D data.
		static void fitCircleWeighted(const Points2D& P, const Weights& W, Eigen::Vector2d& centre, double& radius); //!< @brief Fit a weighted circle to 2D data.
		static void fitCircleCentredWeighted(const WeightedPoints2D& P, const Eigen::Vector2d& centre, double& radius); //!< @brief Fit a weighted circle of known centre to 2D data.
		static void fitCircleCentredWeighted(const Points2D& P, const Weights& W, const Eigen::Vector2d& centre, double& radius); //!< @brief Fit a weighted circle of known centre to 2D data.
		static double fitCircleErrorWeighted(const WeightedPoints2D& P, const Eigen::Vector2d& centre, double radius); //!< @brief Calculate the weighted fitting error of a circle to certain 2D data.
		static double fitCircleErrorWeighted(const Points2D& P, const Weights& W, const Eigen::Vector2d& centre, double radius); //!< @brief Calculate the weighted fitting error of a circle to certain 2D data.
		
		// Ellipse fitting
		static void fitEllipse(const Points2D& P, Eigen::Vector2d& centre, Eigen::Matrix2d& coeff); //!< @brief Fit an ellipse to 2D data.
		static void fitEllipse(const Points3D& P, Eigen::Vector3d& centre, Eigen::Matrix2d& coeff); //!< @brief Fit an ellipse to the XY components of 3D data.
		static void fitEllipseCentred(const Points2D& P, const Eigen::Vector2d& centre, Eigen::Matrix2d& coeff); //!< @brief Fit an ellipse of known centre to 2D data.
		static void fitEllipseCentred(const Points3D& P, const Eigen::Vector3d& centre, Eigen::Matrix2d& coeff); //!< @brief Fit an ellipse of known centre to the XY components of 3D data.
		static double fitEllipseErrorCoeff(const Points2D& P, const Eigen::Vector2d& centre, const Eigen::Matrix2d& coeff); //!< @brief Calculate the dimensionless fitting error of an ellipse to a certain set of 2D data points. Note that this function calls `fitEllipseError()`, and thus the latter should be used directly if the ellipse centre and transform are already available.
		static double fitEllipseErrorCoeff(const Points3D& P, const Eigen::Vector3d& centre, const Eigen::Matrix2d& coeff); //!< @brief Calculate the dimensionless fitting error of an ellipse to a certain set of 3D data points (the z coordinate is ignored). Note that this function calls `fitEllipseError()`, and thus the latter should be used directly if the ellipse centre and transform are already available.
		static double fitEllipseError(const Points2D& P, const Eigen::Vector2d& centre, const Eigen::Matrix2d& transform); //!< @brief Calculate the dimensionless fitting error of an ellipse to a certain set of 2D data points.
		static double fitEllipseError(const Points3D& P, const Eigen::Vector3d& centre, const Eigen::Matrix2d& transform); //!< @brief Calculate the dimensionless fitting error of an ellipse to a certain set of 3D data points (the z coordinate is ignored).
		
		// Sphere fitting
		static void fitSphere(const Points3D& P, Eigen::Vector3d& centre, double& radius); //!< @brief Fit a sphere to 3D data.
		static void fitSphereCentred(const Points3D& P, const Eigen::Vector3d& centre, double& radius); //!< @brief Fit a sphere of known centre to 3D data.
		static double fitSphereError(const Points3D& P, const Eigen::Vector3d& centre, double radius); //!< @brief Calculate the fitting error of a sphere to certain 3D data.
		
		// Ellipsoid fitting
		static void fitEllipsoid(const Points3D& P, Eigen::Vector3d& centre, Eigen::Matrix3d& coeff); //!< @brief Fit an ellipsoid to 3D data.
		static void fitEllipsoidCentred(const Points3D& P, const Eigen::Vector3d& centre, Eigen::Matrix3d& coeff); //!< @brief Fit an ellipsoid of known centre to 3D data.
		static double fitEllipsoidErrorCoeff(const Points3D& P, const Eigen::Vector3d& centre, const Eigen::Matrix3d& coeff); //!< @brief Calculate the dimensionless fitting error of an ellipsoid to a certain set of 3D data points. Note that this function calls `fitEllipsoidError()`, and thus the latter should be used directly if the ellipsoid centre and transform are already available.
		static double fitEllipsoidError(const Points3D& P, const Eigen::Vector3d& centre, const Eigen::Matrix3d& transform); //!< @brief Calculate the dimensionless fitting error of an ellipsoid to a certain set of 3D data points.
		
		// Ellipse functions
		static bool ellipseMatrixToAxes(const Eigen::Matrix2d& coeff, Eigen::Matrix2d& rotmat, Eigen::Vector2d& radii, double& angle); //!< @brief Calculate the rotation matrix, radii and rotation angle of an ellipse defined by a coefficient matrix. Only the lower triangular part of the coefficient matrix is referenced as it should be symmetric.
		static bool ellipseAxesToTransform(const Eigen::Matrix2d& rotmat, const Eigen::Vector2d& radii, Eigen::Matrix2d& transform); //!< @brief Calculate the normalisation transformation matrix of an ellipse defined by its rotation matrix and radii. The transformation matrix transforms points on the ellipse to points on the unit circle with zero net rotation.
		
		// Ellipsoid functions
		static bool ellipsoidMatrixToAxes(const Eigen::Matrix3d& coeff, Eigen::Matrix3d& rotmat, Eigen::Vector3d& radii); //!< @brief Calculate the rotation matrix and radii of an ellipsoid defined by a coefficient matrix. Only the lower triangular part of the coefficient matrix is referenced as it should be symmetric.
		static bool ellipsoidAxesToTransform(const Eigen::Matrix3d& rotmat, const Eigen::Vector3d& radii, Eigen::Matrix3d& transform); //!< @brief Calculate the normalisation transformation matrix of an ellipsoid defined by its rotation matrix and radii. The transformation matrix transforms points on the ellipsoid to points on the unit sphere with zero net rotation.
		
		// Conic data generation functions
		static void genCircleData(ConicFit::Points2D& P, size_t N, const Eigen::Vector2d& centre, double radius); //!< @brief Generate data points that lie on a given circle (returns `N` data points in vector @p P).
		static void genEllipseData(ConicFit::Points2D& P, size_t N, const Eigen::Vector2d& centre, const Eigen::Vector2d& radii, double angle); //!< @brief Generate data points that lie on a given ellipse (returns `N` data points in vector @p P).
		static void genSphereData(ConicFit::Points3D& PP, size_t N, const Eigen::Vector3d& centre, double radius); //!< @brief Generate data points that lie on a given sphere (returns `N^2` data points in vector @p PP).
		static void genEllipsoidData(ConicFit::Points3D& PP, size_t N, const Eigen::Vector3d& centre, const Eigen::Vector3d& radii, const Eigen::Matrix3d& rotmat); //!< @brief Generate data points that lie on a given ellipsoid (returns `N^2` data points in vector @p PP).
		
	private:
		// Constructor
		ConicFit() {}
		
		// Worker functions
		static void fitCircle(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::Vector2d& centre, double& radius);
		static void fitEllipse(const Eigen::Vector2d& mean, const Eigen::MatrixXd& D, const Eigen::VectorXd& y, Eigen::Vector2d& centre);
		static void fitEllipseCentred(const Eigen::MatrixXd& D, const Eigen::VectorXd& y, Eigen::Matrix2d& coeff);
	};
}

#endif
// EOF