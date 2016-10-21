// Implements functions that fit conics to data using linear least squares methods.
// File: conicfit.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rc_utils/conicfit.h>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <numeric>
#include <cmath>

// Namespaces
using namespace rc_utils;

// Fitting problems can often be expressed as an overconstrained set of linear equations Ax = b. Such a set of equations is generally
// solved by treating it as a least squares problem. The task is then to find a vector xhat that has the smallest residual squared
// error in satisfying Ax = b. It is a well-known result that the required minimising solution is given by:
//   xhat = (A'A)\(A'b)               {Matlab style}
// Or equivalently:
//   xhat = (A^T * A)^-1 * A^T * b    {Explicit style}
// The exact residual sum being minimised is:
//   J(x) = norm(b - Ax)^2

// Constants
const double ConicFit::ZeroTol = 1e-12;

//
// Circle fitting
//

// Fit a circle to 2D data
void ConicFit::fitCircle(const Points2D& P, Eigen::Vector2d& centre, double& radius)
{
	// Suppose the i-th data point is (xi,yi). We try to fit a circle of radius R and centre (cx,cy) to the data. Let:
	//   A = N rows of [xi yi 1]              {Nx3 matrix}
	//   x = [2*cx; 2*cy; R^2-cx^2-cy^2]      {3x1 column vector}
	//   b = N rows of [xi^2 + yi^2]          {Nx1 column vector}
	// Ax = b is equivalent to the condition that all data points lie on a circle of radius R, centered at (cx,cy).
	// Hence by solving Ax = b for x in a least squares sense, we are implicitly solving for the values of cx, cy, R
	// for which the corresponding circle most closely fits the data. We perform a mean shift on the data for
	// numerical conditioning and stability reasons.
	
	// Declare variables
	size_t N = P.size();
	Eigen::MatrixXd A(N, 3);
	Eigen::VectorXd b(N);
	
	// Calculate the mean of the data points
	Eigen::Vector2d mean = Eigen::Vector2d::Zero();
	if(N >= 1)
		mean = std::accumulate(P.begin(), P.end(), mean) / N;
	
	// Construct the linear equation to solve (Ax = b)
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector2d p = P[i] - mean;
		A(i, 0) = p.x();
		A(i, 1) = p.y();
		A(i, 2) = 1.0;
		b(i) = p.squaredNorm();
	}
	
	// Call the worker function to fit the circle
	fitCircle(A, b, centre, radius);
	
	// Correct for the applied mean shift
	centre += mean;
}

// Fit a circle to the XY components of 3D data
void ConicFit::fitCircle(const Points3D& P, Eigen::Vector3d& centre, double& radius)
{
	// We discard the z coordinates and fit a circle as normal for 2D data.
	// The returned centre z coordinate is a plain mean of the input z coordinates.
	
	// Declare variables
	size_t N = P.size();
	Eigen::MatrixXd A(N, 3);
	Eigen::VectorXd b(N);
	
	// Calculate the mean of the data points
	Eigen::Vector3d mean = Eigen::Vector3d::Zero();
	if(N >= 1)
		mean = std::accumulate(P.begin(), P.end(), mean) / N;
	
	// Construct the linear equation to solve (Ax = b)
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector3d p = P[i] - mean;
		A(i, 0) = p.x();
		A(i, 1) = p.y();
		A(i, 2) = 1.0;
		b(i) = p.x()*p.x() + p.y()*p.y();
	}
	
	// Call the worker function to fit the circle
	Eigen::Vector2d centre2D;
	fitCircle(A, b, centre2D, radius);
	
	// Correct for the applied mean shift
	centre.x() = mean.x() + centre2D.x();
	centre.y() = mean.y() + centre2D.y();
	centre.z() = mean.z();
}

// Worker function for fitting a circle
void ConicFit::fitCircle(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::Vector2d& centre, double& radius)
{
	// Find the least squares solution to Ax = b using Singular Value Decomposition (linear least squares fitting)
	Eigen::VectorXd coefficients = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
	
	// Extract the centre and radius of the fitted circle
	centre.x() = 0.5*coefficients(0);
	centre.y() = 0.5*coefficients(1);
	radius = sqrt(coefficients(2) + centre.squaredNorm());
}

// Fit a circle of known centre to 2D data
void ConicFit::fitCircleCentred(const Points2D& P, const Eigen::Vector2d& centre, double& radius)
{
	// Suppose the i-th data point is (xi,yi). The least squares solution to finding a circle of radius R of known centre
	// (cx,cy) that best fits the data points has a simple algebraic solution, given by:
	//   R = sqrt(sum((xi-cx)^2 + (yi-cy)^2, 1, N) / N)
	// In other words, the radius of best fit is the quadratic mean of the distances from the known centre to each point.
	
	// Declare variables
	size_t N = P.size();
	
	// Calculate the radius of best fit
	double Rsq = 0.0;
	for(size_t i = 0; i < N; i++)
		Rsq += (P[i] - centre).squaredNorm() / N;
	radius = sqrt(Rsq);
}

// Fit a circle of known centre to the XY components of 3D data
void ConicFit::fitCircleCentred(const Points3D& P, const Eigen::Vector3d& centre, double& radius)
{
	// We discard the z coordinates and fit a circle of known centre as normal for 2D data.
	
	// Declare variables
	size_t N = P.size();
	Eigen::Vector2d centre2D = centre.head<2>();
	
	// Calculate the radius of best fit
	double Rsq = 0.0;
	for(size_t i = 0; i < N; i++)
		Rsq += (P[i].head<2>() - centre2D).squaredNorm() / N;
	radius = sqrt(Rsq);
}

// Calculate the fitting error of a circle to certain 2D data
double ConicFit::fitCircleError(const Points2D& P, const Eigen::Vector2d& centre, double radius)
{
	// The returned error is the average distance to the circle in units of circle radii.
	
	// Calculate and return the required dimensionless fitting error
	size_t N = P.size();
	if(N <= 0) return 0.0;
	double err = 0.0;
	if(radius != 0.0)
	{
		for(size_t i = 0; i < N; i++)
			err += fabs((P[i] - centre).norm() - radius);
		err /= N*fabs(radius);
	}
	else
	{
		for(size_t i = 0; i < N; i++)
			err += fabs((P[i] - centre).norm());
		err /= N;
	}
	return err;
}

// Calculate the fitting error of a circle to certain 3D data
double ConicFit::fitCircleError(const Points3D& P, const Eigen::Vector3d& centre, double radius)
{
	// The returned error is the average planar XY distance to the circle in units of circle radii.
	
	// Calculate and return the required dimensionless fitting error
	size_t N = P.size();
	if(N <= 0) return 0.0;
	Eigen::Vector2d centre2D = centre.head<2>();
	double err = 0.0;
	if(radius != 0.0)
	{
		for(size_t i = 0; i < N; i++)
			err += fabs((P[i].head<2>() - centre2D).norm() - radius);
		err /= N*fabs(radius);
	}
	else
	{
		for(size_t i = 0; i < N; i++)
			err += fabs((P[i].head<2>() - centre2D).norm());
		err /= N;
	}
	return err;
}

//
// Weighted circle fitting
//

// Fit a weighted circle to 2D data
void ConicFit::fitCircleWeighted(const WeightedPoints2D& P, Eigen::Vector2d& centre, double& radius)
{
	// Suppose the i-th data point is (xi,yi,wi). We try to fit a circle of radius R and centre (cx,cy) to the data. Let:
	//   A = N rows of [wi*xi wi*yi wi]       {Nx3 matrix}
	//   x = [2*cx; 2*cy; R^2-cx^2-cy^2]      {3x1 column vector}
	//   b = N rows of [wi*(xi^2 + yi^2)]     {Nx1 column vector}
	// Ax = b is equivalent to the weighted condition that all data points lie on a circle of radius R, centered at (cx,cy).
	// Hence by solving Ax = b for x in a least squares sense, we are implicitly solving for the values of cx, cy, R
	// for which the corresponding circle most closely fits the weighted data. We perform a mean shift on the data for
	// numerical conditioning and stability reasons.
	
	// Declare variables
	size_t N = P.size();
	Eigen::MatrixXd A(N, 3);
	Eigen::VectorXd b(N);
	
	// Calculate the mean of the data points
	Eigen::Vector2d mean = Eigen::Vector2d::Zero();
	for(size_t i = 0; i < N; i++)
		mean += P[i].head<2>();
	if(N >= 1)
		mean /= N;
	
	// Construct the linear equation to solve (Ax = b)
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector2d p = P[i].head<2>() - mean;
		double w = P[i].z();
		A(i, 0) = w*p.x();
		A(i, 1) = w*p.y();
		A(i, 2) = w;
		b(i) = w*p.squaredNorm();
	}
	
	// Call the worker function to fit the circle
	fitCircle(A, b, centre, radius);
	
	// Correct for the applied mean shift
	centre += mean.head<2>();
}

// Fit a weighted circle to 2D data
void ConicFit::fitCircleWeighted(const Points2D& P, const Weights& W, Eigen::Vector2d& centre, double& radius)
{
	// Suppose the i-th data point is (xi,yi,wi). We try to fit a circle of radius R and centre (cx,cy) to the data. Let:
	//   A = N rows of [wi*xi wi*yi wi]       {Nx3 matrix}
	//   x = [2*cx; 2*cy; R^2-cx^2-cy^2]      {3x1 column vector}
	//   b = N rows of [wi*(xi^2 + yi^2)]     {Nx1 column vector}
	// Ax = b is equivalent to the weighted condition that all data points lie on a circle of radius R, centered at (cx,cy).
	// Hence by solving Ax = b for x in a least squares sense, we are implicitly solving for the values of cx, cy, R
	// for which the corresponding circle most closely fits the weighted data. We perform a mean shift on the data for
	// numerical conditioning and stability reasons.
	
	// Error checking
	size_t N = P.size();
	if(N != W.size())
	{
		centre.setZero();
		radius = 0.0;
		return;
	}
	
	// Declare variables
	Eigen::MatrixXd A(N, 3);
	Eigen::VectorXd b(N);
	
	// Calculate the mean of the data points
	Eigen::Vector2d mean = Eigen::Vector2d::Zero();
	if(N >= 1)
		mean = std::accumulate(P.begin(), P.end(), mean) / N;
	
	// Construct the linear equation to solve (Ax = b)
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector2d p = P[i] - mean;
		double w = W[i];
		A(i, 0) = w*p.x();
		A(i, 1) = w*p.y();
		A(i, 2) = w;
		b(i) = w*p.squaredNorm();
	}
	
	// Call the worker function to fit the circle
	fitCircle(A, b, centre, radius);
	
	// Correct for the applied mean shift
	centre += mean;
}

// Fit a weighted circle of known centre to 2D data
void ConicFit::fitCircleCentredWeighted(const WeightedPoints2D& P, const Eigen::Vector2d& centre, double& radius)
{
	// Suppose the i-th data point is (xi,yi,wi). The least squares solution to finding a circle of radius R of known centre
	// (cx,cy) that best fits the data points has a simple algebraic solution, given by:
	//   R = sqrt(sum(wi^2*((xi-cx)^2 + (yi-cy)^2), 1, N) / sum(wi^2, 1, N))
	// In other words, the radius of best fit is the weighted quadratic mean of the distances from the known centre to each point,
	// using weights the squares of those that are passed (to be consistent with the definition of the weights in other fitCircle functions).
	
	// Declare variables
	size_t N = P.size();
	
	// Calculate the radius of best fit
	double Rsq = 0.0, wsqsum = 0.0;
	for(size_t i = 0; i < N; i++)
	{
		double w = P[i].z();
		double wsq = w*w;
		Rsq += wsq*((P[i].head<2>() - centre).squaredNorm());
		wsqsum += wsq;
	}
	if(wsqsum > 0.0)
		radius = sqrt(Rsq / wsqsum);
	else
		radius = 0.0;
}

// Fit a weighted circle of known centre to 2D data
void ConicFit::fitCircleCentredWeighted(const Points2D& P, const Weights& W, const Eigen::Vector2d& centre, double& radius)
{
	// Suppose the i-th data point is (xi,yi,wi). The least squares solution to finding a circle of radius R of known centre
	// (cx,cy) that best fits the data points has a simple algebraic solution, given by:
	//   R = sqrt(sum(wi^2*((xi-cx)^2 + (yi-cy)^2), 1, N) / sum(wi^2, 1, N))
	// In other words, the radius of best fit is the weighted quadratic mean of the distances from the known centre to each point,
	// using weights the squares of those that are passed (to be consistent with the definition of the weights in other fitCircle functions).
	
	// Error checking
	size_t N = P.size();
	if(N != W.size())
	{
		radius = 0.0;
		return;
	}
	
	// Calculate the radius of best fit
	double Rsq = 0.0, wsqsum = 0.0;
	for(size_t i = 0; i < N; i++)
	{
		double w = W[i];
		double wsq = w*w;
		Rsq += wsq*((P[i] - centre).squaredNorm());
		wsqsum += wsq;
	}
	if(wsqsum > 0.0)
		radius = sqrt(Rsq / wsqsum);
	else
		radius = 0.0;
}

// Calculate the weighted fitting error of a circle to certain 2D data (the returned error is the weighted average distance of the data points to the circle in units of circle radii)
double ConicFit::fitCircleErrorWeighted(const WeightedPoints2D& P, const Eigen::Vector2d& centre, double radius)
{
	// Calculate and return the required dimensionless fitting error
	size_t N = P.size();
	if(N <= 0) return 0.0;
	radius = fabs(radius);
	double err = 0.0, wsum = 0.0;
	for(size_t i = 0; i < N; i++)
	{
		double w = fabs(P[i].z());
		err += w*fabs((P[i].head<2>() - centre).norm() - radius);
		wsum += w;
	}
	if(wsum == 0.0)
		err = 0.0;
	else
	{
		if(radius != 0.0)
			err /= radius;
		err /= wsum;
	}
	return err;
}

// Calculate the weighted fitting error of a circle to certain 2D data (the returned error is the weighted average distance of the data points to the circle in units of circle radii)
double ConicFit::fitCircleErrorWeighted(const Points2D& P, const Weights& W, const Eigen::Vector2d& centre, double radius)
{
	// Calculate and return the required dimensionless fitting error
	size_t N = P.size();
	if(N != W.size()) return 1e16;
	if(N <= 0) return 0.0;
	radius = fabs(radius);
	double err = 0.0, wsum = 0.0;
	for(size_t i = 0; i < N; i++)
	{
		double w = fabs(W[i]);
		err += w*fabs((P[i] - centre).norm() - radius);
		wsum += w;
	}
	if(wsum == 0.0)
		err = 0.0;
	else
	{
		if(radius != 0.0)
			err /= radius;
		err /= wsum;
	}
	return err;
}


//
// Ellipse fitting
//

// Fit an ellipse to 2D data
void ConicFit::fitEllipse(const Points2D& P, Eigen::Vector2d& centre, Eigen::Matrix2d& coeff)
{
	// We first compute the mean point (xm,ym) of the data points, and bias the data points by the negative of this, resulting in zero mean data.
	// Suppose that the zero mean set of points is (xi,yi). We then fit the following equation by least squares:
	//   ax^2 + by^2 + 2cxy + 2dx + 2ey = 1
	// We do this by expressing the least squares fitting problem as the matrix equation Dv = y, where:
	//   D = N rows of [xi^2 yi^2 2*xi*yi 2*xi 2*yi]    {Nx5 matrix}
	//   v = [a; b; c; d; e]                            {5x1 column vector}
	//   y = N rows of [1]                              {Nx1 column vector}
	// The centre of the ellipse is at the point where the gradient of ax^2 + by^2 + 2cxy + 2dx + 2ey is zero. Therefore, accounting for (xm,ym):
	//   [cx; cy] = [xm; ym] - [a c; c b] \ [d; e]      {Matlab style}
	// (cx,cy) is the centre of the ellipse of best fit through the original data points. The coefficient matrix is then computed by fitting an
	// ellipse of known centre (cx,cy) through the data points. If A is the resulting coefficient matrix, the ellipse of best fit is:
	//   (x-c)'A(x-c) = 1
	// where x = [x; y] and c = [cx; cy] are column vectors.
	
	// Declare variables
	size_t N = P.size();
	Eigen::MatrixXd D(N, 5);
	Eigen::VectorXd y(N);
	
	// Calculate the mean of the data points
	Eigen::Vector2d mean = Eigen::Vector2d::Zero();
	if(N >= 1)
		mean = std::accumulate(P.begin(), P.end(), mean) / N;
	
	// Construct the linear equation to solve (Dv = y)
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector2d p = P[i] - mean;
		D(i, 0) = p.x()*p.x();
		D(i, 1) = p.y()*p.y();
		D(i, 2) = 2.0*p.x()*p.y();
		D(i, 3) = 2.0*p.x();
		D(i, 4) = 2.0*p.y();
		y(i) = 1.0;
	}
	
	// Compute the centre of the ellipse of best fit
	fitEllipse(mean, D, y, centre);
	
	// Compute the coefficient matrix of the ellipse of best fit through the nominated centre
	fitEllipseCentred(P, centre, coeff);
}

// Fit an ellipse to the XY components of 3D data
void ConicFit::fitEllipse(const Points3D& P, Eigen::Vector3d& centre, Eigen::Matrix2d& coeff)
{
	// We discard the z coordinates and fit an ellipse as normal for 2D data.
	// The returned centre z coordinate is a plain mean of the input z coordinates.
	
	// Declare variables
	size_t N = P.size();
	Eigen::MatrixXd D(N, 5);
	Eigen::VectorXd y(N);
	
	// Calculate the mean of the data points
	Eigen::Vector3d mean = Eigen::Vector3d::Zero();
	if(N >= 1)
		mean = std::accumulate(P.begin(), P.end(), mean) / N;
	
	// Construct the linear equation to solve (Dv = y)
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector3d p = P[i] - mean;
		D(i, 0) = p.x()*p.x();
		D(i, 1) = p.y()*p.y();
		D(i, 2) = 2.0*p.x()*p.y();
		D(i, 3) = 2.0*p.x();
		D(i, 4) = 2.0*p.y();
		y(i) = 1.0;
	}
	
	// Compute the centre of the ellipse of best fit
	Eigen::Vector2d centre2D;
	fitEllipse(mean.head<2>(), D, y, centre2D);
	centre.x() = centre2D.x();
	centre.y() = centre2D.y();
	centre.z() = mean.z();
	
	// Compute the coefficient matrix of the ellipse of best fit through the nominated centre
	fitEllipseCentred(P, centre, coeff);
}

// Worker function for fitting an ellipse
void ConicFit::fitEllipse(const Eigen::Vector2d& mean, const Eigen::MatrixXd& D, const Eigen::VectorXd& y, Eigen::Vector2d& centre)
{
	// Find the least squares solution to Dv = y using Singular Value Decomposition (linear least squares fitting)
	Eigen::VectorXd vhat = D.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);
	
	// Construct the coefficient matrix and vector
	Eigen::Matrix2d A;
	A << vhat(0), vhat(2),
	     vhat(2), vhat(1);
	Eigen::Vector2d b(vhat(3), vhat(4));
	
	// Calculate the centre of the fitted ellipse
	centre = mean - A.colPivHouseholderQr().solve(b);
}

// Fit an ellipse of known centre to 2D data
void ConicFit::fitEllipseCentred(const Points2D& P, const Eigen::Vector2d& centre, Eigen::Matrix2d& coeff)
{
	// We first bias the data to be relative to the centre (cx,cy) of the ellipse to fit. We then fit the following equation by least squares:
	//   ax^2 + by^2 + 2cxy = 1
	// If we define A = [a c; c b] as the coefficient matrix, and let xc = [x; y] then this is equivalent to:
	//   xc'*A*xc = 1
	// Suppose that the i-th data point of biased data is (xi,yi). We perform the ellipse fit by expressing the least squares fitting problem
	// as the matrix equation Dv = y, where:
	//   D = N rows of [xi^2 yi^2 2*xi*yi]    {Nx3 matrix}
	//   v = [a; b; c]                        {3x1 column vector}
	//   y = N rows of [1]                    {Nx1 column vector}
	// The coefficient matrix A can be constructed from the solution v, after which the equation of the ellipse of best fit through the
	// original data becomes:
	//   (x-c)'A(x-c) = 1
	// where x = [x; y] and c = [cx; cy] are column vectors, and c is the specified centre of the ellipse.
	
	// Declare variables
	size_t N = P.size();
	Eigen::MatrixXd D(N, 3);
	Eigen::VectorXd y(N);
	
	// Construct the linear equation to solve (Dv = y)
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector2d p = P[i] - centre;
		D(i, 0) = p.x()*p.x();
		D(i, 1) = p.y()*p.y();
		D(i, 2) = 2.0*p.x()*p.y();
		y(i) = 1.0;
	}
	
	// Compute the coefficient matrix of the ellipse of best fit through the nominated centre
	fitEllipseCentred(D, y, coeff);
}

// Fit an ellipse of known centre to the XY components of 3D data
void ConicFit::fitEllipseCentred(const Points3D& P, const Eigen::Vector3d& centre, Eigen::Matrix2d& coeff)
{
	// We discard the z coordinates and fit an ellipse of known centre as normal for 2D data.
	
	// Declare variables
	size_t N = P.size();
	Eigen::MatrixXd D(N, 3);
	Eigen::VectorXd y(N);
	
	// Construct the linear equation to solve (Dv = y)
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector3d p = P[i] - centre;
		D(i, 0) = p.x()*p.x();
		D(i, 1) = p.y()*p.y();
		D(i, 2) = 2.0*p.x()*p.y();
		y(i) = 1.0;
	}
	
	// Compute the coefficient matrix of the ellipse of best fit through the nominated centre
	fitEllipseCentred(D, y, coeff);
}

// Worker function for fitting an ellipse with a known centre
void ConicFit::fitEllipseCentred(const Eigen::MatrixXd& D, const Eigen::VectorXd& y, Eigen::Matrix2d& coeff)
{
	// Find the least squares solution to Dv = y using Singular Value Decomposition (linear least squares fitting)
	Eigen::VectorXd vhat = D.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);
	
	// Construct the coefficient matrix
	coeff << vhat(0), vhat(2),
	         vhat(2), vhat(1);
}

// Calculate the fitting error of an ellipse to certain 2D data (average distance of normalised points to the unit circle)
double ConicFit::fitEllipseErrorCoeff(const Points2D& P, const Eigen::Vector2d& centre, const Eigen::Matrix2d& coeff)
{
	// Declare variables
	Eigen::Matrix2d rotmat, transform;
	Eigen::Vector2d radii;
	double angle;
	
	// Calculate the transformation matrix
	double failed = (double) P.size();
	if(!ellipseMatrixToAxes(coeff, rotmat, radii, angle)) return failed;
	if(!ellipseAxesToTransform(rotmat, radii, transform)) return failed;
	
	// Calculate the fitting error based on the centre and transformation matrix
	return fitEllipseError(P, centre, transform);
}

// Calculate the fitting error of an ellipse to certain 2D data (average distance of normalised points to the unit circle)
double ConicFit::fitEllipseError(const Points2D& P, const Eigen::Vector2d& centre, const Eigen::Matrix2d& transform)
{
	// Calculate and return the required dimensionless fitting error
	double err = 0.0;
	size_t N = P.size();
	for(size_t i = 0; i < N; i++)
		err += fabs((transform*(P[i] - centre)).norm() - 1.0)/N;
	return err;
}

// Calculate the fitting error of an ellipse to certain 3D data (average distance of normalised points to the unit circle)
double ConicFit::fitEllipseErrorCoeff(const Points3D& P, const Eigen::Vector3d& centre, const Eigen::Matrix2d& coeff)
{
	// Declare variables
	Eigen::Matrix2d rotmat, transform;
	Eigen::Vector2d radii;
	double angle;
	
	// Calculate the transformation matrix
	double failed = (double) P.size();
	if(!ellipseMatrixToAxes(coeff, rotmat, radii, angle)) return failed;
	if(!ellipseAxesToTransform(rotmat, radii, transform)) return failed;
	
	// Calculate the fitting error based on the centre and transformation matrix
	return fitEllipseError(P, centre, transform);
}

// Calculate the fitting error of an ellipse to certain 3D data (average distance of normalised points to the unit circle)
double ConicFit::fitEllipseError(const Points3D& P, const Eigen::Vector3d& centre, const Eigen::Matrix2d& transform)
{
	// Calculate and return the required dimensionless fitting error
	double err = 0.0;
	size_t N = P.size();
	Eigen::Vector2d centre2D = centre.head<2>();
	for(size_t i = 0; i < N; i++)
		err += fabs((transform*(P[i].head<2>() - centre2D)).norm() - 1.0)/N;
	return err;
}

//
// Sphere fitting
//

// Fit a sphere to 3D data
void ConicFit::fitSphere(const Points3D& P, Eigen::Vector3d& centre, double& radius)
{
	// Suppose the i-th data point is (xi,yi,zi). We try to fit a sphere of radius R and centre (cx,cy,cz) to the data. Let:
	//   A = N rows of [xi yi zi 1]                     {Nx4 matrix}
	//   x = [2*cx; 2*cy; 2*cz; R^2-cx^2-cy^2-cz^2]     {4x1 column vector}
	//   b = N rows of [xi^2 + yi^2 + zi^2]             {Nx1 column vector}
	// Ax = b is equivalent to the condition that all data points lie on a sphere of radius R, centered at (cx,cy,cz). Hence,
	// by solving Ax = b for x in a least squares sense, we are implicitly solving for the values of cx, cy, cz and R for
	// which the corresponding sphere most closely fits the data. We perform a mean shift on the data for numerical
	// conditioning and stability reasons.
	
	// Declare variables
	size_t N = P.size();
	Eigen::MatrixXd A(N, 4);
	Eigen::VectorXd b(N);
	
	// Calculate the mean of the data points
	Eigen::Vector3d mean = Eigen::Vector3d::Zero();
	if(N >= 1)
		mean = std::accumulate(P.begin(), P.end(), mean) / N;

	// Construct the linear equation to solve (Ax = b)
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector3d p = P[i] - mean;
		A(i, 0) = p.x();
		A(i, 1) = p.y();
		A(i, 2) = p.z();
		A(i, 3) = 1;
		b(i) = p.squaredNorm();
	}

	// Find the least squares solution to Ax = b using Singular Value Decomposition (linear least squares fitting)
	Eigen::VectorXd coefficients = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	// Extract the centre and radius of the fitted sphere
	centre = 0.5*coefficients.head<3>();
	radius = sqrt(coefficients(3) + centre.squaredNorm());

	// Correct for the applied mean shift
	centre += mean;
}

// Fit a sphere of known centre to 3D data
void ConicFit::fitSphereCentred(const Points3D& P, const Eigen::Vector3d& centre, double& radius)
{
	// Suppose the i-th data point is (xi,yi,zi). The least squares solution to finding a sphere of radius R of known
	// centre (cx,cy,cz) that best fits the data points has a simple algebraic solution, given by:
	//   R = sqrt(sum((xi-cx)^2 + (yi-cy)^2 + (zi-cz)^2, 1, N) / N)
	// In other words, the radius of best fit is the quadratic mean of the distances from the known centre to each point.
	
	// Declare variables
	size_t N = P.size();
	
	// Calculate the radius of best fit
	double Rsq = 0.0;
	for(size_t i = 0; i < N; i++)
		Rsq += (P[i] - centre).squaredNorm() / N;
	radius = sqrt(Rsq);
}

// Calculate the fitting error of a sphere to certain 3D data (average distance to the sphere in units of sphere radii)
double ConicFit::fitSphereError(const Points3D& P, const Eigen::Vector3d& centre, double radius)
{
	// Calculate and return the required dimensionless fitting error
	double err = 0.0;
	size_t N = P.size();
	if(radius != 0.0)
	{
		for(size_t i = 0; i < N; i++)
			err += fabs((P[i] - centre).norm()/radius - 1.0)/N;
	}
	else
	{
		for(size_t i = 0; i < N; i++)
			err += fabs((P[i] - centre).norm())/N;
	}
	return err;
}

//
// Ellipsoid fitting
//

// Fit an ellipsoid to 3D data
void ConicFit::fitEllipsoid(const Points3D& P, Eigen::Vector3d& centre, Eigen::Matrix3d& coeff)
{
	// We first compute the mean point (xm,ym,zm) of the data points, and bias the data points by the negative of this, resulting in zero mean data.
	// Suppose that the zero mean set of points is (xi,yi,zi). We then fit the following equation by least squares:
	//   ax^2 + by^2 + cz^2 + 2dxy + 2exz + 2fyz + 2gx + 2hy + 2iz = 1
	// We do this by expressing the least squares fitting problem as the matrix equation Dv = y, where:
	//   D = N rows of [xi^2 yi^2 zi^2 2*xi*yi 2*xi*zi 2*yi*zi 2*xi 2*yi 2*zi]    {Nx9 matrix}
	//   v = [a; b; c; d; e; f; g; h; i]                                          {9x1 column vector}
	//   y = N rows of [1]                                                        {Nx1 column vector}
	// The centre of the ellipsoid is at the point where the gradient of the defining function is zero. Therefore, accounting for (xm,ym,zm):
	//   [cx; cy; cz] = [xm; ym; zm] - [a d e; d b f; e f c] \ [g; h; i]          {Matlab style}
	// (cx,cy,cz) is the centre of the ellipsoid of best fit through the original data points. The coefficient matrix is then computed by fitting an
	// ellipsoid of known centre (cx,cy,cz) through the data points. If A is the resulting coefficient matrix, the ellipsoid of best fit is:
	//   (x-c)'A(x-c) = 1
	// where x = [x; y; z] and c = [cx; cy; cz] are column vectors.
	
	// Declare variables
	size_t N = P.size();
	Eigen::MatrixXd D(N, 9);
	Eigen::VectorXd y(N);
	
	// Calculate the mean of the data points
	Eigen::Vector3d mean = Eigen::Vector3d::Zero();
	if(N >= 1)
		mean = std::accumulate(P.begin(), P.end(), mean) / N;
	
	// Construct the linear equation to solve (Dv = y)
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector3d p = P[i] - mean;
		D(i, 0) = p.x()*p.x();
		D(i, 1) = p.y()*p.y();
		D(i, 2) = p.z()*p.z();
		D(i, 3) = 2.0*p.x()*p.y();
		D(i, 4) = 2.0*p.x()*p.z();
		D(i, 5) = 2.0*p.y()*p.z();
		D(i, 6) = 2.0*p.x();
		D(i, 7) = 2.0*p.y();
		D(i, 8) = 2.0*p.z();
		y(i) = 1.0;
	}
	
	// Find the least squares solution to Dv = y using Singular Value Decomposition (linear least squares fitting)
	Eigen::VectorXd vhat = D.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);
	
	// Construct the coefficient matrix and vector
	Eigen::Matrix3d A;
	A << vhat(0), vhat(3), vhat(4),
	     vhat(3), vhat(1), vhat(5),
	     vhat(4), vhat(5), vhat(2);
	Eigen::Vector3d b(vhat(6), vhat(7), vhat(8));
	
	// Calculate the centre of the fitted ellipse
	centre = mean - A.colPivHouseholderQr().solve(b);
	
	// Compute the coefficient matrix of the ellipsoid of best fit through the nominated centre
	fitEllipsoidCentred(P, centre, coeff);
}

// Fit an ellipsoid of known centre to 3D data
void ConicFit::fitEllipsoidCentred(const Points3D& P, const Eigen::Vector3d& centre, Eigen::Matrix3d& coeff)
{
	// We first bias the data to be relative to the centre (cx,cy) of the ellipsoid to fit. We then fit the following equation by least squares:
	//   ax^2 + by^2 + cz^2 + 2dxy + 2exz + 2fyz = 1
	// If we define A = [a d e; d b f; e f c] as the coefficient matrix, and let xc = [x; y; z] then this is equivalent to:
	//   xc'*A*xc = 1
	// Suppose that the i-th data point of biased data is (xi,yi,zi). We perform the ellipsoid fit by expressing the least squares fitting problem
	// as the matrix equation Dv = y, where:
	//   D = N rows of [xi^2 yi^2 zi^2 2*xi*yi 2*xi*zi 2*yi*zi]    {Nx6 matrix}
	//   v = [a; b; c; d; e; f]                                    {6x1 column vector}
	//   y = N rows of [1]                                         {Nx1 column vector}
	// The coefficient matrix A can be constructed from the solution v, after which the equation of the ellipsoid of best fit through the original
	// data becomes:
	//   (x-c)'A(x-c) = 1
	// where x = [x; y; z] and c = [cx; cy; cz] are column vectors, and c is the specified centre of the ellipsoid.
	
	// Declare variables
	size_t N = P.size();
	Eigen::MatrixXd D(N, 6);
	Eigen::VectorXd y(N);
	
	// Construct the linear equation to solve (Dv = y)
	for(size_t i = 0; i < N; i++)
	{
		const Eigen::Vector3d p = P[i] - centre;
		D(i, 0) = p.x()*p.x();
		D(i, 1) = p.y()*p.y();
		D(i, 2) = p.z()*p.z();
		D(i, 3) = 2.0*p.x()*p.y();
		D(i, 4) = 2.0*p.x()*p.z();
		D(i, 5) = 2.0*p.y()*p.z();
		y(i) = 1.0;
	}
	
	// Find the least squares solution to Dv = y using Singular Value Decomposition (linear least squares fitting)
	Eigen::VectorXd vhat = D.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);
	
	// Construct the coefficient matrix
	coeff << vhat(0), vhat(3), vhat(4),
	         vhat(3), vhat(1), vhat(5),
	         vhat(4), vhat(5), vhat(2);
}

// Calculate the fitting error of an ellipsoid to certain 3D data (average distance of normalised points to the unit sphere)
double ConicFit::fitEllipsoidErrorCoeff(const Points3D& P, const Eigen::Vector3d& centre, const Eigen::Matrix3d& coeff)
{
	// Declare variables
	Eigen::Matrix3d rotmat, transform;
	Eigen::Vector3d radii;
	
	// Calculate the transformation matrix
	double failed = (double) P.size();
	if(!ellipsoidMatrixToAxes(coeff, rotmat, radii)) return failed;
	if(!ellipsoidAxesToTransform(rotmat, radii, transform)) return failed;
	
	// Calculate the fitting error based on the centre and transformation matrix
	return fitEllipsoidError(P, centre, transform);
}

// Calculate the fitting error of an ellipsoid to certain 3D data (average distance of normalised points to the unit sphere)
double ConicFit::fitEllipsoidError(const Points3D& P, const Eigen::Vector3d& centre, const Eigen::Matrix3d& transform)
{
	// Calculate and return the required dimensionless fitting error
	double err = 0.0;
	size_t N = P.size();
	for(size_t i = 0; i < N; i++)
		err += fabs((transform*(P[i] - centre)).norm() - 1.0)/N;
	return err;
}

//
// Ellipse functions
//

// Calculate the rotation matrix, radii and rotation angle of an ellipse defined by a coefficient matrix
bool ConicFit::ellipseMatrixToAxes(const Eigen::Matrix2d& coeff, Eigen::Matrix2d& rotmat, Eigen::Vector2d& radii, double& angle)
{
	// If the input coefficient matrix (coeff) is A, the output rotation matrix (rotmat) is Q, and the output radii (radii) are R,
	// then if we let D = diag(1./R.^2), the outputs of this function are constructed so that A = QDQ', Q^-1 = Q', |Q| = 1 and R > 0.
	// The angle is expressed as the magnitude of the rotation Q as a counterclockwise rotation.
	
	// Ensure the coefficient matrix is symmetric (trust the lower triangle)
	Eigen::Matrix2d A = coeff;
	A(0,1) = A(1,0);
	
	// Compute the eigendecomposition of the coefficient matrix (the solver assumes the matrix is symmetric and trusts the lower triangle)
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> ES(A);
	
	// Retrieve the eigenvalues
	Eigen::Vector2d lambda = ES.eigenvalues();
	
	// Check and ensure that the eigenvalues are not negative
	for(size_t i = 0; i < 2; i++)
	{
		if(lambda(i) < -ZeroTol) return false;
		if(lambda(i) < 0.0) lambda(i) = 0.0;
	}
	
	// Retrieve the eigenvectors (columns of the matrix Q)
	Eigen::Matrix2d Q = ES.eigenvectors();
	
	// Ensure that the eigenvectors are unit norm
	for(size_t i = 0; i < 2; i++)
	{
		double norm = Q.col(i).norm();
		if(norm < ZeroTol) return false;
		Q.col(i) /= norm;
	}
	
	// Enforce the right hand rule for the eigenvectors
	if(Q.determinant() < 0.0)
		Q.col(1) = -Q.col(1);
	
	// Transcribe the rotation matrix of the ellipse
	rotmat = Q;
	
	// Calculate the radii of the ellipse
	radii.x() = 1.0 / sqrt(lambda.x());
	radii.y() = 1.0 / sqrt(lambda.y());
	
	// Calculate the angle of the ellipse
	angle = atan2(rotmat(1,0) + rotmat(1,1), rotmat(0,0) + rotmat(0,1)) - M_PI/4.0;
	if(angle <= -M_PI)
		angle += 2.0*M_PI;
	
	// Return success
	return true;
}

// Calculate the normalisation transformation matrix of an ellipse defined by its rotation matrix and radii
bool ConicFit::ellipseAxesToTransform(const Eigen::Matrix2d& rotmat, const Eigen::Vector2d& radii, Eigen::Matrix2d& transform)
{
	// If the input rotation matrix (rotmat) is Q, the input radii (radii) are R, and the output normalisation matrix (transform) is W,
	// then the corresponding coefficient matrix is A = QDQ' where D = diag(1./R.^2). The normalisation matrix is then the positive
	// definite symmetric solution to W*W = A (i.e. W = sqrtm(A)).
	
	// Check that none of the radii are negative or zero
	if(radii.x() <= 0.0 || radii.y() <= 0.0) return false;
	
	// Construct the required normalisation transformation matrix
	Eigen::Vector2d rootLambda(1.0/radii.x(), 1.0/radii.y());
	transform = rotmat * rootLambda.asDiagonal() * rotmat.inverse();
	
	// Return success
	return true;
}

//
// Ellipsoid functions
//

// Calculate the rotation matrix and radii of an ellipsoid defined by a coefficient matrix
bool ConicFit::ellipsoidMatrixToAxes(const Eigen::Matrix3d& coeff, Eigen::Matrix3d& rotmat, Eigen::Vector3d& radii)
{
	// If the input coefficient matrix (coeff) is A, the output rotation matrix (rotmat) is Q, and the output radii (radii) are R,
	// then if we let D = diag(1./R.^2), the outputs of this function are constructed so that A = QDQ', Q^-1 = Q', |Q| = 1 and R > 0.
	
	// Ensure the coefficient matrix is symmetric (trust the lower triangle)
	Eigen::Matrix3d A = coeff;
	A(0,1) = A(1,0);
	A(0,2) = A(2,0);
	A(1,2) = A(2,1);
	
	// Compute the eigendecomposition of the coefficient matrix (the solver assumes the matrix is symmetric and trusts the lower triangle)
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ES(A);
	
	// Retrieve the eigenvalues
	Eigen::Vector3d lambda = ES.eigenvalues();
	
	// Check and ensure that the eigenvalues are not negative
	for(size_t i = 0; i < 3; i++)
	{
		if(lambda(i) < -ZeroTol) return false;
		if(lambda(i) < 0.0) lambda(i) = 0.0;
	}
	
	// Retrieve the eigenvectors (columns of the matrix Q)
	Eigen::Matrix3d Q = ES.eigenvectors();
	
	// Ensure that the eigenvectors are unit norm
	for(size_t i = 0; i < 3; i++)
	{
		double norm = Q.col(i).norm();
		if(norm < ZeroTol) return false;
		Q.col(i) /= norm;
	}
	
	// Enforce the right hand rule for the eigenvectors
	if(Q.determinant() < 0.0)
		Q.col(2) = -Q.col(2);
	
	// Transcribe the rotation matrix of the ellipsoid
	rotmat = Q;
	
	// Calculate the radii of the ellipsoid
	radii.x() = 1.0 / sqrt(lambda.x());
	radii.y() = 1.0 / sqrt(lambda.y());
	radii.z() = 1.0 / sqrt(lambda.z());
	
	// Return success
	return true;
}

// Calculate the normalisation transformation matrix of an ellipsoid defined by its rotation matrix and radii
bool ConicFit::ellipsoidAxesToTransform(const Eigen::Matrix3d& rotmat, const Eigen::Vector3d& radii, Eigen::Matrix3d& transform)
{
	// If the input rotation matrix (rotmat) is Q, the input radii (radii) are R, and the output normalisation matrix (transform) is W,
	// then the corresponding coefficient matrix is A = QDQ' where D = diag(1./R.^2). The normalisation matrix is then the positive
	// definite symmetric solution to W*W = A (i.e. W = sqrtm(A)).
	
	// Check that none of the radii are negative or zero
	if(radii.x() <= 0.0 || radii.y() <= 0.0 || radii.z() <= 0.0) return false;
	
	// Construct the required normalisation transformation matrix
	Eigen::Vector3d rootLambda(1.0/radii.x(), 1.0/radii.y(), 1.0/radii.z());
	transform = rotmat * rootLambda.asDiagonal() * rotmat.inverse();
	
	// Return success
	return true;
}

//
// Conic data generation functions
//

// Generate circle data (returns N points)
void ConicFit::genCircleData(ConicFit::Points2D& P, size_t N, const Eigen::Vector2d& centre, double radius)
{
	// Generate the required data
	P.clear();
	if(N < 3) N = 3;
	Eigen::Vector2d tmp;
	for(size_t i = 0; i < N; i++)
	{
		double t = i * (4.0*M_PI / (N - 1));
		tmp << radius*cos(t), radius*sin(t);
		tmp += centre;
		P.push_back(tmp);
	}
}

// Generate ellipse data (returns N points)
void ConicFit::genEllipseData(ConicFit::Points2D& P, size_t N, const Eigen::Vector2d& centre, const Eigen::Vector2d& radii, double angle)
{
	// Generate the required data
	P.clear();
	if(N < 3) N = 3;
	Eigen::Vector2d tmp;
	Eigen::Matrix2d rotmat;
	double cangle = cos(angle);
	double sangle = sin(angle);
	rotmat << cangle, -sangle, sangle, cangle;
	for(size_t i = 0; i < N; i++)
	{
		double t = i * (4.0*M_PI / (N - 1));
		tmp << radii.x()*cos(t), radii.y()*sin(t);
		tmp = rotmat * tmp;
		tmp += centre;
		P.push_back(tmp);
	}
}

// Generate sphere data (returns N^2 points)
void ConicFit::genSphereData(ConicFit::Points3D& PP, size_t N, const Eigen::Vector3d& centre, double radius)
{
	// Generate the required data
	PP.clear();
	if(N < 3) N = 3;
	Eigen::Vector3d tmp;
	for(size_t i = 0; i < N; i++)
	{
		double u = i * (2.0*M_PI / (N - 1));
		for(size_t j = 0; j < N; j++)
		{
			double v = j * (M_PI / (N - 1));
			tmp << radius*cos(u)*sin(v), radius*sin(u)*sin(v), radius*cos(v);
			tmp += centre;
			PP.push_back(tmp);
		}
	}
}

// Generate ellipsoid data (returns N^2 points)
void ConicFit::genEllipsoidData(ConicFit::Points3D& PP, size_t N, const Eigen::Vector3d& centre, const Eigen::Vector3d& radii, const Eigen::Matrix3d& rotmat)
{
	// Generate the required data
	PP.clear();
	if(N < 3) N = 3;
	Eigen::Vector3d tmp;
	for(size_t i = 0; i < N; i++)
	{
		double u = i * (2.0*M_PI / (N - 1));
		for(size_t j = 0; j < N; j++)
		{
			double v = j * (M_PI / (N - 1));
			tmp << radii.x()*cos(u)*sin(v), radii.y()*sin(u)*sin(v), radii.z()*cos(v);
			tmp = rotmat * tmp;
			tmp += centre;
			PP.push_back(tmp);
		}
	}
}
// EOF