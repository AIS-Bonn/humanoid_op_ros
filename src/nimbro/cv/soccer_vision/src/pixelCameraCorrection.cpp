// Soccer Vision node
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#include "pixelCameraCorrection.h"
#include "globaldefinitions.h"
#include "camera_parameters.h"

using namespace std;
using namespace soccervision;



// Convert from a pixel coordinate (px,py) to camera frame vector (a,b,1)
// Although this is not strictly enforced, the given pixel should be within the image dimensions.
// Calling this function with a pixel too far outside the image might have unexpected results.
// The stopping condition for the Newton-Raphson method is ||fx*F(a,b)|| < tol.
// This function works with doubles instead of floats so as to minimise the effects of round-off/truncation errors.
tf::Vector3 soccervision::pixelCameraCorrectionOriginalSize(float px, float py, float tol) // Note: Input specified in full-size pixel coordinates!
{
	// Calculate the target coordinates
	double xin = ((double) px-CamParam::cx)/CamParam::fx;
	double yin = ((double) py-CamParam::cy)/CamParam::fy;

	// Initial guess is centre of image (avoids a potentially bad starting guess from affecting the numerical solution performance)
	double a = 0;
	double b = 0;

	// Evaluate how good the initial guess is
	double r2 = a*a + b*b;
	double pn = 1.0 + r2*(CamParam::k1 + r2*(CamParam::k2 + r2*CamParam::k3));
	double pd = 1.0 + r2*(CamParam::k4 + r2*(CamParam::k5 + r2*CamParam::k6));
	double kr = pn/pd;
	double kab = 2.0*a*b;
	double Fx = a*kr + CamParam::p1*kab + CamParam::p2*(r2 + 2.0*a*a) - xin;
	double Fy = b*kr + CamParam::p2*kab + CamParam::p1*(r2 + 2.0*b*b) - yin;
	double f = 0.5*(Fx*Fx + Fy*Fy);

	// Calculate stopping tolerance
	const double ftol = 0.5*tol*tol/(CamParam::fx*CamParam::fx);

	// Check stopping tolerance
	if(f < ftol)
		return tf::Vector3(a, b, 1.0);

	// Declare variables
	double c1, c2, c3, dFxda, dFxdb, dFyda, dFydb, Jdet;
	
	// Apply Newton-Raphson method
	for(int k = 1;k <= 20;k++) // Maximum of 20 steps! (never needs more than 5 anyway)
	{
		// Evaluate function partial derivatives
		c1 = 2.0*(CamParam::k1 + 2.0*CamParam::k2*r2 + 3.0*CamParam::k3*r2*r2);
		c2 = 2.0*(CamParam::k4 + 2.0*CamParam::k5*r2 + 3.0*CamParam::k6*r2*r2);
		c3 = (c1*pd - c2*pn)/(pd*pd);
		dFxda = kr + a*a*c3 + 2.0*CamParam::p1*b + 6.0*CamParam::p2*a;
		dFydb = kr + b*b*c3 + 2.0*CamParam::p2*a + 6.0*CamParam::p1*b;
		dFxdb = a*b*c3 + 2.0*(CamParam::p1*a + CamParam::p2*b);
		dFyda = dFxdb;

		// Calculate required adjustment
		Jdet = dFxda*dFydb - dFxdb*dFyda;
		if(fabs(Jdet) < 1e-12) // Problem: Encountered a singular Jacobian - should never happen
			return tf::Vector3(a, b, 1.0);

		// Always take a full step
		a += (dFxdb*Fy - dFydb*Fx)/Jdet;
		b += (dFyda*Fx - dFxda*Fy)/Jdet;

		// Evaluate how good the updated guess is
		r2 = a*a + b*b;
		pn = 1.0 + r2*(CamParam::k1 + r2*(CamParam::k2 + r2*CamParam::k3));
		pd = 1.0 + r2*(CamParam::k4 + r2*(CamParam::k5 + r2*CamParam::k6));
		kr = pn/pd;
		kab = 2.0*a*b;
		Fx = a*kr + CamParam::p1*kab + CamParam::p2*(r2 + 2.0*a*a) - xin;
		Fy = b*kr + CamParam::p2*kab + CamParam::p1*(r2 + 2.0*b*b) - yin;
		f = 0.5*(Fx*Fx + Fy*Fy);

		// Check stopping tolerance
		if(f < ftol)
			return tf::Vector3(a, b, 1.0);
	}

	// Failed: Maximum number of iterations was reached, just return what we've got at the moment
	return tf::Vector3(a, b, 1.0);
}

// Convert from a subsampled pixel coordinate (px,py) to camera frame vector (a,b,1)
// Although this is not strictly enforced, the given pixel should be within the subsampled image dimensions.
// Calling this function with a pixel too far outside the image might have unexpected results.
tf::Vector3 soccervision::pixelCameraCorrection(float px, float py, float tol) // Note: Input specified in subsampled pixel coordinates!
{
	// Scale back the pixel to its original size
	return pixelCameraCorrectionOriginalSize(px*SUB_SAMPLING_PARAMETER, py*SUB_SAMPLING_PARAMETER, tol);
}

// Convert from a camera frame vector (x,y,z) to a full-size pixel coordinate (px,py)
// The input z-component must be strictly positive, but otherwise there is no restriction.
// The output is (px,py,0)
tf::Vector3 soccervision::cameraPixelCorrectionOriginalSize(tf::Vector3 camvec, bool useLinExt)
{
	// Verify that z is strictly positive
	if(camvec.z() <= 0.0)
		return tf::Vector3(-1.0, -1.0, 0.0);

	// Normalise the z component of the input vector [resulting vector is (a,b,1)]
	float a = camvec.x()/camvec.z();
	float b = camvec.y()/camvec.z();
	float olda = a;
	float oldb = b;
	
	// Saturate a and b if using linear extension
	if(useLinExt)
	{
		if(a > CamParam::apo) a = CamParam::apo;
		if(a < CamParam::ano) a = CamParam::ano;
		if(b > CamParam::bpo) b = CamParam::bpo;
		if(b < CamParam::bno) b = CamParam::bno;
	}

	// Calculate additional parameters required for the distortion equations
	float r2 = a*a + b*b;
	float kr = (1.0 + r2*(CamParam::k1 + r2*(CamParam::k2 + r2*CamParam::k3)))/(1.0 + r2*(CamParam::k4 + r2*(CamParam::k5 + r2*CamParam::k6)));
	float kab = 2.0*a*b;

	// Adjust for tangential and radial distortions
	float x = kr*a + kab*CamParam::p1 + (r2+2.0*a*a)*CamParam::p2;
	float y = kr*b + kab*CamParam::p2 + (r2+2.0*b*b)*CamParam::p1;

	// Adjust for focal lengths and offsets
	float px = CamParam::fx*x+CamParam::cx;
	float py = CamParam::fy*y+CamParam::cy;

	// Extend the distortion function linearly if desired
	if(useLinExt)
	{
		// Calculate lambda values (0 => Use raw value / 1 => Use linear extension)
		float alambda = (olda >= 0.0 ? (olda-CamParam::api)/(CamParam::apo-CamParam::api) : (olda-CamParam::ani)/(CamParam::ano-CamParam::ani));
		float blambda = (oldb >= 0.0 ? (oldb-CamParam::bpi)/(CamParam::bpo-CamParam::bpi) : (oldb-CamParam::bni)/(CamParam::bno-CamParam::bni));
		if(alambda < 0.0) alambda = 0.0;
		if(alambda > 1.0) alambda = 1.0;
		if(blambda < 0.0) blambda = 0.0;
		if(blambda > 1.0) blambda = 1.0;

		// Calculate linear extension in x direction
		px = (1.0-alambda)*px + alambda*(CamParam::mx*olda+CamParam::bx);
		py = (1.0-blambda)*py + blambda*(CamParam::my*oldb+CamParam::by);
	}

	// Return the required pixel coordinate
	return tf::Vector3(px,py,0.0);
}

// Convert from a camera frame vector (x,y,z) to a subsampled pixel coordinate (px,py)
// The input z-component must be strictly positive, but otherwise there is no restriction.
tf::Vector3 soccervision::cameraPixelCorrection(tf::Vector3 camvec, bool useLinExt)
{
	// Scale back the pixel to its subsampled size
	return cameraPixelCorrectionOriginalSize(camvec, useLinExt)/SUB_SAMPLING_PARAMETER;
}

// Compute the ego world vector (point on ground) corresponding to a camera frame vector
tf::Vector3 soccervision::compute_ego_position(tf::Matrix3x3 rotMat, tf::Vector3 trunkVec, tf::Vector3 vecCam2Point)
{
	// Transform the camera frame vector using the current pose of the camera on the robot
	tf::Vector3 vecRobotEyeToPointRot = rotMat*vecCam2Point;
	float lambda = (0.0 - trunkVec.z() - 0.61)/vecRobotEyeToPointRot.z(); // TODO: Is 0.61 still correct?
	tf::Vector3 ego_ball_vector = trunkVec + lambda*vecRobotEyeToPointRot;
	return ego_ball_vector;
}

// Compute the ego world vector (point in shoulder plane) corresponding to a camera frame vector
tf::Vector3 soccervision::compute_bodymark_ego_position(tf::Matrix3x3 rotMat, tf::Vector3 trunkVec, tf::Vector3 vecCam2Point)
{
	// Transform the camera frame vector using the current pose of the camera on the robot
	tf::Vector3 vecRobotEyeToPointRot = rotMat*vecCam2Point;
	float lambda = (0.0 - trunkVec.z() + 0.14)/vecRobotEyeToPointRot.z(); // TODO: Is 0.14 still correct?

	// Returning a zero vector will ensure that the bodymark is not taken into account because it will be too close to the robot
	if(lambda < 0.0)
		return tf::Vector3(0.0,0.0,0.0);

	// Return the required vector
	tf::Vector3 ego_ball_vector = trunkVec + lambda*vecRobotEyeToPointRot;
	return ego_ball_vector;
}
