// Rotations conversion library
// File: rot_conv.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rot_conv/rot_conv.h>
#include <cmath>

// Defines
#define M_2PI (2.0*M_PI)

// Rotations conversion namespace
namespace rot_conv
{
	//
	// Conversions from quaternions
	//
	
	// These conversion routines assume that the input quaternions are of unit norm.
	// Ensure that you normalise quaternions for which this may not be the case.
	
	// Conversion: Quaternion --> Fused angles (2D)
	void FusedFromQuat(const Quat& q, double& fPitch, double& fRoll)
	{
		// Calculate the fused pitch and roll
		double stheta = 2.0*(q.y()*q.w() - q.x()*q.z());
		double sphi   = 2.0*(q.y()*q.z() + q.x()*q.w());
		stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
		sphi   = (sphi   >= 1.0 ? 1.0 : (sphi   <= -1.0 ? -1.0 : sphi  )); // Coerce sphi   to [-1,1]
		fPitch = asin(stheta);
		fRoll  = asin(sphi);
	}
	
	// Conversion: Quaternion --> Fused angles (3D)
	void FusedFromQuat(const Quat& q, double& fYaw, double& fPitch, double& fRoll)
	{
		// Calculate and wrap the fused yaw
		fYaw = 2.0*atan2(q.z(), q.w());  // Output of atan2 is [-pi,pi], so this expression is in [-2*pi,2*pi]
		if(fYaw > M_PI) fYaw -= M_2PI;   // fYaw is now in [-2*pi,pi]
		if(fYaw <= -M_PI) fYaw += M_2PI; // fYaw is now in (-pi,pi]

		// Calculate the fused pitch and roll
		double stheta = 2.0*(q.y()*q.w() - q.x()*q.z());
		double sphi   = 2.0*(q.y()*q.z() + q.x()*q.w());
		stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
		sphi   = (sphi   >= 1.0 ? 1.0 : (sphi   <= -1.0 ? -1.0 : sphi  )); // Coerce sphi   to [-1,1]
		fPitch = asin(stheta);
		fRoll  = asin(sphi);
	}
	
	// Conversion: Quaternion --> Fused angles (4D)
	void FusedFromQuat(const Quat& q, double& fYaw, double& fPitch, double& fRoll, bool& hemi)
	{
		// Calculate and wrap the fused yaw
		fYaw = 2.0*atan2(q.z(), q.w());  // Output of atan2 is [-pi,pi], so this expression is in [-2*pi,2*pi]
		if(fYaw > M_PI) fYaw -= M_2PI;   // fYaw is now in [-2*pi,pi]
		if(fYaw <= -M_PI) fYaw += M_2PI; // fYaw is now in (-pi,pi]

		// Calculate the fused pitch and roll
		double stheta = 2.0*(q.y()*q.w() - q.x()*q.z());
		double sphi   = 2.0*(q.y()*q.z() + q.x()*q.w());
		stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
		sphi   = (sphi   >= 1.0 ? 1.0 : (sphi   <= -1.0 ? -1.0 : sphi  )); // Coerce sphi   to [-1,1]
		fPitch = asin(stheta);
		fRoll  = asin(sphi);

		// Calculate the hemisphere of the rotation
		hemi = (0.5 - (q.x()*q.x() + q.y()*q.y()) >= 0.0);
	}
	
	//
	// Conversions from fused angles
	//
	
	// Conversion: Fused angles (2D) --> Quaternion
	Quat QuatFromFused(double fPitch, double fRoll) // Assume: fYaw = 0, hemi = true
	{
		// Precalculate the sin values
		double sth  = sin(fPitch);
		double sphi = sin(fRoll);

		// Calculate the sine sum criterion
		double crit = sth*sth + sphi*sphi;

		// Calculate the tilt angle alpha
		double alpha = (crit >= 1.0 ? M_PI_2 : acos(sqrt(1.0 - crit)));
		double halpha  = 0.5*alpha;
		double chalpha = cos(halpha);
		double shalpha = sin(halpha);

		// Calculate the tilt axis gamma
		double gamma = atan2(sth,sphi);
		double cgamma = cos(gamma);
		double sgamma = sin(gamma);

		// Return the required quaternion orientation (a rotation about (cgamma, sgamma, 0) by angle alpha)
		return Quat(chalpha, cgamma*shalpha, sgamma*shalpha, 0.0); // Order: (w,x,y,z)
	}
	
	// Conversion: Fused angles (3D/4D) --> Quaternion
	Quat QuatFromFused(double fYaw, double fPitch, double fRoll, bool hemi)
	{
		// Precalculate the sin values
		double sth  = sin(fPitch);
		double sphi = sin(fRoll);

		// Calculate the sine sum criterion
		double crit = sth*sth + sphi*sphi;

		// Calculate the tilt angle alpha
		double alpha;
		if(crit >= 1.0)
			alpha = M_PI_2;
		else
			alpha = acos(hemi ? sqrt(1.0-crit) : -sqrt(1.0-crit));

		// Calculate the tilt axis gamma
		double gamma = atan2(sth,sphi);

		// Evaluate the required intermediate angles
		double halpha  = 0.5*alpha;
		double hpsi    = 0.5*fYaw;
		double hgampsi = gamma + hpsi;

		// Precalculate trigonometric terms involved in the quaternion expression
		double chalpha = cos(halpha);
		double shalpha = sin(halpha);
		double chpsi = cos(hpsi);
		double shpsi = sin(hpsi);
		double chgampsi = cos(hgampsi);
		double shgampsi = sin(hgampsi);
		
		// Calculate and return the required quaternion
		return Quat(chalpha*chpsi, shalpha*chgampsi, shalpha*shgampsi, chalpha*shpsi); // Order: (w,x,y,z)
	}
}
// EOF