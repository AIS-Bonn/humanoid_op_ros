// Rotations conversion library of functions
// File: rot_conv.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef ROT_CONV_H
#define ROT_CONV_H

// Includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// Rotations conversion namespace
namespace rot_conv
{
	//
	// Rotation types
	//

	// Typedefs
	typedef Eigen::Quaterniond Quat; // Format: (w,x,y,z)

	// Fused angles struct
	struct Fused // Format: (fYaw, fPitch, fRoll, hemi)
	{
		// Constructors
		Fused() {}
		Fused(double fPitch, double fRoll) : fYaw(0.0), fPitch(fPitch), fRoll(fRoll), hemi(true) {}
		Fused(double fYaw, double fPitch, double fRoll, double hemi = true) : fYaw(fYaw), fPitch(fPitch), fRoll(fRoll), hemi(hemi) {}

		// Data members
		double fYaw;   // Fused yaw:    psi  = yaw   is in (-pi,pi]
		double fPitch; // Fused pitch: theta = pitch is in [-pi/2,pi/2]
		double fRoll;  // Fused roll:   phi  = roll  is in [-pi/2,pi/2]
		bool hemi;     // Hemisphere:    h   = hemi  is in {-1,1} (stored as {false,true} respectively)
	};
	
	//
	// Conversions from quaternions
	//

	// Conversion: Quaternion --> Fused angles
	void FusedFromQuat(const Quat& q, double& fPitch, double& fRoll);
	void FusedFromQuat(const Quat& q, double& fYaw, double& fPitch, double& fRoll);
	void FusedFromQuat(const Quat& q, double& fYaw, double& fPitch, double& fRoll, bool& hemi);
	inline Fused FusedFromQuat(const Quat& q) { Fused f; FusedFromQuat(q, f.fYaw, f.fPitch, f.fRoll, f.hemi); return f; }
	
	//
	// Conversions from fused angles
	//

	// Conversion: Fused angles --> Quaternion
	Quat QuatFromFused(double fPitch, double fRoll);
	Quat QuatFromFused(double fYaw, double fPitch, double fRoll, bool hemi = true);
	inline Quat QuatFromFused(const Fused& f) { return QuatFromFused(f.fYaw, f.fPitch, f.fRoll, f.hemi); }
}

#endif /* ROT_CONV_H */
// EOF