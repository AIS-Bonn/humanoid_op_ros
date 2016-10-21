// Rotations conversion library of functions
// File: rot_conv.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef ROT_CONV_H
#define ROT_CONV_H

// Includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// Defines
#define ROT_CONV_DEF_TOL  1e-10

// Note: The inline keywords below for the functions that are defined (as well as declared) in this header are
//       required to avoid multiple definition linker errors.
// Note: Where quaternions and rotation matrices are used as inputs to the conversion functions, it is implicitly
//       assumed that they are valid (i.e. unit norm q, orthogonal R, det(R) = 1, etc...)

// Rotations conversion namespace
namespace rot_conv
{
	// ########################
	// #### Rotation types ####
	// ########################

	// Typedefs
	typedef Eigen::Matrix3d Rotmat;
	typedef Eigen::Quaterniond Quat; // Format: (w,x,y,z)
	typedef Eigen::Vector3d ZVec;
	typedef Eigen::Vector3d Vec3;

	// Stream insertion operator for the Quat type
	inline std::ostream& operator<<(std::ostream& os, const Quat& q) { return os << "(" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << ")"; }

	// Euler angles struct
	struct EulerAngles // Format: ZYX Euler (yaw, pitch, roll)
	{
		// Constants
		static const EulerAngles Identity;

		// Constructors
		EulerAngles() {}
		EulerAngles(double yaw) : yaw(yaw), pitch(0.0), roll(0.0) {}
		EulerAngles(double yaw, double pitch, double roll) : yaw(yaw), pitch(pitch), roll(roll) {}

		// Misc functions
		void set(double yaw, double pitch, double roll) { this->yaw = yaw; this->pitch = pitch; this->roll = roll; }
		void setIdentity() { yaw = pitch = roll = 0.0; }

		// Data members
		double yaw;   // Yaw:    psi  is in (-pi,pi]
		double pitch; // Pitch: theta is in [-pi/2,pi/2]
		double roll;  // Roll:   phi  is in (-pi,pi]
	};

	// Stream insertion operator for the EulerAngles struct
	inline std::ostream& operator<<(std::ostream& os, const EulerAngles& e) { return os << "(" << e.yaw << ", " << e.pitch << ", " << e.roll << ")"; }

	// Fused angles struct
	struct FusedAngles // Format: (fusedYaw, fusedPitch, fusedRoll, hemi)
	{
		// Constants
		static const FusedAngles Identity;

		// Constructors
		FusedAngles() {}
		FusedAngles(double fusedYaw) : fusedYaw(fusedYaw), fusedPitch(0.0), fusedRoll(0.0), hemi(true) {}
		FusedAngles(double fusedPitch, double fusedRoll, bool hemi = true) : fusedYaw(0.0), fusedPitch(fusedPitch), fusedRoll(fusedRoll), hemi(hemi) {}
		FusedAngles(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi = true) : fusedYaw(fusedYaw), fusedPitch(fusedPitch), fusedRoll(fusedRoll), hemi(hemi) {}

		// Misc functions
		void set(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi = true) { this->fusedYaw = fusedYaw; this->fusedPitch = fusedPitch; this->fusedRoll = fusedRoll; this->hemi = hemi; }
		void setIdentity() { fusedYaw = fusedPitch = fusedRoll = 0.0; hemi = true; }

		// Data members
		double fusedYaw;   // Fused yaw:    psi  = yaw   is in (-pi,pi]
		double fusedPitch; // Fused pitch: theta = pitch is in [-pi/2,pi/2]
		double fusedRoll;  // Fused roll:   phi  = roll  is in [-pi/2,pi/2]
		bool hemi;         // Hemisphere:    h   = hemi  is in {-1,1} (stored as {false,true} respectively)
	};

	// Stream insertion operator for the FusedAngles struct
	inline std::ostream& operator<<(std::ostream& os, const FusedAngles& f) { return os << "(" << f.fusedYaw << ", " << f.fusedPitch << ", " << f.fusedRoll << ", " << (f.hemi ? "1" : "-1") << ")"; }

	// Tilt angles struct
	struct TiltAngles // Format: (fusedYaw, tiltAxisAngle, tiltAngle)
	{
		// Constants
		static const TiltAngles Identity;

		// Constructors
		TiltAngles() {}
		TiltAngles(double fusedYaw) : fusedYaw(fusedYaw), tiltAxisAngle(0.0), tiltAngle(0.0) {}
		TiltAngles(double tiltAxisAngle, double tiltAngle) : fusedYaw(0.0), tiltAxisAngle(tiltAxisAngle), tiltAngle(tiltAngle) {}
		TiltAngles(double fusedYaw, double tiltAxisAngle, double tiltAngle) : fusedYaw(fusedYaw), tiltAxisAngle(tiltAxisAngle), tiltAngle(tiltAngle) {}

		// Misc functions
		void set(double fusedYaw, double tiltAxisAngle, double tiltAngle) { this->fusedYaw = fusedYaw; this->tiltAxisAngle = tiltAxisAngle; this->tiltAngle = tiltAngle; }
		void setIdentity() { fusedYaw = tiltAxisAngle = tiltAngle = 0.0; }

		// Data members
		double fusedYaw;      // Fused yaw:        psi  is in (-pi,pi]
		double tiltAxisAngle; // Tilt axis angle: gamma is in (-pi,pi]
		double tiltAngle;     // Tilt angle:      alpha is in [0,pi]
	};

	// Stream insertion operator for the TiltAngles struct
	inline std::ostream& operator<<(std::ostream& os, const TiltAngles& t) { return os << "(" << t.fusedYaw << ", " << t.tiltAxisAngle << ", " << t.tiltAngle << ")"; }

	// ##########################################
	// #### Rotation checking and validation ####
	// ##########################################

	// Check and validate: Rotation matrix
	bool ValidateRotmat(Rotmat& R, double tol = ROT_CONV_DEF_TOL);
	inline bool CheckRotmat(const Rotmat& R, double tol = ROT_CONV_DEF_TOL) { Rotmat RR = R; return ValidateRotmat(RR, tol); }
	inline bool CheckRotmat(const Rotmat& R, Rotmat& Rout, double tol = ROT_CONV_DEF_TOL) { Rout = R; return ValidateRotmat(Rout, tol); }

	// Check and validate: Quaternion
	bool ValidateQuat(Quat& q, double tol = ROT_CONV_DEF_TOL, bool unique = false);
	inline bool CheckQuat(const Quat& q, double tol = ROT_CONV_DEF_TOL, bool unique = false) { Quat qq = q; return ValidateQuat(qq, tol, unique); }
	inline bool CheckQuat(const Quat& q, Quat& qout, double tol = ROT_CONV_DEF_TOL, bool unique = false) { qout = q; return ValidateQuat(qout, tol, unique); }

	// Check and validate: Euler angles
	bool ValidateEuler(EulerAngles& e, double tol = ROT_CONV_DEF_TOL, bool unique = false);
	inline bool CheckEuler(const EulerAngles& e, double tol = ROT_CONV_DEF_TOL, bool unique = false) { EulerAngles ee = e; return ValidateEuler(ee, tol, unique); }
	inline bool CheckEuler(const EulerAngles& e, EulerAngles& eout, double tol = ROT_CONV_DEF_TOL, bool unique = false) { eout = e; return ValidateEuler(eout, tol, unique); }

	// Check and validate: Fused angles
	bool ValidateFused(FusedAngles& f, double tol = ROT_CONV_DEF_TOL, bool unique = false);
	inline bool CheckFused(const FusedAngles& f, double tol = ROT_CONV_DEF_TOL, bool unique = false) { FusedAngles ff = f; return ValidateFused(ff, tol, unique); }
	inline bool CheckFused(const FusedAngles& f, FusedAngles& fout, double tol = ROT_CONV_DEF_TOL, bool unique = false) { fout = f; return ValidateFused(fout, tol, unique); }

	// Check and validate: Tilt angles
	bool ValidateTilt(TiltAngles& t, double tol = ROT_CONV_DEF_TOL, bool unique = false);
	inline bool CheckTilt(const TiltAngles& t, double tol = ROT_CONV_DEF_TOL, bool unique = false) { TiltAngles tt = t; return ValidateTilt(tt, tol, unique); }
	inline bool CheckTilt(const TiltAngles& t, TiltAngles& tout, double tol = ROT_CONV_DEF_TOL, bool unique = false) { tout = t; return ValidateTilt(tout, tol, unique); }

	// ###########################
	// #### Rotation equality ####
	// ###########################

	// Check equality: Rotation matrix
	bool RotmatEqual(const Rotmat& Ra, const Rotmat& Rb, double tol = ROT_CONV_DEF_TOL);

	// Check equality: Quaternion
	bool QuatEqual(const Quat& qa, const Quat& qb, double tol = ROT_CONV_DEF_TOL);

	// Check equality: Euler angles
	bool EulerEqual(const EulerAngles& ea, const EulerAngles& eb, double tol = ROT_CONV_DEF_TOL);

	// Check equality: Fused angles
	bool FusedEqual(const FusedAngles& fa, const FusedAngles& fb, double tol = ROT_CONV_DEF_TOL);

	// Check equality: Tilt angles
	bool TiltEqual(const TiltAngles& ta, const TiltAngles& tb, double tol = ROT_CONV_DEF_TOL);

	// #########################
	// #### Yaw of rotation ####
	// #########################

	// Yaw of: Rotation matrix
	double EYawOfRotmat(const Rotmat& R);
	double FYawOfRotmat(const Rotmat& R);

	// Yaw of: Quaternion
	double EYawOfQuat(const Quat& q);
	double FYawOfQuat(const Quat& q);

	// Yaw of: Euler angles
	inline double EYawOfEuler(const EulerAngles& e) { return e.yaw; }
	double FYawOfEuler(const EulerAngles& e);

	// Yaw of: Fused angles
	double EYawOfFused(const FusedAngles& f);
	inline double FYawOfFused(const FusedAngles& f) { return f.fusedYaw; }

	// Yaw of: Tilt angles
	double EYawOfTilt(const TiltAngles& t);
	inline double FYawOfTilt(const TiltAngles& t) { return t.fusedYaw; }

	// ##################################
	// #### Remove yaw from rotation ####
	// ##################################

	// Remove yaw from: Rotation matrix
	void RotmatNoEYaw(const Rotmat& R, Rotmat& Rout);
	void RotmatNoFYaw(const Rotmat& R, Rotmat& Rout);
	inline Rotmat RotmatNoEYaw(const Rotmat& R) { Rotmat Rout; RotmatNoEYaw(R, Rout); return Rout; }
	inline Rotmat RotmatNoFYaw(const Rotmat& R) { Rotmat Rout; RotmatNoFYaw(R, Rout); return Rout; }

	// Remove yaw from: Quaternion
	void QuatNoEYaw(const Quat& q, Quat& qout);
	void QuatNoFYaw(const Quat& q, Quat& qout);
	inline Quat QuatNoEYaw(const Quat& q) { Quat qout; QuatNoEYaw(q, qout); return qout; }
	inline Quat QuatNoFYaw(const Quat& q) { Quat qout; QuatNoFYaw(q, qout); return qout; }

	// Remove yaw from: Euler angles
	inline void EulerNoEYaw(const EulerAngles& e, EulerAngles& eout) { eout.set(0.0, e.pitch, e.roll); }
	void EulerNoFYaw(const EulerAngles& e, EulerAngles& eout);
	inline EulerAngles EulerNoEYaw(const EulerAngles& e) { return EulerAngles(0.0, e.pitch, e.roll); }
	inline EulerAngles EulerNoFYaw(const EulerAngles& e) { EulerAngles eout; EulerNoFYaw(e, eout); return eout; }

	// Remove yaw from: Fused angles
	void FusedNoEYaw(const FusedAngles& f, FusedAngles& fout);
	inline void FusedNoFYaw(const FusedAngles& f, FusedAngles& fout) { fout.set(0.0, f.fusedPitch, f.fusedRoll, f.hemi); }
	inline FusedAngles FusedNoEYaw(const FusedAngles& f) { FusedAngles fout; FusedNoEYaw(f, fout); return fout; }
	inline FusedAngles FusedNoFYaw(const FusedAngles& f) { return FusedAngles(f.fusedPitch, f.fusedRoll, f.hemi); }

	// Remove yaw from: Tilt angles
	void TiltNoEYaw(const TiltAngles& t, TiltAngles& tout);
	inline void TiltNoFYaw(const TiltAngles& t, TiltAngles& tout) { tout.set(0.0, t.tiltAxisAngle, t.tiltAngle); }
	inline TiltAngles TiltNoEYaw(const TiltAngles& t) { TiltAngles tout; TiltNoEYaw(t, tout); return tout; }
	inline TiltAngles TiltNoFYaw(const TiltAngles& t) { return TiltAngles(t.tiltAxisAngle, t.tiltAngle); }

	// ###########################
	// #### Rotation inverses ####
	// ###########################

	// Inverse: Rotation matrix
	void RotmatInv(const Rotmat& R, Rotmat& Rinv);
	inline Rotmat RotmatInv(const Rotmat& R) { Rotmat Rinv; RotmatInv(R, Rinv); return Rinv; }

	// Inverse: Quaternion
	void QuatInv(const Quat& q, Quat& qinv);
	inline Quat QuatInv(const Quat& q) { Quat qinv; QuatInv(q, qinv); return qinv; }

	// Inverse: Euler angles
	void EulerInv(const EulerAngles& e, EulerAngles& einv);
	inline EulerAngles EulerInv(const EulerAngles& e) { EulerAngles einv; EulerInv(e, einv); return einv; }

	// Inverse: Fused angles
	void FusedInv(const FusedAngles& f, FusedAngles& finv);
	inline FusedAngles FusedInv(const FusedAngles& f) { FusedAngles finv; FusedInv(f, finv); return finv; }

	// Inverse: Tilt angles
	void TiltInv(const TiltAngles& t, TiltAngles& tinv);
	inline TiltAngles TiltInv(const TiltAngles& t) { TiltAngles tinv; TiltInv(t, tinv); return tinv; }

	// ##########################
	// #### Vector rotations ####
	// ##########################

	// Rotate vector by: Rotation matrix
	Vec3 RotmatRotVec(const Rotmat& R, const Vec3& v);

	// Rotate vector by: Quaternion
	Vec3 QuatRotVec(const Quat& q, const Vec3& v);

	// Rotate vector by: Euler angles
	Vec3 EulerRotVec(const EulerAngles& e, const Vec3& v);

	// Rotate vector by: Fused angles
	Vec3 FusedRotVec(const FusedAngles& f, const Vec3& v);

	// Rotate vector by: Tilt angles
	Vec3 TiltRotVec(const TiltAngles& t, const Vec3& v);

	// ##############################
	// #### Pure yaw conversions ####
	// ##############################

	// Conversion: Pure yaw --> Rotation matrix
	void RotmatFromYaw(double yaw, Rotmat& R);
	inline Rotmat RotmatFromYaw(double yaw) { Rotmat R; RotmatFromYaw(yaw, R); return R; }

	// Conversion: Pure yaw --> Quaternion
	void QuatFromYaw(double yaw, Quat& q);
	inline Quat QuatFromYaw(double yaw) { Quat q; QuatFromYaw(yaw, q); return q; }

	// ############################################
	// #### Conversions from rotation matrices ####
	// ############################################

	// Conversion: Rotation matrix --> Quaternion
	void QuatFromRotmat(const Rotmat& R, Quat& q);
	inline Quat QuatFromRotmat(const Rotmat& R) { Quat q; QuatFromRotmat(R, q); return q; }

	// Conversion: Rotation matrix --> Euler angles
	inline void EulerFromRotmat(const Rotmat& R, double& yaw) { yaw = EYawOfRotmat(R); }
	void EulerFromRotmat(const Rotmat& R, double& yaw, double& pitch, double& roll);
	inline void EulerFromRotmat(const Rotmat& R, EulerAngles& e) { EulerFromRotmat(R, e.yaw, e.pitch, e.roll); }
	inline EulerAngles EulerFromRotmat(const Rotmat& R) { EulerAngles e; EulerFromRotmat(R, e.yaw, e.pitch, e.roll); return e; }

	// Conversion: Rotation matrix --> Fused angles
	inline void FusedFromRotmat(const Rotmat& R, double& fusedYaw) { fusedYaw = FYawOfRotmat(R); }
	void FusedFromRotmat(const Rotmat& R, double& fusedPitch, double& fusedRoll);
	void FusedFromRotmat(const Rotmat& R, double& fusedYaw, double& fusedPitch, double& fusedRoll);
	void FusedFromRotmat(const Rotmat& R, double& fusedYaw, double& fusedPitch, double& fusedRoll, bool& hemi);
	inline void FusedFromRotmat(const Rotmat& R, FusedAngles& f) { FusedFromRotmat(R, f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); }
	inline FusedAngles FusedFromRotmat(const Rotmat& R) { FusedAngles f; FusedFromRotmat(R, f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); return f; }

	// Conversion: Rotation matrix --> Tilt angles
	inline void TiltFromRotmat(const Rotmat& R, double& fusedYaw) { fusedYaw = FYawOfRotmat(R); }
	void TiltFromRotmat(const Rotmat& R, double& tiltAxisAngle, double& tiltAngle);
	void TiltFromRotmat(const Rotmat& R, double& fusedYaw, double& tiltAxisAngle, double& tiltAngle);
	inline void TiltFromRotmat(const Rotmat& R, TiltAngles& t) { TiltFromRotmat(R, t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); }
	inline TiltAngles TiltFromRotmat(const Rotmat& R) { TiltAngles t; TiltFromRotmat(R, t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); return t; }

	// Conversion: Rotation matrix --> Z vector
	inline void ZVecFromRotmat(const Rotmat& R, ZVec& z) { z = R.row(2); }
	inline ZVec ZVecFromRotmat(const Rotmat& R) { return R.row(2); }

	// ######################################
	// #### Conversions from quaternions ####
	// ######################################

	// Conversion: Quaternion --> Rotation matrix
	void RotmatFromQuat(const Quat& q, Rotmat& R);
	inline Rotmat RotmatFromQuat(const Quat& q) { Rotmat R; RotmatFromQuat(q, R); return R; }

	// Conversion: Quaternion --> Euler angles
	inline void EulerFromQuat(const Quat& q, double& yaw) { yaw = EYawOfQuat(q); }
	void EulerFromQuat(const Quat& q, double& yaw, double& pitch, double& roll);
	inline void EulerFromQuat(const Quat& q, EulerAngles& e) { EulerFromQuat(q, e.yaw, e.pitch, e.roll); }
	inline EulerAngles EulerFromQuat(const Quat& q) { EulerAngles e; EulerFromQuat(q, e.yaw, e.pitch, e.roll); return e; }

	// Conversion: Quaternion --> Fused angles
	inline void FusedFromQuat(const Quat& q, double& fusedYaw) { fusedYaw = FYawOfQuat(q); }
	void FusedFromQuat(const Quat& q, double& fusedPitch, double& fusedRoll);
	void FusedFromQuat(const Quat& q, double& fusedYaw, double& fusedPitch, double& fusedRoll);
	void FusedFromQuat(const Quat& q, double& fusedYaw, double& fusedPitch, double& fusedRoll, bool& hemi);
	inline void FusedFromQuat(const Quat& q, FusedAngles& f) { FusedFromQuat(q, f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); }
	inline FusedAngles FusedFromQuat(const Quat& q) { FusedAngles f; FusedFromQuat(q, f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); return f; }

	// Conversion: Quaternion --> Tilt angles
	inline void TiltFromQuat(const Quat& q, double& fusedYaw) { fusedYaw = FYawOfQuat(q); }
	void TiltFromQuat(const Quat& q, double& tiltAxisAngle, double& tiltAngle);
	void TiltFromQuat(const Quat& q, double& fusedYaw, double& tiltAxisAngle, double& tiltAngle);
	inline void TiltFromQuat(const Quat& q, TiltAngles& t) { TiltFromQuat(q, t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); }
	inline TiltAngles TiltFromQuat(const Quat& q) { TiltAngles t; TiltFromQuat(q, t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); return t; }

	// Conversion: Quaternion --> Z vector
	void ZVecFromQuat(const Quat& q, ZVec& z);
	inline ZVec ZVecFromQuat(const Quat& q) { ZVec z; ZVecFromQuat(q, z); return z; }

	// #######################################
	// #### Conversions from Euler angles ####
	// #######################################

	// Conversion: Euler angles --> Rotation matrix
	inline Rotmat RotmatFromEuler(double yaw) { return RotmatFromYaw(yaw); }
	Rotmat RotmatFromEuler(double yaw, double pitch, double roll);
	inline Rotmat RotmatFromEuler(const EulerAngles& e) { return RotmatFromEuler(e.yaw, e.pitch, e.roll); }

	// Conversion: Euler angles --> Quaternion
	inline Quat QuatFromEuler(double yaw) { return QuatFromYaw(yaw); }
	Quat QuatFromEuler(double yaw, double pitch, double roll);
	inline Quat QuatFromEuler(const EulerAngles& e) { return QuatFromEuler(e.yaw, e.pitch, e.roll); }

	// Conversion: Euler angles --> Fused angles
	inline FusedAngles FusedFromEuler(double yaw) { return FusedAngles(yaw); }
	FusedAngles FusedFromEuler(double yaw, double pitch, double roll);
	inline FusedAngles FusedFromEuler(const EulerAngles& e) { return FusedFromEuler(e.yaw, e.pitch, e.roll); }

	// Conversion: Euler angles --> Tilt angles
	inline TiltAngles TiltFromEuler(double yaw) { return TiltAngles(yaw); }
	TiltAngles TiltFromEuler(double yaw, double pitch, double roll);
	inline TiltAngles TiltFromEuler(const EulerAngles& e) { return TiltFromEuler(e.yaw, e.pitch, e.roll); }

	// Conversion: Euler angles --> Z vector
	ZVec ZVecFromEuler(double pitch, double roll);
	inline ZVec ZVecFromEuler(const EulerAngles& e) { return ZVecFromEuler(e.pitch, e.roll); }

	// #######################################
	// #### Conversions from fused angles ####
	// #######################################

	// Conversion: Fused angles --> Rotation matrix
	inline Rotmat RotmatFromFused(double fusedYaw) { return RotmatFromYaw(fusedYaw); }
	Rotmat RotmatFromFused(double fusedPitch, double fusedRoll);
	Rotmat RotmatFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi = true);
	inline Rotmat RotmatFromFused(const FusedAngles& f) { return RotmatFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); }

	// Conversion: Fused angles --> Quaternion
	inline Quat QuatFromFused(double fusedYaw) { return QuatFromYaw(fusedYaw); }
	Quat QuatFromFused(double fusedPitch, double fusedRoll);
	Quat QuatFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi = true);
	inline Quat QuatFromFused(const FusedAngles& f) { return QuatFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); }

	// Conversion: Fused angles --> Euler angles
	inline EulerAngles EulerFromFused(double fusedYaw) { return EulerAngles(fusedYaw); }
	EulerAngles EulerFromFused(double fusedPitch, double fusedRoll);
	EulerAngles EulerFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi = true);
	inline EulerAngles EulerFromFused(const FusedAngles& f) { return EulerFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); }

	// Conversion: Fused angles --> Tilt angles
	inline TiltAngles TiltFromFused(double fusedYaw) { return TiltAngles(fusedYaw); }
	TiltAngles TiltFromFused(double fusedPitch, double fusedRoll);
	TiltAngles TiltFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi = true);
	inline TiltAngles TiltFromFused(const FusedAngles& f) { return TiltFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); }

	// Conversion: Fused angles --> Z vector
	ZVec ZVecFromFused(double fusedPitch, double fusedRoll, bool hemi = true);
	inline ZVec ZVecFromFused(const FusedAngles& f) { return ZVecFromFused(f.fusedPitch, f.fusedRoll, f.hemi); }

	// ######################################
	// #### Conversions from tilt angles ####
	// ######################################

	// Conversion: Tilt angles --> Rotation matrix
	inline Rotmat RotmatFromTilt(double fusedYaw) { return RotmatFromYaw(fusedYaw); }
	Rotmat RotmatFromTilt(double tiltAxisAngle, double tiltAngle);
	Rotmat RotmatFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle);
	inline Rotmat RotmatFromTilt(const TiltAngles& t) { return RotmatFromTilt(t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); }

	// Conversion: Tilt angles --> Quaternion
	inline Quat QuatFromTilt(double fusedYaw) { return QuatFromYaw(fusedYaw); }
	Quat QuatFromTilt(double tiltAxisAngle, double tiltAngle);
	Quat QuatFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle);
	inline Quat QuatFromTilt(const TiltAngles& t) { return QuatFromTilt(t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); }

	// Conversion: Tilt angles --> Euler angles
	inline EulerAngles EulerFromTilt(double fusedYaw) { return EulerAngles(fusedYaw); }
	EulerAngles EulerFromTilt(double tiltAxisAngle, double tiltAngle);
	EulerAngles EulerFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle);
	inline EulerAngles EulerFromTilt(const TiltAngles& t) { return EulerFromTilt(t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); }

	// Conversion: Tilt angles --> Fused angles
	inline FusedAngles FusedFromTilt(double fusedYaw) { return FusedAngles(fusedYaw); }
	FusedAngles FusedFromTilt(double tiltAxisAngle, double tiltAngle);
	FusedAngles FusedFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle);
	inline FusedAngles FusedFromTilt(const TiltAngles& t) { return FusedFromTilt(t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); }

	// Conversion: Tilt angles --> Z vector
	ZVec ZVecFromTilt(double tiltAxisAngle, double tiltAngle);
	inline ZVec ZVecFromTilt(const TiltAngles& t) { return ZVecFromTilt(t.tiltAxisAngle, t.tiltAngle); }

	// ####################################
	// #### Conversions from Z vectors ####
	// ####################################

	// Conversion: Z vector --> Fused angles
	void FusedFromZVec(const ZVec& z, double& fusedPitch, double& fusedRoll);
	void FusedFromZVec(const ZVec& z, double& fusedPitch, double& fusedRoll, bool& hemi);
	inline void FusedFromZVec(const ZVec& z, FusedAngles& f) { FusedFromZVec(z, f.fusedPitch, f.fusedRoll, f.hemi); f.fusedYaw = 0.0; }
	inline FusedAngles FusedFromZVec(const ZVec& z) { FusedAngles f; FusedFromZVec(z, f.fusedPitch, f.fusedRoll, f.hemi); f.fusedYaw = 0.0; return f; }

	// Conversion: Z vector --> Tilt angles
	void TiltFromZVec(const ZVec& z, double& tiltAxisAngle, double& tiltAngle);
	inline void TiltFromZVec(const ZVec& z, TiltAngles& t) { TiltFromZVec(z, t.tiltAxisAngle, t.tiltAngle); t.fusedYaw = 0.0; }
	inline TiltAngles TiltFromZVec(const ZVec& z) { TiltAngles t; TiltFromZVec(z, t.tiltAxisAngle, t.tiltAngle); t.fusedYaw = 0.0; return t; }
}

#endif
// EOF