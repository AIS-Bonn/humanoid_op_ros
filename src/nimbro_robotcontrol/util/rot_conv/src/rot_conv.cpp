// Rotations conversion library
// File: rot_conv.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rot_conv/rot_conv.h>
#include <Eigen/Eigenvalues>
#include <cmath>

// Defines
#define M_2PI (2.0*M_PI)

// Rotations conversion namespace
namespace rot_conv
{
	// ########################
	// #### Rotation types ####
	// ########################

	// Euler angles constants
	const EulerAngles EulerAngles::Identity(0.0, 0.0, 0.0);

	// Fused angles constants
	const FusedAngles FusedAngles::Identity(0.0, 0.0, 0.0, true);

	// Tilt angles constants
	const TiltAngles TiltAngles::Identity(0.0, 0.0, 0.0);

	// ##########################################
	// #### Rotation checking and validation ####
	// ##########################################

	// Check and validate: Rotation matrix
	bool ValidateRotmat(Rotmat& R, double tol)
	{
		// Make a copy of the input
		Rotmat Rorig = R;

		// Find the closest orthogonal matrix to the input rotation matrix
		Rotmat nonOrth = R.transpose() * R;
		R *= Eigen::SelfAdjointEigenSolver<Rotmat>(nonOrth).operatorInverseSqrt();

		// Filter out invalid left hand coordinate systems
		if(R.determinant() < 0.0)
			R.setIdentity();

		// Return whether the rotation matrix was valid within the given tolerance
		return (R - Rorig).isZero(tol);
	}

	// Check and validate: Quaternion
	bool ValidateQuat(Quat& q, double tol, bool unique)
	{
		// Make a copy of the input
		Quat qorig = q;

		// Renormalise the quaternion
		double normsq = q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z();
		if(normsq <= 0.0)
		{
			q.w() = 1.0;
			q.x() = q.y() = q.z() = 0.0;
		}
		else
		{
			double norm = sqrt(normsq);
			q.w() /= norm;
			q.x() /= norm;
			q.y() /= norm;
			q.z() /= norm;
		}

		// Make the quaternion unique
		if(unique && q.w() < 0.0)
		{
			q.w() = -q.w();
			q.x() = -q.x();
			q.y() = -q.y();
			q.z() = -q.z();
		}

		// Return whether the quaternion was valid within the given tolerance
		return (fabs(q.w() - qorig.w()) <= tol && fabs(q.x() - qorig.x()) <= tol && fabs(q.y() - qorig.y()) <= tol && fabs(q.z() - qorig.z()) <= tol);
	}

	// Check and validate: Euler angles
	bool ValidateEuler(EulerAngles& e, double tol, bool unique)
	{
		// Make a copy of the input
		EulerAngles eorig = e;

		// Wrap the pitch to (-pi,pi] and then collapse it to the [-pi/2,pi/2] interval
		e.pitch += M_2PI*std::floor((M_PI - e.pitch) / M_2PI);
		if(fabs(e.pitch) > M_PI_2)
		{
			e.yaw += M_PI;
			e.pitch = (e.pitch >= 0.0 ? M_PI - e.pitch : -M_PI - e.pitch);
			e.roll += M_PI;
		}

		// Make the positive and negative gimbal lock representations unique
		if(unique)
		{
			double spitch = sin(e.pitch);
			if(fabs(spitch - 1.0) <= tol)
			{
				e.roll -= e.yaw;
				e.yaw = 0.0;
			}
			else if(fabs(spitch + 1.0) <= tol)
			{
				e.roll += e.yaw;
				e.yaw = 0.0;
			}
		}

		// Wrap yaw and roll to (-pi,pi]
		e.yaw += M_2PI*std::floor((M_PI - e.yaw) / M_2PI);
		e.roll += M_2PI*std::floor((M_PI - e.roll) / M_2PI);

		// Return whether the Euler angles were valid within the given tolerance
		return (fabs(e.yaw - eorig.yaw) <= tol && fabs(e.pitch - eorig.pitch) <= tol && fabs(e.roll - eorig.roll) <= tol);
	}

	// Check and validate: Fused angles
	bool ValidateFused(FusedAngles& f, double tol, bool unique)
	{
		// Make a copy of the input
		FusedAngles forig = f;

		// Wrap the angles to (-pi,pi]
		f.fusedYaw += M_2PI*std::floor((M_PI - f.fusedYaw) / M_2PI);
		f.fusedPitch += M_2PI*std::floor((M_PI - f.fusedPitch) / M_2PI);
		f.fusedRoll += M_2PI*std::floor((M_PI - f.fusedRoll) / M_2PI);

		// Coerce the L1 norm
		double L1Norm = fabs(f.fusedPitch) + fabs(f.fusedRoll);
		if(L1Norm > M_PI_2)
		{
			double scale = M_PI_2 / L1Norm;
			f.fusedPitch *= scale;
			f.fusedRoll *= scale;
		}

		// Make the representation unique if required
		if(unique)
		{
			double spitch = sin(f.fusedPitch);
			double sroll = sin(f.fusedRoll);
			double sineSum = spitch*spitch + sroll*sroll;
			if(sineSum >= 1.0 - tol)
				f.hemi = true;
			L1Norm = fabs(f.fusedPitch) + fabs(f.fusedRoll);
			if(L1Norm <= tol && !f.hemi)
				f.fusedYaw = 0.0;
		}

		// Return whether the fused angles were valid within the given tolerance
		return (fabs(f.fusedYaw - forig.fusedYaw) <= tol && fabs(f.fusedPitch - forig.fusedPitch) <= tol && fabs(f.fusedRoll - forig.fusedRoll) <= tol && f.hemi == forig.hemi);
	}

	// Check and validate: Tilt angles
	bool ValidateTilt(TiltAngles& t, double tol, bool unique)
	{
		// Make a copy of the input
		TiltAngles torig = t;

		// Wrap the angles to (-pi,pi]
		t.fusedYaw += M_2PI*std::floor((M_PI - t.fusedYaw) / M_2PI);
		t.tiltAxisAngle += M_2PI*std::floor((M_PI - t.tiltAxisAngle) / M_2PI);
		t.tiltAngle += M_2PI*std::floor((M_PI - t.tiltAngle) / M_2PI);

		// Handle the case of a negative tilt angle
		if(t.tiltAngle < 0.0)
		{
			t.tiltAxisAngle = (t.tiltAxisAngle > 0.0 ? -M_PI + t.tiltAxisAngle : M_PI + t.tiltAxisAngle);
			t.tiltAngle = -t.tiltAngle;
		}

		// Make the representation unique if required
		if(unique)
		{
			double ctilt = cos(t.tiltAngle);
			bool near0 = (fabs(ctilt - 1.0) <= tol);
			bool near180 = (fabs(ctilt + 1.0) <= tol);
			if(near0 || near180)
				t.tiltAxisAngle = 0.0;
			if(near180)
				t.fusedYaw = 0.0;
		}

		// Return whether the tilt angles were valid within the given tolerance
		return (fabs(t.fusedYaw - torig.fusedYaw) <= tol && fabs(t.tiltAxisAngle - torig.tiltAxisAngle) <= tol && fabs(t.tiltAngle - torig.tiltAngle) <= tol);
	}

	// ###########################
	// #### Rotation equality ####
	// ###########################

	// Check equality: Rotation matrix
	bool RotmatEqual(const Rotmat& Ra, const Rotmat& Rb, double tol)
	{
		// Return whether none of the elements of the rotation matrices differ by more than the tolerance
		return (Ra - Rb).isZero(tol);
	}

	// Check equality: Quaternion
	bool QuatEqual(const Quat& qa, const Quat& qb, double tol)
	{
		// Return whether to the specified tolerance the quaternions are the same
		bool isSame = (fabs(qa.w() - qb.w()) <= tol && fabs(qa.x() - qb.x()) <= tol && fabs(qa.y() - qb.y()) <= tol && fabs(qa.z() - qb.z()) <= tol);
		bool isOpp  = (fabs(qa.w() + qb.w()) <= tol && fabs(qa.x() + qb.x()) <= tol && fabs(qa.y() + qb.y()) <= tol && fabs(qa.z() + qb.z()) <= tol);
		return (isSame || isOpp);
	}

	// Check equality: Euler angles
	bool EulerEqual(const EulerAngles& ea, const EulerAngles& eb, double tol)
	{
		// Convert both Euler angles to their unique representations
		EulerAngles eau = ea, ebu = eb;
		ValidateEuler(eau, tol, true);
		ValidateEuler(ebu, tol, true);

		// Handle angle wrapping issues
		if(fabs(eau.yaw - ebu.yaw) > M_PI)
		{
			if(eau.yaw > ebu.yaw)
				ebu.yaw += M_2PI;
			else
				eau.yaw += M_2PI;
		}
		if(fabs(eau.roll - ebu.roll) > M_PI)
		{
			if(eau.roll > ebu.roll)
				ebu.roll += M_2PI;
			else
				eau.roll += M_2PI;
		}

		// Return whether to the specified tolerance the Euler angles are the same
		return (fabs(eau.yaw - ebu.yaw) <= tol && fabs(sin(eau.pitch) - sin(ebu.pitch)) <= tol && fabs(eau.roll - ebu.roll) <= tol); // The pitch suffers from the numerical insensitivity of asin, so the sine thereof is checked
	}

	// Check equality: Fused angles
	bool FusedEqual(const FusedAngles& fa, const FusedAngles& fb, double tol)
	{
		// Convert both fused angles to their unique representations
		FusedAngles fau = fa, fbu = fb;
		ValidateFused(fau, tol, true);
		ValidateFused(fbu, tol, true);

		// Handle angle wrapping issues
		if(fabs(fau.fusedYaw - fbu.fusedYaw) > M_PI)
		{
			if(fau.fusedYaw > fbu.fusedYaw)
				fbu.fusedYaw += M_2PI;
			else
				fau.fusedYaw += M_2PI;
		}

		// Return whether to the specified tolerance the fused angles are the same
		return (fabs(fau.fusedYaw - fbu.fusedYaw) <= tol && fabs(sin(fau.fusedPitch) - sin(fbu.fusedPitch)) <= tol && fabs(sin(fau.fusedRoll) - sin(fbu.fusedRoll)) <= tol && fau.hemi == fbu.hemi); // The fused pitch and roll suffer from the numerical insensitivity of asin, so the sine's thereof are checked
	}

	// Check equality: Tilt angles
	bool TiltEqual(const TiltAngles& ta, const TiltAngles& tb, double tol)
	{
		// Convert both tilt angles to their unique representations
		TiltAngles tau = ta, tbu = tb;
		ValidateTilt(tau, tol, true);
		ValidateTilt(tbu, tol, true);

		// Handle angle wrapping issues
		if(fabs(tau.fusedYaw - tbu.fusedYaw) > M_PI)
		{
			if(tau.fusedYaw > tbu.fusedYaw)
				tbu.fusedYaw += M_2PI;
			else
				tau.fusedYaw += M_2PI;
		}

		// Return whether to the specified tolerance the fused angles are the same
		double stilta = sin(tau.tiltAngle);
		double stiltb = sin(tbu.tiltAngle);
		double stiltasq = stilta*stilta;
		double stiltbsq = stiltb*stiltb;
		return (fabs(tau.fusedYaw - tbu.fusedYaw) <= tol && fabs(stiltasq*cos(tau.tiltAxisAngle) - stiltbsq*cos(tbu.tiltAxisAngle)) <= tol && fabs(stiltasq*sin(tau.tiltAxisAngle) - stiltbsq*sin(tbu.tiltAxisAngle)) <= tol && fabs(cos(tau.tiltAngle) - cos(tbu.tiltAngle)) <= tol); // The tilt angle suffers from the numerical insensitivity of acos, so the cosine thereof is checked / The tilt axis angle has a singularity when the tilt angle is zero, so two geometrically relevant terms are checked instead of the tilt axis angle directly
	}

	// #########################
	// #### Yaw of rotation ####
	// #########################

	// Euler yaw of: Rotation matrix
	double EYawOfRotmat(const Rotmat& R)
	{
		// Calculate and return the Euler ZYX yaw of the rotation
		return atan2(R.coeff(1,0), R.coeff(0,0));
	}

	// Fused yaw of: Rotation matrix
	double FYawOfRotmat(const Rotmat& R)
	{
		// Calculate, wrap and return the fused yaw
		double fusedYaw, trace = R.coeff(0,0) + R.coeff(1,1) + R.coeff(2,2);
		if(trace >= 0.0)
			fusedYaw = 2.0*atan2(R.coeff(1,0) - R.coeff(0,1), 1.0 + trace);
		else if(R.coeff(2,2) >= R.coeff(1,1) && R.coeff(2,2) >= R.coeff(0,0))
			fusedYaw = 2.0*atan2(1.0 - R.coeff(0,0) - R.coeff(1,1) + R.coeff(2,2), R.coeff(1,0) - R.coeff(0,1));
		else if(R.coeff(1,1) >= R.coeff(0,0))
			fusedYaw = 2.0*atan2(R.coeff(2,1) + R.coeff(1,2), R.coeff(0,2) - R.coeff(2,0));
		else
			fusedYaw = 2.0*atan2(R.coeff(0,2) + R.coeff(2,0), R.coeff(2,1) - R.coeff(1,2));
		if(fusedYaw > M_PI) fusedYaw -= M_2PI;   // fusedYaw is now in [-2*pi,pi]
		if(fusedYaw <= -M_PI) fusedYaw += M_2PI; // fusedYaw is now in (-pi,pi]
		return fusedYaw;
	}

	// Euler yaw of: Quaternion
	double EYawOfQuat(const Quat& q)
	{
		// Calculate and return the Euler ZYX yaw of the rotation
		return atan2(q.w()*q.z() + q.x()*q.y(), 0.5 - (q.y()*q.y() + q.z()*q.z()));
	}

	// Fused yaw of: Quaternion
	double FYawOfQuat(const Quat& q)
	{
		// Calculate, wrap and return the fused yaw
		double fusedYaw = 2.0*atan2(q.z(), q.w()); // Output of atan2 is [-pi,pi], so this expression is in [-2*pi,2*pi]
		if(fusedYaw > M_PI) fusedYaw -= M_2PI;     // fusedYaw is now in [-2*pi,pi]
		if(fusedYaw <= -M_PI) fusedYaw += M_2PI;   // fusedYaw is now in (-pi,pi]
		return fusedYaw;
	}

	// Fused yaw of: Euler angles
	double FYawOfEuler(const EulerAngles& e)
	{
		// Calculate and return the fused yaw of the rotation
		return FYawOfRotmat(RotmatFromEuler(e));
	}

	// Euler yaw of: Fused angles
	double EYawOfFused(const FusedAngles& f)
	{
		// Precalculate the sin values
		double sth  = sin(f.fusedPitch);
		double sphi = sin(f.fusedRoll);

		// Calculate the sine sum criterion
		double crit = sth*sth + sphi*sphi;

		// Calculate the cosine of the tilt angle alpha
		double calpha;
		if(crit >= 1.0)
			calpha = 0.0;
		else
			calpha = (f.hemi ? sqrt(1.0-crit) : -sqrt(1.0-crit));

		// Calculate the tilt axis gamma
		double gamma = atan2(sth,sphi);

		// Precalculate trigonometric terms
		double psigam = f.fusedYaw + gamma;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);
		double cgam = cos(gamma);
		double sgam = sin(gamma);
		double A = sgam*calpha;

		// Calculate and return the Euler ZYX yaw of the rotation
		return atan2(cgam*spsigam - A*cpsigam, cgam*cpsigam + A*spsigam);
	}

	// Euler yaw of: Tilt angles
	double EYawOfTilt(const TiltAngles& t)
	{
		// Precalculate trigonometric terms
		double psigam = t.fusedYaw + t.tiltAxisAngle;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);
		double cgam = cos(t.tiltAxisAngle);
		double sgam = sin(t.tiltAxisAngle);
		double calpha = cos(t.tiltAngle);
		double A = sgam*calpha;

		// Calculate and return the Euler ZYX yaw of the rotation
		return atan2(cgam*spsigam - A*cpsigam, cgam*cpsigam + A*spsigam);
	}

	// ##################################
	// #### Remove yaw from rotation ####
	// ##################################

	// Remove Euler yaw from: Rotation matrix
	void RotmatNoEYaw(const Rotmat& R, Rotmat& Rout)
	{
		// Calculate the Euler ZYX yaw of the input
		double EYaw = EYawOfRotmat(R);

		// Precalculate trigonometric terms
		double cEYaw = cos(EYaw);
		double sEYaw = sin(EYaw);

		// Construct the Euler ZYX yaw component of the rotation
		Rotmat REYawTrans;
		REYawTrans << cEYaw, sEYaw, 0.0, -sEYaw, cEYaw, 0.0, 0.0, 0.0, 1.0;

		// Remove the Euler ZYX yaw component of the rotation
		Rout = REYawTrans * R;
	}

	// Remove fused yaw from: Rotation matrix
	void RotmatNoFYaw(const Rotmat& R, Rotmat& Rout)
	{
		// Calculate the fused yaw of the input
		double FYaw = FYawOfRotmat(R);

		// Precalculate trigonometric terms
		double cFYaw = cos(FYaw);
		double sFYaw = sin(FYaw);

		// Construct the fused yaw component of the rotation
		Rotmat RFYawTrans;
		RFYawTrans << cFYaw, sFYaw, 0.0, -sFYaw, cFYaw, 0.0, 0.0, 0.0, 1.0;

		// Remove the fused yaw component of the rotation
		Rout = RFYawTrans * R;
	}

	// Remove Euler yaw from: Quaternion
	void QuatNoEYaw(const Quat& q, Quat& qout)
	{
		// Calculate the Euler ZYX yaw of the input
		double EYaw = EYawOfQuat(q);

		// Construct the Euler ZYX yaw component of the rotation
		double hcEYaw = cos(0.5*EYaw);
		double hsEYaw = sin(0.5*EYaw);

		// Remove the Euler ZYX yaw component of the rotation
		qout.w() = hcEYaw*q.w() + hsEYaw*q.z();
		qout.x() = hcEYaw*q.x() + hsEYaw*q.y();
		qout.y() = hcEYaw*q.y() - hsEYaw*q.x();
		qout.z() = hcEYaw*q.z() - hsEYaw*q.w();
	}

	// Remove fused yaw from: Quaternion
	void QuatNoFYaw(const Quat& q, Quat& qout)
	{
		// Calculate the fused yaw of the input
		double FYaw = FYawOfQuat(q);

		// Construct the fused yaw component of the rotation
		double hcFYaw = cos(0.5*FYaw);
		double hsFYaw = sin(0.5*FYaw);

		// Remove the fused yaw component of the rotation
		qout.w() = hcFYaw*q.w() + hsFYaw*q.z();
		qout.x() = hcFYaw*q.x() + hsFYaw*q.y();
		qout.y() = hcFYaw*q.y() - hsFYaw*q.x();
		qout.z() = hcFYaw*q.z() - hsFYaw*q.w();
	}

	// Remove yaw from: Euler angles
	void EulerNoFYaw(const EulerAngles& e, EulerAngles& eout)
	{
		// Calculate the fused yaw of the input
		double FYaw = FYawOfEuler(e);

		// Remove the fused yaw component of the rotation
		eout.yaw = e.yaw - FYaw;
		eout.pitch = e.pitch;
		eout.roll = e.roll;
	}

	// Remove yaw from: Fused angles
	void FusedNoEYaw(const FusedAngles& f, FusedAngles& fout)
	{
		// Calculate the Euler ZYX yaw of the input
		double EYaw = EYawOfFused(f);

		// Remove the Euler ZYX yaw component of the rotation
		fout.fusedYaw = f.fusedYaw - EYaw;
		fout.fusedPitch = f.fusedPitch;
		fout.fusedRoll = f.fusedRoll;
		fout.hemi = f.hemi;
	}

	// Remove yaw from: Tilt angles
	void TiltNoEYaw(const TiltAngles& t, TiltAngles& tout)
	{
		// Calculate the Euler ZYX yaw of the input
		double EYaw = EYawOfTilt(t);

		// Remove the Euler ZYX yaw component of the rotation
		tout.fusedYaw = t.fusedYaw - EYaw;
		tout.tiltAxisAngle = t.tiltAxisAngle;
		tout.tiltAngle = t.tiltAngle;
	}

	// ###########################
	// #### Rotation inverses ####
	// ###########################

	// Inverse: Rotation matrix
	void RotmatInv(const Rotmat& R, Rotmat& Rinv)
	{
		// Calculate the inverse of the rotation
		Rinv = R.transpose();
	}

	// Inverse: Quaternion
	void QuatInv(const Quat& q, Quat& qinv)
	{
		// Calculate the inverse of the rotation
		qinv.w() = q.w();
		qinv.x() = -q.x();
		qinv.y() = -q.y();
		qinv.z() = -q.z();
	}

	// Inverse: Euler angles
	void EulerInv(const EulerAngles& e, EulerAngles& einv)
	{
		// Precalculate the required sin and cos values
		double cpsi = cos(e.yaw);
		double spsi = sin(e.yaw);
		double cth = cos(e.pitch);
		double sth = sin(e.pitch);
		double cphi = cos(e.roll);
		double sphi = sin(e.roll);

		// Calculate the sine of the inverse pitch angle
		double sthinv = -(cpsi*sth*cphi + spsi*sphi);
		sthinv = (sthinv >= 1.0 ? 1.0 : (sthinv <= -1.0 ? -1.0 : sthinv)); // Coerce sthinv to [-1,1]

		// Calculate the required inverse Euler angles representation
		einv.yaw = atan2(cpsi*sth*sphi - spsi*cphi, cpsi*cth);
		einv.pitch = asin(sthinv);
		einv.roll = atan2(spsi*sth*cphi - cpsi*sphi, cth*cphi);
	}

	// Inverse: Fused angles
	void FusedInv(const FusedAngles& f, FusedAngles& finv)
	{
		// Precalculate the sin values
		double sth  = sin(f.fusedPitch);
		double sphi = sin(f.fusedRoll);

		// Calculate the sine of the tilt angle alpha
		double crit = sth*sth + sphi*sphi;
		double salpha = (crit >= 1.0 ? 1.0 : sqrt(crit));

		// Calculate the tilt axis gamma
		double gamma = atan2(sth,sphi);

		// Precalculate trigonometric values
		double psigam = f.fusedYaw + gamma;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);

		// Calculate the inverse fused pitch and roll
		double thinv = asin(-salpha*spsigam);
		double phinv = asin(-salpha*cpsigam);

		// Construct the inverse fused angles rotation
		finv.fusedYaw = -f.fusedYaw;
		finv.fusedPitch = thinv;
		finv.fusedRoll = phinv;
		finv.hemi = f.hemi;
	}

	// Inverse: Tilt angles
	void TiltInv(const TiltAngles& t, TiltAngles& tinv)
	{
		// Calculate the inverse tilt axis angle
		double gammainv = t.fusedYaw + t.tiltAxisAngle - M_PI;
		gammainv += M_2PI*std::floor((M_PI - gammainv) / M_2PI);

		// Construct the inverse tilt angles rotation
		tinv.fusedYaw = -t.fusedYaw;
		tinv.tiltAxisAngle = gammainv;
		tinv.tiltAngle = t.tiltAngle;
	}

	// ##########################
	// #### Vector rotations ####
	// ##########################

	// Rotate vector by: Rotation matrix
	Vec3 RotmatRotVec(const Rotmat& R, const Vec3& v)
	{
		// Return the required rotated vector
		return R*v;
	}

	// Rotate vector by: Quaternion
	Vec3 QuatRotVec(const Quat& q, const Vec3& v)
	{
		// Precalculate an intermediate vector term
		double tx = 2.0*(q.y()*v.z() - v.y()*q.z());
		double ty = 2.0*(q.z()*v.x() - v.z()*q.x());
		double tz = 2.0*(q.x()*v.y() - v.x()*q.y());

		// Calculate and return the required vector
		Vec3 vout = v;
		vout.x() += q.w()*tx + q.y()*tz - ty*q.z();
		vout.y() += q.w()*ty + q.z()*tx - tz*q.x();
		vout.z() += q.w()*tz + q.x()*ty - tx*q.y();
		return vout;
	}

	// Rotate vector by: Euler angles
	Vec3 EulerRotVec(const EulerAngles& e, const Vec3& v)
	{
		// Return the required rotated vector
		return RotmatFromEuler(e)*v;
	}

	// Rotate vector by: Fused angles
	Vec3 FusedRotVec(const FusedAngles& f, const Vec3& v)
	{
		// Return the required rotated vector
		return RotmatFromFused(f)*v;
	}

	// Rotate vector by: Tilt angles
	Vec3 TiltRotVec(const TiltAngles& t, const Vec3& v)
	{
		// Return the required rotated vector
		return RotmatFromTilt(t)*v;
	}

	// ##############################
	// #### Pure yaw conversions ####
	// ##############################

	// Conversion: Pure yaw --> Rotation matrix
	void RotmatFromYaw(double yaw, Rotmat& R)
	{
		// Precalculate trigonometric values
		double cyaw = cos(yaw);
		double syaw = sin(yaw);

		// Set the required rotation matrix
		R << cyaw, -syaw, 0.0, syaw, cyaw, 0.0, 0.0, 0.0, 1.0;
	}

	// Conversion: Pure yaw --> Quaternion
	void QuatFromYaw(double yaw, Quat& q)
	{
		// Precalculate the sin values
		double hpsi = 0.5*yaw;
		double chpsi = cos(hpsi);
		double shpsi = sin(hpsi);

		// Set the required quaternion orientation
		q.w() = chpsi;
		q.x() = 0.0;
		q.y() = 0.0;
		q.z() = shpsi;
	}

	// ############################################
	// #### Conversions from rotation matrices ####
	// ############################################

	//
	// Conversion: Rotation matrix --> Quaternion
	//

	// Conversion: Rotation matrix --> Quaternion
	void QuatFromRotmat(const Rotmat& R, Quat& q)
	{
		// Perform the required conversion in a numerically stable manner
		double r, s, t = R.coeff(0,0) + R.coeff(1,1) + R.coeff(2,2);
		if(t >= 0.0)
		{
			r = sqrt(1.0 + t);
			s = 0.5/r;
			q.w() = 0.5*r;
			q.x() = s*(R.coeff(2,1) - R.coeff(1,2));
			q.y() = s*(R.coeff(0,2) - R.coeff(2,0));
			q.z() = s*(R.coeff(1,0) - R.coeff(0,1));
		}
		else if(R.coeff(2,2) >= R.coeff(1,1) && R.coeff(2,2) >= R.coeff(0,0))
		{
			r = sqrt(1.0 - (R.coeff(0,0) + R.coeff(1,1) - R.coeff(2,2)));
			s = 0.5/r;
			q.w() = s*(R.coeff(1,0) - R.coeff(0,1));
			q.x() = s*(R.coeff(0,2) + R.coeff(2,0));
			q.y() = s*(R.coeff(2,1) + R.coeff(1,2));
			q.z() = 0.5*r;
		}
		else if(R.coeff(1,1) >= R.coeff(0,0))
		{
			r = sqrt(1.0 - (R.coeff(0,0) - R.coeff(1,1) + R.coeff(2,2)));
			s = 0.5/r;
			q.w() = s*(R.coeff(0,2) - R.coeff(2,0));
			q.x() = s*(R.coeff(1,0) + R.coeff(0,1));
			q.y() = 0.5*r;
			q.z() = s*(R.coeff(2,1) + R.coeff(1,2));
		}
		else
		{
			r = sqrt(1.0 + (R.coeff(0,0) - R.coeff(1,1) - R.coeff(2,2)));
			s = 0.5/r;
			q.w() = s*(R.coeff(2,1) - R.coeff(1,2));
			q.x() = 0.5*r;
			q.y() = s*(R.coeff(1,0) + R.coeff(0,1));
			q.z() = s*(R.coeff(0,2) + R.coeff(2,0));
		}
	}

	//
	// Conversion: Rotation matrix --> Euler angles
	//

	// Conversion: Rotation matrix --> Euler angles
	void EulerFromRotmat(const Rotmat& R, double& yaw, double& pitch, double& roll)
	{
		// Calculate the sine of the pitch angle
		double sth = -R.coeff(2,0);
		sth = (sth >= 1.0 ? 1.0 : (sth <= -1.0 ? -1.0 : sth)); // Coerce sth to [-1,1]

		// Calculate the required Euler angles
		yaw = atan2(R.coeff(1,0), R.coeff(0,0));
		pitch = asin(sth);
		roll = atan2(R.coeff(2,1), R.coeff(2,2));
	}

	//
	// Conversion: Rotation matrix --> Fused angles
	//

	// Conversion: Rotation matrix --> Fused angles (2D)
	void FusedFromRotmat(const Rotmat& R, double& fusedPitch, double& fusedRoll)
	{
		// Calculate the fused pitch and roll
		double stheta = -R.coeff(2,0);
		double sphi   = R.coeff(2,1);
		stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
		sphi   = (sphi   >= 1.0 ? 1.0 : (sphi   <= -1.0 ? -1.0 : sphi  )); // Coerce sphi   to [-1,1]
		fusedPitch = asin(stheta);
		fusedRoll  = asin(sphi);
	}

	// Conversion: Rotation matrix --> Fused angles (3D)
	void FusedFromRotmat(const Rotmat& R, double& fusedYaw, double& fusedPitch, double& fusedRoll)
	{
		// Calculate the fused yaw, pitch and roll
		fusedYaw = FYawOfRotmat(R);
		FusedFromRotmat(R, fusedPitch, fusedRoll);
	}

	// Conversion: Rotation matrix --> Fused angles (4D)
	void FusedFromRotmat(const Rotmat& R, double& fusedYaw, double& fusedPitch, double& fusedRoll, bool& hemi)
	{
		// Calculate the fused yaw, pitch and roll
		fusedYaw = FYawOfRotmat(R);
		FusedFromRotmat(R, fusedPitch, fusedRoll);

		// Calculate the hemisphere of the rotation
		hemi = (R.coeff(2,2) >= 0.0);
	}

	//
	// Conversion: Rotation matrix --> Tilt angles
	//

	// Conversion: Rotation matrix --> Tilt angles (2D)
	void TiltFromRotmat(const Rotmat& R, double& tiltAxisAngle, double& tiltAngle)
	{
		// Calculate the tilt axis angle
		tiltAxisAngle = atan2(-R.coeff(2,0), R.coeff(2,1));

		// Calculate the tilt angle
		double calpha = R.coeff(2,2);
		calpha = (calpha >= 1.0 ? 1.0 : (calpha <= -1.0 ? -1.0 : calpha)); // Coerce calpha to [-1,1]
		tiltAngle = acos(calpha);
	}

	// Conversion: Rotation matrix --> Tilt angles (3D)
	void TiltFromRotmat(const Rotmat& R, double& fusedYaw, double& tiltAxisAngle, double& tiltAngle)
	{
		// Calculate the fused yaw, tilt axis angle and tilt angle
		fusedYaw = FYawOfRotmat(R);
		TiltFromRotmat(R, tiltAxisAngle, tiltAngle);
	}

	// ######################################
	// #### Conversions from quaternions ####
	// ######################################

	//
	// Conversion: Quaternion --> Rotation matrix
	//

	// Conversion: Quaternion --> Rotation matrix
	void RotmatFromQuat(const Quat& q, Rotmat& R)
	{
		// Construct the required rotation matrix
		R << 1.0 - 2.0*(q.y()*q.y() + q.z()*q.z()),       2.0*(q.x()*q.y() - q.z()*q.w()),       2.0*(q.x()*q.z() + q.y()*q.w()),
		           2.0*(q.x()*q.y() + q.z()*q.w()), 1.0 - 2.0*(q.x()*q.x() + q.z()*q.z()),       2.0*(q.y()*q.z() - q.x()*q.w()),
		           2.0*(q.x()*q.z() - q.y()*q.w()),       2.0*(q.y()*q.z() + q.x()*q.w()), 1.0 - 2.0*(q.x()*q.x() + q.y()*q.y());
	}

	//
	// Conversion: Quaternion --> Euler angles
	//

	// Conversion: Quaternion --> Euler angles
	void EulerFromQuat(const Quat& q, double& yaw, double& pitch, double& roll)
	{
		// Calculate the sine of the pitch angle
		double sth = 2.0*(q.w()*q.y() - q.x()*q.z());
		sth = (sth >= 1.0 ? 1.0 : (sth <= -1.0 ? -1.0 : sth)); // Coerce sth to [-1,1]

		// Calculate the required Euler angles
		double qysq = q.y()*q.y();
		yaw = atan2(q.x()*q.y() + q.z()*q.w(), 0.5 - (qysq + q.z()*q.z()));
		pitch = asin(sth);
		roll = atan2(q.y()*q.z() + q.x()*q.w(), 0.5 - (q.x()*q.x() + qysq));
	}

	//
	// Conversion: Quaternion --> Fused angles
	//

	// Conversion: Quaternion --> Fused angles (2D)
	void FusedFromQuat(const Quat& q, double& fusedPitch, double& fusedRoll)
	{
		// Calculate the fused pitch and roll
		double stheta = 2.0*(q.y()*q.w() - q.x()*q.z());
		double sphi   = 2.0*(q.y()*q.z() + q.x()*q.w());
		stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
		sphi   = (sphi   >= 1.0 ? 1.0 : (sphi   <= -1.0 ? -1.0 : sphi  )); // Coerce sphi   to [-1,1]
		fusedPitch = asin(stheta);
		fusedRoll  = asin(sphi);
	}

	// Conversion: Quaternion --> Fused angles (3D)
	void FusedFromQuat(const Quat& q, double& fusedYaw, double& fusedPitch, double& fusedRoll)
	{
		// Calculate the fused yaw, pitch and roll
		FusedFromQuat(q, fusedYaw);
		FusedFromQuat(q, fusedPitch, fusedRoll);
	}

	// Conversion: Quaternion --> Fused angles (4D)
	void FusedFromQuat(const Quat& q, double& fusedYaw, double& fusedPitch, double& fusedRoll, bool& hemi)
	{
		// Calculate the fused yaw, pitch and roll
		FusedFromQuat(q, fusedYaw);
		FusedFromQuat(q, fusedPitch, fusedRoll);

		// Calculate the hemisphere of the rotation
		hemi = (0.5 - (q.x()*q.x() + q.y()*q.y()) >= 0.0);
	}

	//
	// Conversion: Quaternion --> Tilt angles
	//

	// Conversion: Quaternion --> Tilt angles (2D)
	void TiltFromQuat(const Quat& q, double& tiltAxisAngle, double& tiltAngle)
	{
		// Calculate the tilt axis angle
		tiltAxisAngle = atan2(q.w()*q.y() - q.x()*q.z(), q.w()*q.x() + q.y()*q.z());

		// Calculate the tilt angle
		double calpha = 1.0 - 2.0*(q.x()*q.x() + q.y()*q.y());
		calpha = (calpha >= 1.0 ? 1.0 : (calpha <= -1.0 ? -1.0 : calpha)); // Coerce calpha to [-1,1]
		tiltAngle = acos(calpha);
	}

	// Conversion: Quaternion --> Tilt angles (3D)
	void TiltFromQuat(const Quat& q, double& fusedYaw, double& tiltAxisAngle, double& tiltAngle)
	{
		// Calculate the fused yaw, tilt axis angle and tilt angle
		FusedFromQuat(q, fusedYaw);
		TiltFromQuat(q, tiltAxisAngle, tiltAngle);
	}

	//
	// Conversion: Quaternion --> Z vector
	//

	// Conversion: Quaternion --> Z vector
	void ZVecFromQuat(const Quat& q, ZVec& z)
	{
		// Calculate the required Z vector
		z.x() = 2.0*(q.x()*q.z() - q.y()*q.w());
		z.y() = 2.0*(q.y()*q.z() + q.x()*q.w());
		z.z() = 1.0 - 2.0*(q.x()*q.x() + q.y()*q.y());
	}

	// #######################################
	// #### Conversions from Euler angles ####
	// #######################################

	// Conversion: Euler angles --> Rotation matrix
	Rotmat RotmatFromEuler(double yaw, double pitch, double roll)
	{
		// Precalculate the trigonometric values
		double cpsi = cos(yaw);
		double spsi = sin(yaw);
		double cth  = cos(pitch);
		double sth  = sin(pitch);
		double cphi = cos(roll);
		double sphi = sin(roll);

		// Calculate and return the required rotation matrix
		Rotmat R;
		R << cpsi*cth, cpsi*sth*sphi - spsi*cphi, cpsi*sth*cphi + spsi*sphi,
		     spsi*cth, spsi*sth*sphi + cpsi*cphi, spsi*sth*cphi - cpsi*sphi,
		         -sth,                  cth*sphi,                  cth*cphi;
		return R;
	}

	// Conversion: Euler angles --> Quaternion
	Quat QuatFromEuler(double yaw, double pitch, double roll)
	{
		// Halve the Euler angles
		double hpsi = 0.5*yaw;
		double hth = 0.5*pitch;
		double hphi = 0.5*roll;

		// Precalculate the trigonometric values
		double hcpsi = cos(hpsi);
		double hspsi = sin(hpsi);
		double hcth  = cos(hth);
		double hsth  = sin(hth);
		double hcphi = cos(hphi);
		double hsphi = sin(hphi);

		// Calculate and return the required quaternion
		return Quat(hcphi*hcth*hcpsi + hsphi*hsth*hspsi, hsphi*hcth*hcpsi - hcphi*hsth*hspsi, hcphi*hsth*hcpsi + hsphi*hcth*hspsi, hcphi*hcth*hspsi - hsphi*hsth*hcpsi);
	}

	// Conversion: Euler angles --> Fused angles
	FusedAngles FusedFromEuler(double yaw, double pitch, double roll)
	{
		// Construct a fused angles object
		FusedAngles f;

		// Calculation of the fused yaw in a numerically stable manner requires the complete rotation matrix representation
		Rotmat R = RotmatFromEuler(yaw, pitch, roll);

		// Calculate the fused yaw
		f.fusedYaw = FYawOfRotmat(R);

		// Calculate the fused pitch
		f.fusedPitch = pitch; // ZYX Euler pitch is equivalent to fused pitch!

		// Calculate the fused roll
		double sphi = R.coeff(2,1);
		sphi = (sphi >= 1.0 ? 1.0 : (sphi <= -1.0 ? -1.0 : sphi)); // Coerce sphi to [-1,1]
		f.fusedRoll  = asin(sphi);

		// See which hemisphere we're in
		f.hemi = (R.coeff(2,2) >= 0.0);

		// Return the calculated fused angles
		return f;
	}

	// Conversion: Euler angles --> Tilt angles
	TiltAngles TiltFromEuler(double yaw, double pitch, double roll)
	{
		// Construct a tilt angles object
		TiltAngles t;

		// Calculation of the fused yaw in a numerically stable manner requires the complete rotation matrix representation
		Rotmat R = RotmatFromEuler(yaw, pitch, roll);

		// Calculate the fused yaw
		t.fusedYaw = FYawOfRotmat(R);

		// Calculate the tilt axis angle
		t.tiltAxisAngle = atan2(-R.coeff(2,0), R.coeff(2,1));

		// Calculate the tilt angle
		double calpha = R.coeff(2,2);
		calpha = (calpha >= 1.0 ? 1.0 : (calpha <= -1.0 ? -1.0 : calpha)); // Coerce calpha to [-1,1]
		t.tiltAngle = acos(calpha);

		// Return the calculated tilt angles
		return t;
	}

	// Conversion: Euler angles --> Z vector
	ZVec ZVecFromEuler(double pitch, double roll)
	{
		// Precalculate the trigonometric values
		double cth  = cos(pitch);
		double sth  = sin(pitch);
		double cphi = cos(roll);
		double sphi = sin(roll);

		// Calculate and return the required Z vector
		return ZVec(-sth, cth*sphi, cth*cphi);
	}

	// #######################################
	// #### Conversions from fused angles ####
	// #######################################

	//
	// Conversion: Fused angles --> Rotation matrix
	//

	// Conversion: Fused angles (2D) --> Rotation matrix
	Rotmat RotmatFromFused(double fusedPitch, double fusedRoll) // Assume: fusedYaw = 0, hemi = true
	{
		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the sine sum criterion
		double crit = sth*sth + sphi*sphi;
		double calpha = (crit >= 1.0 ? 0.0 : sqrt(1.0 - crit));

		// Calculate the tilt axis angle gamma
		double gamma = atan2(sth,sphi);

		// Precalculate terms involved in the rotation matrix expression
		double calphabar = 1.0 - calpha;
		double cgam = cos(gamma);
		double sgam = sin(gamma);
		double A = calpha + calphabar*cgam*cgam;
		double B = calpha + calphabar*sgam*sgam;
		double C = calphabar*cgam*sgam;

		// Calculate and return the required rotation matrix
		Rotmat R;
		R <<    A,    C,    sth,
		        C,    B,  -sphi,
		     -sth, sphi, calpha;
		return R;
	}

	// Conversion: Fused angles (3D/4D) --> Rotation matrix
	Rotmat RotmatFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi)
	{
		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the sine sum criterion
		double crit = sth*sth + sphi*sphi;

		// Calculate the tilt angle alpha
		double calpha, salpha;
		if(crit >= 1.0)
		{
			calpha = 0.0;
			salpha = 1.0;
		}
		else
		{
			calpha = (hemi ? sqrt(1.0 - crit) : -sqrt(1.0 - crit));
			salpha = sqrt(crit);
		}

		// Calculate the tilt axis angle gamma
		double gamma = atan2(sth,sphi);

		// Precalculate terms involved in the rotation matrix expression
		double cgam = cos(gamma);
		double sgam = sin(gamma);
		double psigam = fusedYaw + gamma;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);
		double A = cgam*cpsigam;
		double B = sgam*cpsigam;
		double C = cgam*spsigam;
		double D = sgam*spsigam;

		// Calculate and return the required rotation matrix
		Rotmat R;
		R << A + D*calpha, B - C*calpha,  salpha*spsigam,
		     C - B*calpha, D + A*calpha, -salpha*cpsigam,
		          -sth,          sphi,    calpha;
		return R;
	}

	//
	// Conversion: Fused angles --> Quaternion
	//

	// Conversion: Fused angles (2D) --> Quaternion
	Quat QuatFromFused(double fusedPitch, double fusedRoll) // Assume: fusedYaw = 0, hemi = true
	{
		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the sine sum criterion
		double crit = sth*sth + sphi*sphi;

		// Calculate the tilt angle alpha
		double alpha = (crit >= 1.0 ? M_PI_2 : acos(sqrt(1.0 - crit)));
		double halpha = 0.5*alpha;
		double chalpha = cos(halpha);
		double shalpha = sin(halpha);

		// Calculate the tilt axis angle gamma
		double gamma = atan2(sth,sphi);
		double cgamma = cos(gamma);
		double sgamma = sin(gamma);

		// Return the required quaternion orientation (a rotation about (cgamma, sgamma, 0) by angle alpha)
		return Quat(chalpha, cgamma*shalpha, sgamma*shalpha, 0.0); // Order: (w,x,y,z)
	}

	// Conversion: Fused angles (3D/4D) --> Quaternion
	Quat QuatFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi)
	{
		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the sine sum criterion
		double crit = sth*sth + sphi*sphi;

		// Calculate the tilt angle alpha
		double alpha;
		if(crit >= 1.0)
			alpha = M_PI_2;
		else
			alpha = acos(hemi ? sqrt(1.0 - crit) : -sqrt(1.0 - crit));

		// Calculate the tilt axis angle gamma
		double gamma = atan2(sth,sphi);

		// Evaluate the required intermediate angles
		double halpha = 0.5*alpha;
		double hpsi = 0.5*fusedYaw;
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

	//
	// Conversion: Fused angles --> Euler angles
	//

	// Conversion: Fused angles (2D) --> Euler angles
	EulerAngles EulerFromFused(double fusedPitch, double fusedRoll) // Assume: fusedYaw = 0, hemi = true
	{
		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the cosine of the tilt angle
		double crit = sth*sth + sphi*sphi;
		double calpha = (crit >= 1.0 ? 0.0 : sqrt(1.0 - crit));

		// Calculate the tilt axis angle gamma
		double gamma = atan2(sth,sphi);

		// Precalculate terms
		double cgam = cos(gamma);
		double sgam = sin(gamma);
		double A = cgam*(1.0 - calpha);

		// Calculate and return the required Euler angles
		return EulerAngles(atan2(A*sgam, calpha + A*cgam), fusedPitch, atan2(sphi, calpha));
	}

	// Conversion: Fused angles (3D/4D) --> Euler angles
	EulerAngles EulerFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi)
	{
		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the cosine of the tilt angle
		double crit = sth*sth + sphi*sphi;
		double calpha = 0.0;
		if(crit < 1.0)
			calpha = (hemi ? sqrt(1.0 - crit) : -sqrt(1.0 - crit));

		// Calculate the tilt axis angle gamma
		double gamma = atan2(sth,sphi);

		// Precalculate terms
		double cgam = cos(gamma);
		double sgam = sin(gamma);
		double psigam = fusedYaw + gamma;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);
		double A = sgam*calpha;

		// Calculate and return the required Euler angles
		return EulerAngles(atan2(cgam*spsigam - A*cpsigam, cgam*cpsigam + A*spsigam), fusedPitch, atan2(sphi, calpha));
	}

	//
	// Conversion: Fused angles --> Tilt angles
	//

	// Conversion: Fused angles (2D) --> Tilt angles
	TiltAngles TiltFromFused(double fusedPitch, double fusedRoll) // Assume: fusedYaw = 0, hemi = true
	{
		// Construct a tilt angles object
		TiltAngles t;

		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the cosine of the tilt angle
		double crit = sth*sth + sphi*sphi;
		double calpha = (crit >= 1.0 ? 0.0 : sqrt(1.0 - crit));

		// Calculate and return the tilt angles representation
		t.fusedYaw = 0.0;
		t.tiltAxisAngle = atan2(sth,sphi);
		t.tiltAngle = acos(calpha);
		return t;
	}

	// Conversion: Fused angles (3D/4D) --> Tilt angles
	TiltAngles TiltFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi)
	{
		// Construct a tilt angles object
		TiltAngles t;

		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the cosine of the tilt angle
		double crit = sth*sth + sphi*sphi;
		double calpha = 0.0;
		if(crit < 1.0)
			calpha = (hemi ? sqrt(1.0 - crit) : -sqrt(1.0 - crit));

		// Calculate and return the tilt angles representation
		t.fusedYaw = fusedYaw;
		t.tiltAxisAngle = atan2(sth,sphi);
		t.tiltAngle = acos(calpha);
		return t;
	}

	//
	// Conversion: Fused angles --> Z vector
	//

	// Conversion: Fused angles --> Z vector
	ZVec ZVecFromFused(double fusedPitch, double fusedRoll, bool hemi)
	{
		// Precalculate the sin values
		double sth  = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the cosine of the tilt angle
		double crit = sth*sth + sphi*sphi;
		double calpha = 0.0;
		if(crit < 1.0)
			calpha = (hemi ? sqrt(1.0 - crit) : -sqrt(1.0 - crit));

		// Return the required Z vector
		return ZVec(-sth, sphi, calpha);
	}

	// ######################################
	// #### Conversions from tilt angles ####
	// ######################################

	//
	// Conversion: Tilt angles --> Rotation matrix
	//

	// Conversion: Tilt angles (2D) --> Rotation matrix
	Rotmat RotmatFromTilt(double tiltAxisAngle, double tiltAngle) // Assume: fusedYaw = 0
	{
		// Precalculate terms involved in the rotation matrix expression
		double cgam = cos(tiltAxisAngle);
		double sgam = sin(tiltAxisAngle);
		double calpha = cos(tiltAngle);
		double salpha = sin(tiltAngle);
		double calphabar = 1.0 - calpha;
		double sth = salpha*sgam;
		double sphi = salpha*cgam;
		double A = calpha + calphabar*cgam*cgam;
		double B = calpha + calphabar*sgam*sgam;
		double C = calphabar*cgam*sgam;

		// Calculate and return the required rotation matrix
		Rotmat R;
		R <<    A,    C,    sth,
		        C,    B,  -sphi,
		     -sth, sphi, calpha;
		return R;
	}

	// Conversion: Tilt angles (3D) --> Rotation matrix
	Rotmat RotmatFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle)
	{
		// Precalculate terms involved in the rotation matrix expression
		double cgam = cos(tiltAxisAngle);
		double sgam = sin(tiltAxisAngle);
		double calpha = cos(tiltAngle);
		double salpha = sin(tiltAngle);
		double psigam = fusedYaw + tiltAxisAngle;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);
		double A = cgam*cpsigam;
		double B = sgam*cpsigam;
		double C = cgam*spsigam;
		double D = sgam*spsigam;

		// Calculate and return the required rotation matrix
		Rotmat R;
		R << A + D*calpha, B - C*calpha,  salpha*spsigam,
		     C - B*calpha, D + A*calpha, -salpha*cpsigam,
		     -sgam*salpha,  cgam*salpha,  calpha;
		return R;
	}

	//
	// Conversion: Tilt angles --> Quaternion
	//

	// Conversion: Tilt angles (2D) --> Quaternion
	Quat QuatFromTilt(double tiltAxisAngle, double tiltAngle) // Assume: fusedYaw = 0
	{
		// Precalculate the required angles
		double halpha = 0.5*tiltAngle;

		// Precalculate the required trigonometric values
		double chalpha = cos(halpha);
		double shalpha = sin(halpha);
		double cgamma = cos(tiltAxisAngle);
		double sgamma = sin(tiltAxisAngle);

		// Return the required quaternion orientation
		return Quat(chalpha, shalpha*cgamma, shalpha*sgamma, 0.0); // Order: (w,x,y,z)
	}

	// Conversion: Tilt angles (3D) --> Quaternion
	Quat QuatFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle)
	{
		// Precalculate the required angles
		double hpsi = 0.5*fusedYaw;
		double halpha = 0.5*tiltAngle;
		double hgampsi = tiltAxisAngle + hpsi;

		// Precalculate the required trigonometric values
		double chpsi = cos(hpsi);
		double shpsi = sin(hpsi);
		double chalpha = cos(halpha);
		double shalpha = sin(halpha);
		double chgampsi = cos(hgampsi);
		double shgampsi = sin(hgampsi);

		// Return the required quaternion orientation
		return Quat(chalpha*chpsi, shalpha*chgampsi, shalpha*shgampsi, chalpha*shpsi); // Order: (w,x,y,z)
	}

	//
	// Conversion: Tilt angles --> Euler angles
	//

	// Conversion: Tilt angles (2D) --> Euler angles
	EulerAngles EulerFromTilt(double tiltAxisAngle, double tiltAngle) // Assume: fusedYaw = 0
	{
		// Precalculate terms
		double cgam = cos(tiltAxisAngle);
		double sgam = sin(tiltAxisAngle);
		double calpha = cos(tiltAngle);
		double salpha = sin(tiltAngle);
		double sth = sgam*salpha;
		double sphi = cgam*salpha;
		double A = cgam*(1.0 - calpha);

		// Calculate and return the required Euler angles
		return EulerAngles(atan2(A*sgam, calpha + A*cgam), asin(sth), atan2(sphi, calpha));
	}

	// Conversion: Tilt angles (3D) --> Euler angles
	EulerAngles EulerFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle)
	{
		// Precalculate terms
		double cgam = cos(tiltAxisAngle);
		double sgam = sin(tiltAxisAngle);
		double calpha = cos(tiltAngle);
		double salpha = sin(tiltAngle);
		double sth = sgam*salpha;
		double sphi = cgam*salpha;
		double psigam = fusedYaw + tiltAxisAngle;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);
		double A = sgam*calpha;

		// Calculate and return the required Euler angles
		return EulerAngles(atan2(cgam*spsigam - A*cpsigam, cgam*cpsigam + A*spsigam), asin(sth), atan2(sphi, calpha));
	}

	//
	// Conversion: Tilt angles --> Fused angles
	//

	// Conversion: Tilt angles (2D) --> Fused angles
	FusedAngles FusedFromTilt(double tiltAxisAngle, double tiltAngle) // Assume: fusedYaw = 0
	{
		// Calculate and return the fused angles representation
		return FusedFromTilt(0.0, tiltAxisAngle, tiltAngle);
	}

	// Conversion: Tilt angles (3D) --> Fused angles
	FusedAngles FusedFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle)
	{
		// Construct a fused angles object
		FusedAngles f;

		// Precalculate terms
		double cgam = cos(tiltAxisAngle);
		double sgam = sin(tiltAxisAngle);
		double salpha = sin(tiltAngle);

		// Calculate and return the fused angles representation
		f.fusedYaw = fusedYaw;
		f.fusedPitch = asin(salpha*sgam);
		f.fusedRoll = asin(salpha*cgam);
		f.hemi = (tiltAngle <= M_PI_2);
		return f;
	}

	//
	// Conversion: Tilt angles --> Z vector
	//

	// Conversion: Tilt angles --> Z vector
	ZVec ZVecFromTilt(double tiltAxisAngle, double tiltAngle)
	{
		// Precalculate the required trigonometric terms
		double cgamma = cos(tiltAxisAngle);
		double sgamma = sin(tiltAxisAngle);
		double calpha = cos(tiltAngle);
		double salpha = sin(tiltAngle);

		// Return the required Z vector
		return ZVec(-salpha*sgamma, salpha*cgamma, calpha);
	}

	// ####################################
	// #### Conversions from Z vectors ####
	// ####################################

	//
	// Conversion: Z vector --> Fused angles
	//

	// Conversion: Z vector --> Fused angles (2D)
	void FusedFromZVec(const ZVec& z, double& fusedPitch, double& fusedRoll)
	{
		// Calculate the fused pitch and roll
		double stheta = -z.x();
		double sphi   = z.y();
		stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
		sphi   = (sphi   >= 1.0 ? 1.0 : (sphi   <= -1.0 ? -1.0 : sphi  )); // Coerce sphi   to [-1,1]
		fusedPitch = asin(stheta);
		fusedRoll  = asin(sphi);
	}

	// Conversion: Z vector --> Fused angles (3D)
	void FusedFromZVec(const ZVec& z, double& fusedPitch, double& fusedRoll, bool& hemi)
	{
		// Calculate the fused pitch and roll
		FusedFromZVec(z, fusedPitch, fusedRoll);

		// Calculate the hemisphere
		hemi = (z.z() >= 0.0);
	}

	//
	// Conversion: Z vector --> Tilt angles
	//

	// Conversion: Z vector --> Tilt angles (2D)
	void TiltFromZVec(const ZVec& z, double& tiltAxisAngle, double& tiltAngle)
	{
		// Calculate the tilt axis angle
		tiltAxisAngle = atan2(-z.x(), z.y());

		// Calculate the tilt angle
		double calpha = z.z();
		calpha = (calpha >= 1.0 ? 1.0 : (calpha <= -1.0 ? -1.0 : calpha)); // Coerce calpha to [-1,1]
		tiltAngle = acos(calpha);
	}
}
// EOF