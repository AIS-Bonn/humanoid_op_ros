// Rotations conversion library
// File: rot_conv_extras.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef ROT_CONV_EXTRAS_H
#define ROT_CONV_EXTRAS_H

// Includes
#include <rot_conv/rot_conv.h>

// Rotations conversion namespace
namespace rot_conv
{
	// ############################
	// #### Absolute rotations ####
	// ############################

	// Typedefs
	typedef TiltAngles AbsTiltAngles; // Absolute tilt angles struct with format: (fusedYaw, absTiltAxisAngle, tiltAngle)
	typedef TiltPhase2D AbsTiltPhase2D; // Absolute tilt phase 2D struct with format: (px, py) = (tiltAngle*cos(absTiltAxisAngle), tiltAngle*sin(absTiltAxisAngle))
	typedef TiltPhase3D AbsTiltPhase3D; // Absolute tilt phase 3D struct with format: (px, py, pz) = (tiltAngle*cos(absTiltAxisAngle), tiltAngle*sin(absTiltAxisAngle), fusedYaw)

	// Absolute tilt rotation struct with format: (absTiltAxisAngle, tiltAngle)
	struct AbsTiltRot
	{
		// Constants
		static inline AbsTiltRot Identity() { return AbsTiltRot(0.0, 0.0); }

		// Constructors
		AbsTiltRot() = default;
		AbsTiltRot(double absTiltAxisAngle, double tiltAngle) : absTiltAxisAngle(absTiltAxisAngle), tiltAngle(tiltAngle) {}

		// Set functions
		void set(double absTiltAxisAngle, double tiltAngle) { this->absTiltAxisAngle = absTiltAxisAngle; this->tiltAngle = tiltAngle; }
		void set(const AbsTiltAngles& at) { absTiltAxisAngle = at.tiltAxisAngle; tiltAngle = at.tiltAngle; }
		void setIdentity() { absTiltAxisAngle = tiltAngle = 0.0; }

		// Data members
		double absTiltAxisAngle; // Absolute tilt axis angle = tiltAxisAngle + fusedYaw in (-pi,pi]
		double tiltAngle;        // Tilt angle = alpha in [0,Inf)
	};

	// Stream insertion operator for the AbsTiltRot struct
	inline std::ostream& operator<<(std::ostream& os, const AbsTiltRot& atr) { return os << "AT(" << atr.absTiltAxisAngle << ", " << atr.tiltAngle << ")"; }

	// Conversion: Tilt angles --> Absolute tilt angles
	inline void AbsTiltFromTilt(const TiltAngles& t, AbsTiltAngles& at) { at.fusedYaw = t.fusedYaw; at.tiltAxisAngle = internal::picut(t.tiltAxisAngle + t.fusedYaw); at.tiltAngle = t.tiltAngle; }
	inline AbsTiltAngles AbsTiltFromTilt(const TiltAngles& t) { return AbsTiltAngles(t.fusedYaw, internal::picut(t.tiltAxisAngle + t.fusedYaw), t.tiltAngle); }

	// Conversion: Absolute tilt angles --> Tilt angles
	inline void TiltFromAbsTilt(const AbsTiltAngles& at, TiltAngles& t) { t.fusedYaw = at.fusedYaw; t.tiltAxisAngle = internal::picut(at.tiltAxisAngle - t.fusedYaw); t.tiltAngle = at.tiltAngle; }
	inline TiltAngles TiltFromAbsTilt(const AbsTiltAngles& at) { return TiltAngles(at.fusedYaw, internal::picut(at.tiltAxisAngle - at.fusedYaw), at.tiltAngle); }

	// Conversion: Absolute fused yaw and tilt rotation --> Absolute tilt angles
	inline void AbsTiltFromAbsYawTilt(const AbsTiltRot& atr, AbsTiltAngles& at) { at.fusedYaw = 0.0; at.tiltAxisAngle = atr.absTiltAxisAngle; at.tiltAngle = atr.tiltAngle; }
	inline void AbsTiltFromAbsYawTilt(const AbsTiltRot& atr, double fusedYaw, AbsTiltAngles& at) { at.fusedYaw = fusedYaw; at.tiltAxisAngle = atr.absTiltAxisAngle; at.tiltAngle = atr.tiltAngle; }
	inline AbsTiltAngles AbsTiltFromAbsYawTilt(const AbsTiltRot& atr) { return AbsTiltAngles(0.0, atr.absTiltAxisAngle, atr.tiltAngle); }
	inline AbsTiltAngles AbsTiltFromAbsYawTilt(const AbsTiltRot& atr, double fusedYaw) { return AbsTiltAngles(fusedYaw, atr.absTiltAxisAngle, atr.tiltAngle); }

	// Conversion: Absolute tilt angles --> Absolute fused yaw and tilt rotation
	inline void AbsYawTiltFromAbsTilt(const AbsTiltAngles& at, AbsTiltRot& atr) { atr.absTiltAxisAngle = at.tiltAxisAngle; atr.tiltAngle = at.tiltAngle; }
	inline void AbsYawTiltFromAbsTilt(const AbsTiltAngles& at, AbsTiltRot& atr, double& fusedYaw) { fusedYaw = at.fusedYaw; AbsYawTiltFromAbsTilt(at, atr); }

	// Conversion: Absolute fused yaw and tilt rotation --> Tilt angles
	inline void TiltFromAbsYawTilt(const AbsTiltRot& atr, TiltAngles& t) { t.fusedYaw = 0.0; t.tiltAxisAngle = atr.absTiltAxisAngle; t.tiltAngle = atr.tiltAngle; }
	inline void TiltFromAbsYawTilt(const AbsTiltRot& atr, double fusedYaw, TiltAngles& t) { t.fusedYaw = fusedYaw; t.tiltAxisAngle = internal::picut(atr.absTiltAxisAngle - fusedYaw); t.tiltAngle = atr.tiltAngle; }
	inline TiltAngles TiltFromAbsYawTilt(const AbsTiltRot& atr) { return TiltAngles(0.0, atr.absTiltAxisAngle, atr.tiltAngle); }
	inline TiltAngles TiltFromAbsYawTilt(const AbsTiltRot& atr, double fusedYaw) { return TiltAngles(fusedYaw, internal::picut(atr.absTiltAxisAngle - fusedYaw), atr.tiltAngle); }

	// Conversion: Tilt angles --> Absolute fused yaw and tilt rotation
	inline void AbsYawTiltFromTilt(const TiltAngles& t, AbsTiltRot& atr) { atr.absTiltAxisAngle = internal::picut(t.tiltAxisAngle + t.fusedYaw); atr.tiltAngle = t.tiltAngle; }
	inline void AbsYawTiltFromTilt(const TiltAngles& t, AbsTiltRot& atr, double& fusedYaw) { fusedYaw = t.fusedYaw; AbsYawTiltFromTilt(t, atr); }

	// Conversion: Absolute fused yaw and tilt rotation --> Absolute tilt phase
	inline void AbsPhaseFromAbsYawTilt(const AbsTiltRot& atr, AbsTiltPhase2D& ap) { ap.setTilt(atr.absTiltAxisAngle, atr.tiltAngle); }
	inline void AbsPhaseFromAbsYawTilt(const AbsTiltRot& atr, double fusedYaw, AbsTiltPhase3D& ap) { ap.setTilt(atr.absTiltAxisAngle, atr.tiltAngle, fusedYaw); }
	inline AbsTiltPhase2D AbsPhaseFromAbsYawTilt(const AbsTiltRot& atr) { AbsTiltPhase2D ap; AbsPhaseFromAbsYawTilt(atr, ap); return ap; }
	inline AbsTiltPhase3D AbsPhaseFromAbsYawTilt(const AbsTiltRot& atr, double fusedYaw) { AbsTiltPhase3D ap; AbsPhaseFromAbsYawTilt(atr, fusedYaw, ap); return ap; }

	// Conversion: Absolute tilt phase --> Absolute fused yaw and tilt rotation
	inline void AbsYawTiltFromAbsPhase(const AbsTiltPhase2D& ap, AbsTiltRot& atr) { ap.getTilt(atr.absTiltAxisAngle, atr.tiltAngle); }
	inline void AbsYawTiltFromAbsPhase(const AbsTiltPhase3D& ap, AbsTiltRot& atr, double& fusedYaw) { ap.getTilt(atr.absTiltAxisAngle, atr.tiltAngle, fusedYaw); }

	// Conversion: Absolute tilt angles --> Absolute tilt phase
	inline void AbsPhaseFromAbsTilt(const AbsTiltAngles& at, AbsTiltPhase2D& ap) { ap.setTilt(at); }
	inline void AbsPhaseFromAbsTilt(const AbsTiltAngles& at, AbsTiltPhase3D& ap) { ap.setTilt(at); }

	// Conversion: Absolute tilt phase --> Absolute tilt angles
	inline void AbsTiltFromAbsPhase(const AbsTiltPhase2D& ap, AbsTiltAngles& at) { ap.getTilt(at); }
	inline void AbsTiltFromAbsPhase(const AbsTiltPhase3D& ap, AbsTiltAngles& at) { ap.getTilt(at); }
	inline AbsTiltAngles AbsTiltFromAbsPhase(const AbsTiltPhase2D& ap) { return ap.getTilt(); }
	inline AbsTiltAngles AbsTiltFromAbsPhase(const AbsTiltPhase3D& ap) { return ap.getTilt(); }

	// Conversion: Tilt phase --> Absolute tilt phase
	inline void AbsPhaseFromPhase(const TiltPhase2D& p, double fusedYaw, AbsTiltPhase2D& ap) { double cpsi = cos(fusedYaw), spsi = sin(fusedYaw); ap.px = cpsi*p.px - spsi*p.py; ap.py = spsi*p.px + cpsi*p.py; }
	inline void AbsPhaseFromPhase(const TiltPhase3D& p, AbsTiltPhase3D& ap) { double cpsi = cos(p.pz), spsi = sin(p.pz); ap.px = cpsi*p.px - spsi*p.py; ap.py = spsi*p.px + cpsi*p.py; ap.pz = p.pz; }
	inline AbsTiltPhase2D AbsPhaseFromPhase(const TiltPhase2D& p, double fusedYaw) { AbsTiltPhase2D ap; AbsPhaseFromPhase(p, fusedYaw, ap); return ap; }
	inline AbsTiltPhase2D AbsPhaseFromPhase(const TiltPhase3D& p) { AbsTiltPhase3D ap; AbsPhaseFromPhase(p, ap); return ap; }

	// Conversion: Absolute tilt phase --> Tilt phase
	inline void PhaseFromAbsPhase(const AbsTiltPhase2D& ap, double fusedYaw, TiltPhase2D& p) { double cpsi = cos(fusedYaw), spsi = sin(fusedYaw); p.px = cpsi*ap.px + spsi*ap.py; p.py = cpsi*ap.py - spsi*ap.px; }
	inline void PhaseFromAbsPhase(const AbsTiltPhase3D& ap, TiltPhase3D& p) { double cpsi = cos(p.pz), spsi = sin(p.pz); p.px = cpsi*ap.px + spsi*ap.py; p.py = cpsi*ap.py - spsi*ap.px; p.pz = ap.pz; }
	inline TiltPhase2D PhaseFromAbsPhase(const AbsTiltPhase2D& ap, double fusedYaw) { TiltPhase2D p; PhaseFromAbsPhase(ap, fusedYaw, p); return p; }
	inline TiltPhase3D PhaseFromAbsPhase(const AbsTiltPhase3D& ap) { TiltPhase3D p; PhaseFromAbsPhase(ap, p); return p; }

	// Sum of absolute tilt rotations
	template<typename... Types> AbsTiltRot AbsTiltRotSum(const AbsTiltRot& atr, Types&&... args) { AbsTiltRot atrout; AbsYawTiltFromAbsPhase(TiltPhaseSum(AbsPhaseFromAbsYawTilt(atr), AbsPhaseFromAbsYawTilt(std::forward<Types>(args))...), atrout); return atrout; }

	// ######################################
	// #### Absolute rotation velocities ####
	// ######################################

	// Typedefs
	typedef TiltPhaseVel2D AbsTiltPhaseVel2D; // Absolute tilt phase velocity 2D struct with format: <pxVel, pyVel> = d/dt(AbsTiltPhase2D)
	typedef TiltPhaseVel3D AbsTiltPhaseVel3D; // Absolute tilt phase velocity 3D struct with format: <pxVel, pyVel, pzVel> = d/dt(AbsTiltPhase3D)

	// Conversion: Tilt phase velocity --> Absolute tilt phase velocity
	void AbsPhaseVelFromPhaseVel(const TiltPhaseVel2D& pdot, double fusedYaw, AbsTiltPhaseVel2D& apdot);
	void AbsPhaseVelFromPhaseVel(const TiltPhaseVel3D& pdot, const TiltAngles& t, AbsTiltPhaseVel3D& apdot);
	inline AbsTiltPhaseVel2D AbsPhaseVelFromPhaseVel(const TiltPhaseVel2D& pdot, double fusedYaw) { AbsTiltPhaseVel2D apdot; AbsPhaseVelFromPhaseVel(pdot, fusedYaw, apdot); return apdot; }
	inline AbsTiltPhaseVel3D AbsPhaseVelFromPhaseVel(const TiltPhaseVel3D& pdot, const TiltAngles& t) { AbsTiltPhaseVel3D apdot; AbsPhaseVelFromPhaseVel(pdot, t, apdot); return apdot; }

	// Conversion: Absolute tilt phase velocity --> Tilt phase velocity
	void PhaseVelFromAbsPhaseVel(const AbsTiltPhaseVel2D& apdot, double fusedYaw, TiltPhaseVel2D& pdot);
	void PhaseVelFromAbsPhaseVel(const AbsTiltPhaseVel3D& apdot, const TiltAngles& t, TiltPhaseVel3D& pdot);
	inline TiltPhaseVel2D PhaseVelFromAbsPhaseVel(const AbsTiltPhaseVel2D& apdot, double fusedYaw) { TiltPhaseVel2D pdot; PhaseVelFromAbsPhaseVel(apdot, fusedYaw, pdot); return pdot; }
	inline TiltPhaseVel3D PhaseVelFromAbsPhaseVel(const AbsTiltPhaseVel3D& apdot, const TiltAngles& t) { TiltPhaseVel3D pdot; PhaseVelFromAbsPhaseVel(apdot, t, pdot); return pdot; }

	// Conversion: Absolute tilt phase velocity --> Angular velocity
	void AngVelFromAbsPhaseVel(const AbsTiltPhaseVel3D& apdot, double absTiltAxisAngle, double tiltAngle, AngVel& angVel);
	inline void AngVelFromAbsPhaseVel(const AbsTiltPhaseVel2D& apdot, double absTiltAxisAngle, double tiltAngle, AngVel& angVel) { AngVelFromAbsPhaseVel(AbsTiltPhaseVel3D(apdot), absTiltAxisAngle, tiltAngle, angVel); }
	inline void AngVelFromAbsPhaseVel(const AbsTiltPhaseVel2D& apdot, const AbsTiltRot& atr, AngVel& angVel) { AngVelFromAbsPhaseVel(apdot, atr.absTiltAxisAngle, atr.tiltAngle, angVel); }
	inline void AngVelFromAbsPhaseVel(const AbsTiltPhaseVel3D& apdot, const AbsTiltRot& atr, AngVel& angVel) { AngVelFromAbsPhaseVel(apdot, atr.absTiltAxisAngle, atr.tiltAngle, angVel); }
	inline void AngVelFromAbsPhaseVel(const AbsTiltPhaseVel2D& apdot, const TiltAngles& t, AngVel& angVel) { AngVelFromAbsPhaseVel(apdot, t.tiltAxisAngle + t.fusedYaw, t.tiltAngle, angVel); }
	inline void AngVelFromAbsPhaseVel(const AbsTiltPhaseVel3D& apdot, const TiltAngles& t, AngVel& angVel) { AngVelFromAbsPhaseVel(apdot, t.tiltAxisAngle + t.fusedYaw, t.tiltAngle, angVel); }
	inline AngVel AngVelFromAbsPhaseVel(const AbsTiltPhaseVel2D& apdot, double absTiltAxisAngle, double tiltAngle) { AngVel angVel; AngVelFromAbsPhaseVel(apdot, absTiltAxisAngle, tiltAngle, angVel); return angVel; }
	inline AngVel AngVelFromAbsPhaseVel(const AbsTiltPhaseVel3D& apdot, double absTiltAxisAngle, double tiltAngle) { AngVel angVel; AngVelFromAbsPhaseVel(apdot, absTiltAxisAngle, tiltAngle, angVel); return angVel; }
	inline AngVel AngVelFromAbsPhaseVel(const AbsTiltPhaseVel2D& apdot, const AbsTiltRot& atr) { AngVel angVel; AngVelFromAbsPhaseVel(apdot, atr, angVel); return angVel; }
	inline AngVel AngVelFromAbsPhaseVel(const AbsTiltPhaseVel3D& apdot, const AbsTiltRot& atr) { AngVel angVel; AngVelFromAbsPhaseVel(apdot, atr, angVel); return angVel; }
	inline AngVel AngVelFromAbsPhaseVel(const AbsTiltPhaseVel2D& apdot, const TiltAngles& t) { AngVel angVel; AngVelFromAbsPhaseVel(apdot, t, angVel); return angVel; }
	inline AngVel AngVelFromAbsPhaseVel(const AbsTiltPhaseVel3D& apdot, const TiltAngles& t) { AngVel angVel; AngVelFromAbsPhaseVel(apdot, t, angVel); return angVel; }
}

#endif
// EOF