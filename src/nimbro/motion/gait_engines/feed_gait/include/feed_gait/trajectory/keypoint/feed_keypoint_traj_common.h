// Feedback gait keypoint trajectory generation common
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_KEYPOINT_TRAJ_COMMON_H
#define FEED_KEYPOINT_TRAJ_COMMON_H

// Includes
#include <feed_gait/feed_common.h>
#include <humanoid_kinematics/robot_kinematics.h>
#include <type_traits>

// Feedback gait namespace
namespace feed_gait
{
	// Keypoint trajectory generation namespace
	namespace keypoint_traj
	{
		// Using declarations
		using hk::LimbIndex;
		using hk::LEFT;
		using hk::RIGHT;
		using hk::NUM_LR;
		using hk::LimbSign;
		using hk::pose_classes::LegTipPoint;
		using hk::pose_classes::LegTipPointVel;

		//! Step size generator type enumeration
		enum SSGType
		{
			SSGT_FIRST = 0,
			SSGT_ABSTRACT = SSGT_FIRST,  // Abstract space step size generator
			SSGT_COUNT,
			SSGT_DEFAULT = SSGT_ABSTRACT
		};

		//! Arm base motion type enumeration
		enum ABMType
		{
			ABMT_FIRST = 0,
			ABMT_SWING = ABMT_FIRST,  // Swing method
			ABMT_COUNT,
			ABMT_DEFAULT = ABMT_SWING
		};

		//! Keypoint enumeration
		enum Keypoint
		{
			KEY_A = 0,  // Start of double support (foot strike of this foot)
			KEY_B,      // End of double support, start of single support (foot lift of other foot)
			KEY_N,      // Middle of single support phase (neutral point)
			KEY_C,      // Start of double support, end of single support (foot strike of other foot)
			KEY_D,      // End of double support (foot lift of this foot)
			KEY_E,      // Intermediate swing start point
			KEY_F,      // Middle of swing phase
			KEY_G,      // Intermediate swing stop point
			NUM_KEYS
		};

		//! Convenience arrays struct
		template<class Kinematics> struct ConvenienceArrays
		{
		private:
			// Static assertions
			static_assert(std::is_base_of<humanoid_kinematics::RobotKinematics, Kinematics>::value, "The Kinematics template parameter must derive from humanoid_kinematics::RobotKinematics");

		public:
			// Typedefs
			typedef typename Kinematics::AbsLegPose AbsLegPose;

			// Constants
			static const int NUM_ABS_LEG = AbsLegPose::NUM_FIELDS;

			// Constructor
			ConvenienceArrays()
			{
				for(LimbIndex l = hk::INDEX0; l < NUM_LR; l = (LimbIndex) (l + 1))
					limbIndices[l] = l;
				for(int n = 0; n < NUM_KEYS; n++)
					keypoints[n] = n;
				for(int k = 0; k < NUM_ABS_LEG; k++)
					absFields[k] = k;
			}

			// Convenience arrays
			LimbIndex limbIndices[NUM_LR]; // Convenience array to loop through all limb indices using: for(LimbIndex l : limbIndices)
			int keypoints[NUM_KEYS];       // Convenience array to loop through all keypoints using: for(int n : keypoints)
			int absFields[NUM_ABS_LEG];    // Convenience array to loop through all abstract leg pose fields using: for(int k : absFields)
		};

		//! Motion centre struct
		struct MotionCentre
		{
			Vec3 point;   // Motion centre point
			Vec3 lineVec; // Unit vector that in combination with the motion centre point defines the motion centre line
		};

		//! Nominal foot tilt struct
		struct NominalFootTilt
		{
			// Set functions
			void set(const rot_conv::TiltAngles& t);
			void set(const Quat& q);

			// Data members
			rot_conv::TiltAngles tilt;    // Tilt angles representation of the nominal orientation of the foot
			rot_conv::AbsTiltRot absTilt; // Absolute tilt rotation representation of the nominal orientation of the foot
		};

		//! Common variables struct
		struct CommonVars
		{
			// Position vectors
			Vec3 hipPointLocal[NUM_LR];       // Hip point in local coordinates (relative to leg cartesian origin)
			Vec3 hipPointGlobal[NUM_LR];      // Hip point in global coordinates (relative to hip centre point)
			Vec3 hipCentrePointLocal[NUM_LR]; // Hip centre point in local coordinates (relative to leg cartesian origin)
			Vec3 localToGlobal[NUM_LR];       // Add this to local vectors to get the corresponding global vectors

			// Nominal ground plane rotations
			Quat qNB;                         // Pure pitch rotation from N to B
			Quat qBN;                         // Pure pitch rotation from B to N
			Rotmat RBN;                       // Pure pitch rotation from B to N
			Vec3 BzN;                         // z-axis of N in B coordinates

			// Swing ground plane rotations
			Quat qNS;                         // Tilt rotation from N to S
			Quat qBS;                         // Rotation from B to S
			Rotmat RBS;                       // Rotation from B to S
			Vec3 BxS;                         // x-axis of S in B coordinates
			Vec3 ByS;                         // y-axis of S in B coordinates
			Vec3 BzS;                         // z-axis of S in B coordinates
			Quat qBNS;                        // Rotation relative to B from N to S
			Rotmat RBNS;                      // Rotation relative to B from N to S

			// Intermediate ground plane rotations
			Quat qNI;                         // Tilt rotation from N to I
			Quat qBI;                         // Rotation from B to I
			Rotmat RBI;                       // Rotation from B to I
			Vec3 BxI;                         // x-axis of I in B coordinates
			Vec3 ByI;                         // y-axis of I in B coordinates
			Vec3 BzI;                         // z-axis of I in B coordinates
			Quat qBNI;                        // Rotation relative to B from N to I
			Rotmat RBNI;                      // Rotation relative to B from N to I

			// Support ground plane rotations
			Quat qNJ;                         // Tilt rotation from N to J
			Quat qBJ;                         // Rotation from B to J
			Vec3 BzJ;                         // z-axis of J in B coordinates
			Quat qBNJ;                        // Rotation relative to B from N to J
			Rotmat RBNJ;                      // Rotation relative to B from N to J

			// Leaned ground plane rotations
			Quat qNL;                         // Tilt rotation from N to L
			Quat qBL;                         // Rotation from B to L
			Rotmat RBL;                       // Rotation from B to L
			Vec3 BzL;                         // z-axis of L in B coordinates
			Quat qLB;                         // Rotation from L to B
			Quat qBNL;                        // Rotation relative to B from N to L
			Rotmat RBNL;                      // Rotation relative to B from N to L
		};

		// Typedefs
		template<class Kinematics> using InvLegPosesT = typename Kinematics::InvLegPose[NUM_LR][NUM_KEYS];
		typedef double LimbPhases[NUM_KEYS];
		typedef double FootYaws[NUM_LR][NUM_KEYS];
		typedef NominalFootTilt NominalFootTilts[NUM_LR];
		typedef LegTipPoint LegTipPoints[NUM_LR][NUM_KEYS];
		typedef LegTipPointVel LegTipPointVels[NUM_LR][NUM_KEYS];
	}
}

#endif
// EOF