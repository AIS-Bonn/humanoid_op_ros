// Feedback gait kinematics for keypoint trajectory generation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_KINEMATICS_KT_H
#define FEED_KINEMATICS_KT_H

// Includes
#include <feed_gait/feed_common.h>
#include <feed_gait/trajectory/keypoint/feed_keypoint_traj_common.h>
#include <feed_gait/kinematics/feed_kinematics.h>
#include <humanoid_kinematics/robot_kinematics.h>
#include <type_traits>

// Feedback gait namespace
namespace feed_gait
{
	// Keypoint trajectory generation namespace
	namespace keypoint_traj
	{
		// Class forward declarations
		class KeypointTrajConfig;

		/**
		* @class KinKTConfig
		* 
		* @brief Kinematics interface for keypoint trajectories configuration parameters class.
		**/
		template<class Kinematics> class KinKTConfig : public KinConfigBase
		{
		private:
			// Static assertions
			static_assert(std::is_base_of<humanoid_kinematics::RobotKinematics, Kinematics>::value, "The Kinematics template parameter must derive from humanoid_kinematics::RobotKinematics");

			// Constructor
			KinKTConfig() = delete;

			// Get singleton instance of class
			static const KinKTConfig<Kinematics>& getInstance() = delete;
		};

		/**
		* @class FeedKinematicsKT
		*
		* @brief Feedback gait kinematics for keypoint trajectories class.
		**/
		template<class Kinematics> class FeedKinematicsKT : public FeedKinematics<Kinematics>
		{
		private:
			// Static assertions
			static_assert(std::is_base_of<humanoid_kinematics::RobotKinematics, Kinematics>::value, "The Kinematics template parameter must derive from humanoid_kinematics::RobotKinematics");

		public:
			// Typedefs
			typedef typename Kinematics::AbsPose AbsPose;
			typedef typename Kinematics::AbsArmPose AbsArmPose;
			typedef typename Kinematics::LegTipPose LegTipPose;
			typedef InvLegPosesT<Kinematics> InvLegPoses;
			typedef ConvenienceArrays<Kinematics> ConvArrays;

			// Constructor/destructor
			explicit FeedKinematicsKT(const KeypointTrajConfig& ktconfig);
			virtual ~FeedKinematicsKT() = default;

			// Configuration parameters
			const KinKTConfig<Kinematics>& ktkonfig;
			const KeypointTrajConfig& ktconfig;

			// Convenience arrays
			ConvArrays CA;

			// Leg step size generation functions
			void generateAbstractStepSizes(const CommonVars& CV, const Vec3& gcv, const AbsPose& baseAP, LimbPhases& limbPhase, LegTipPoints& LTP, MotionCentre& MC, FootYaws& footYaw, NominalFootTilts& nomFootTilt, double& stepHeightDist) const = delete;

			// Arm base motion generation functions
			void genArmBaseMotionSwing(const CommonVars& CV, const Vec3& gcv, double phase, const LimbPhases& limbPhase, AbsArmPose& AAP) const = delete;

			// Calculate final adjustment vector function
			Vec3 calcFinalAdjustVec(const InvLegPoses& ILP, const MotionCentre& MC, const Vec3& heightHat, double hipHeightMax) const = delete; // Note: MC.lineVec and adjustHat MUST be unit vectors!
		};
	}
}

// Include template specialisations for various kinematics
#include <feed_gait/trajectory/keypoint/feed_kinematics_kt_impl_serial.h>

// Include implementations that should occur in the header (must occur after all template specialisations are complete)
#include <feed_gait/trajectory/keypoint/feed_kinematics_kt_impl.h>

#endif
// EOF