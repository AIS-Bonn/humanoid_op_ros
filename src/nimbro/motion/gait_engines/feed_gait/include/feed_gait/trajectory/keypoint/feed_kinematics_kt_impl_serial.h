// Feedback gait serial kinematics for keypoint trajectory generation implementation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_KINEMATICS_KT_IMPL_SERIAL_H
#define FEED_KINEMATICS_KT_IMPL_SERIAL_H

// Includes
#include <feed_gait/trajectory/keypoint/feed_kinematics_kt.h>
#include <humanoid_kinematics/serial/serial_kinematics.h>

// Feedback gait namespace
namespace feed_gait
{
	// Keypoint trajectory generation namespace
	namespace keypoint_traj
	{
		//
		// KinKTConfig<serial::SerialKinematics> class
		//

		// KinKTConfig class template specialisation for serial::SerialKinematics
		template<> class KinKTConfig<serial::SerialKinematics> : public KinConfigBase
		{
		private:
			// Constructor
			KinKTConfig()
			 : TYPE_NAME(trajectoryTypeName(TT_KEYPOINT))
			 , KIN_NAME(kinematicsTypeName(KT_SERIAL))
			 , CONFIG_PARAM_PATH(TRAJ_CONFIG_PARAM_PATH + TYPE_NAME + "/" + KIN_CONFIG_PARAM_EXT + KIN_NAME + "/")

			 , asLegLatPushoutMagGradX (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legPushout/latPushoutMagGradX", 0.0, 0.005, 0.15, 0.0)
			 , asLegLatPushoutMagGradY (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legPushout/latPushoutMagGradY", 0.0, 0.005, 0.15, 0.0)
			 , asLegLatPushoutMagGradZ (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legPushout/latPushoutMagGradZ", 0.0, 0.005, 0.15, 0.0)
			 , asLegLatPushoutMagMax   (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legPushout/latPushoutMagMax", 0.0, 0.005, 0.3, 0.3)
			 , asLegRotVPushoutMagGradZ(CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legPushout/rotVPushoutMagGradZ", 0.0, 0.005, 0.15, 0.0)
			 , asLegRotVPushoutMagMax  (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legPushout/rotVPushoutMagMax", 0.0, 0.005, 0.3, 0.3)
			 , asLegLatHipSwingMag     (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legHipSwing/latHipSwingMag", 0.0, 0.005, 0.15, 0.0)
			 , asLegLatHipSwingMagGradX(CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legHipSwing/latHipSwingMagGradX", 0.0, 0.005, 0.15, 0.0)
			 , asLegLatHipSwingMagGradY(CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legHipSwing/latHipSwingMagGradY", 0.0, 0.005, 0.15, 0.0)
			 , asLegSagSwingMagGradX   (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legSwing/sagSwingMagGradX", 0.0, 0.005, 0.4, 0.15)
			 , asLegLatSwingMagGradY   (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legSwing/latSwingMagGradY", 0.0, 0.005, 0.4, 0.15)
			 , asLegRotSwingMagGradZ   (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legSwing/rotSwingMagGradZ", 0.0, 0.005, 0.4, 0.15)
			 , asLegStepHeight         (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legStepHeight/stepHeight", 0.0, 0.005, 0.15, 0.05)
			 , asLegStepHeightGradX    (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legStepHeight/stepHeightGradX", 0.0, 0.005, 0.15, 0.0)
			 , asLegStepHeightGradY    (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legStepHeight/stepHeightGradY", 0.0, 0.005, 0.15, 0.0)
			 , asLegStepHeightMax      (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/legStepHeight/stepHeightMax", 0.0, 0.005, 0.3, 0.3)
			 , asTuningNoLegPushout    (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/tuning/noLegPushout", false)
			 , asTuningNoLegHipSwing   (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/tuning/noLegHipSwing", false)
			 , asTuningNoLegSwing      (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/tuning/noLegSwing", false)
			 , asTuningNoLegStepHeight (CONFIG_PARAM_PATH + "stepSizeGenerator/abstract/tuning/noLegStepHeight", false)

			 , saArmSagSwingMag        (CONFIG_PARAM_PATH + "armBaseMotion/swing/armSwing/sagSwingMag", 0.0, 0.005, 0.3, 0.0)
			 , saArmSagSwingMagGradX   (CONFIG_PARAM_PATH + "armBaseMotion/swing/armSwing/sagSwingMagGradX", 0.0, 0.005, 0.4, 0.15)
			 , saArmSagSwingMagMax     (CONFIG_PARAM_PATH + "armBaseMotion/swing/armSwing/sagSwingMagMax", 0.0, 0.005, 0.6, 0.6)
			{}

		public:
			// Get singleton instance of class
			static const KinKTConfig<serial::SerialKinematics>& getInstance() { static thread_local KinKTConfig<serial::SerialKinematics> ktkonfig; return ktkonfig; }

			// Constants
			const std::string TYPE_NAME;
			const std::string KIN_NAME;
			const std::string CONFIG_PARAM_PATH;

			// Abstract step size generator parameters
			config_server::Parameter<float> asLegLatPushoutMagGradX;  //!< @brief Gradient of the outwards hip-roll-based lateral leg pushout (nominally zero radians) with respect to the absolute gait command x-velocity
			config_server::Parameter<float> asLegLatPushoutMagGradY;  //!< @brief Gradient of the outwards hip-roll-based lateral leg pushout (nominally zero radians) with respect to the absolute gait command y-velocity
			config_server::Parameter<float> asLegLatPushoutMagGradZ;  //!< @brief Gradient of the outwards hip-roll-based lateral leg pushout (nominally zero radians) with respect to the absolute gait command z-velocity
			config_server::Parameter<float> asLegLatPushoutMagMax;    //!< @brief Maximum allowed outwards hip-roll-based lateral leg pushout
			config_server::Parameter<float> asLegRotVPushoutMagGradZ; //!< @brief Gradient of the rotational leg V pushout magnitude (nominally zero radians) with respect to the absolute gait command z-velocity
			config_server::Parameter<float> asLegRotVPushoutMagMax;   //!< @brief Maximum allowed rotational leg V pushout magnitude
			config_server::Parameter<float> asLegLatHipSwingMag;      //!< @brief Nominal lateral hip swing magnitude (units of radians) to use for a zero gait command vector
			config_server::Parameter<float> asLegLatHipSwingMagGradX; //!< @brief Gradient of the lateral hip swing magnitude with respect to the absolute gait command x-velocity
			config_server::Parameter<float> asLegLatHipSwingMagGradY; //!< @brief Gradient of the lateral hip swing magnitude with respect to the absolute gait command y-velocity
			config_server::Parameter<float> asLegSagSwingMagGradX;    //!< @brief Gradient of the sagittal leg swing magnitude (nominally zero radians) with respect to the gait command x-velocity
			config_server::Parameter<float> asLegLatSwingMagGradY;    //!< @brief Gradient of the lateral leg swing magnitude (nominally zero radians) with respect to the gait command y-velocity
			config_server::Parameter<float> asLegRotSwingMagGradZ;    //!< @brief Gradient of the rotational leg swing magnitude (nominally zero radians) with respect to the gait command z-velocity
			config_server::Parameter<float> asLegStepHeight;          //!< @brief Nominal swing leg step height (away from the ground, in units of inverse leg scale) to use for a zero gait command vector
			config_server::Parameter<float> asLegStepHeightGradX;     //!< @brief Gradient of the swing leg step height with respect to the absolute gait command x-velocity
			config_server::Parameter<float> asLegStepHeightGradY;     //!< @brief Gradient of the swing leg step height with respect to the absolute gait command y-velocity
			config_server::Parameter<float> asLegStepHeightMax;       //!< @brief Maximum allowed swing leg step height
			config_server::Parameter<bool>  asTuningNoLegPushout;     //!< @brief Disable leg pushout
			config_server::Parameter<bool>  asTuningNoLegHipSwing;    //!< @brief Disable leg hip swing
			config_server::Parameter<bool>  asTuningNoLegSwing;       //!< @brief Disable leg swing
			config_server::Parameter<bool>  asTuningNoLegStepHeight;  //!< @brief Disable leg step height

			// Swing method parameters
			config_server::Parameter<float> saArmSagSwingMag;         //!< @brief Nominal sagittal arm swing magnitude (units of radians) to use for a zero gait command vector
			config_server::Parameter<float> saArmSagSwingMagGradX;    //!< @brief Gradient of the sagittal arm swing magnitude with respect to the gait command x-velocity
			config_server::Parameter<float> saArmSagSwingMagMax;      //!< @brief Maximum allowed sagittal arm swing magnitude
		};

		//
		// FeedKinematicsKT<serial::SerialKinematics> class
		//

		// Leg step size generation functions
		template<> void FeedKinematicsKT<serial::SerialKinematics>::generateAbstractStepSizes(const CommonVars& CV, const Vec3& gcv, const AbsPose& baseAP, LimbPhases& limbPhase, LegTipPoints& LTP, MotionCentre& MC, FootYaws& footYaw, NominalFootTilts& nomFootTilt, double& stepHeightDist) const;

		// Arm base motion generation functions
		template<> void FeedKinematicsKT<serial::SerialKinematics>::genArmBaseMotionSwing(const CommonVars& CV, const Vec3& gcv, double phase, const LimbPhases& limbPhase, AbsArmPose& AAP) const;

		// Calculate final adjustment vector function
		template<> Vec3 FeedKinematicsKT<serial::SerialKinematics>::calcFinalAdjustVec(const InvLegPoses& ILP, const MotionCentre& MC, const Vec3& heightHat, double hipHeightMax) const;
	}
}

#endif
// EOF