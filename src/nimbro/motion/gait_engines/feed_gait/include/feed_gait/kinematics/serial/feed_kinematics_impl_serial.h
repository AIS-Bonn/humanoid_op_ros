// Feedback gait serial kinematics implementation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_KINEMATICS_IMPL_SERIAL_H
#define FEED_KINEMATICS_IMPL_SERIAL_H

// Includes
#include <feed_gait/kinematics/feed_kinematics.h>
#include <humanoid_kinematics/serial/serial_kinematics.h>
#include <config_server/parameter.h>

// Feedback gait namespace
namespace feed_gait
{
	// Namespaces
	namespace serial = hk::serial;

	//
	// KinConfig<serial::SerialKinematics> class
	//

	// KinConfig class template specialisation for serial::SerialKinematics
	template<> class KinConfig<serial::SerialKinematics> : public KinConfigBase
	{
	private:
		// Constructor
		KinConfig()
		 : TYPE_NAME(kinematicsTypeName(KT_SERIAL))
		 , CONFIG_PARAM_PATH(KIN_CONFIG_PARAM_PATH + TYPE_NAME + "/")

		 , haltArmAngleX        (CONFIG_PARAM_PATH + "haltPose/armAngleX", -0.1, 0.01, 0.6, 0.0)
		 , haltArmAngleY        (CONFIG_PARAM_PATH + "haltPose/armAngleY", -1.3, 0.01, 1.3, 0.0)
		 , haltArmRetraction    (CONFIG_PARAM_PATH + "haltPose/armRetraction", 0.0, 0.005, 0.8, 0.0)
		 , haltHeadAngleY       (CONFIG_PARAM_PATH + "haltPose/headAngleY", -0.5, 0.01, 0.5, 0.0)
		 , haltHeadAngleZ       (CONFIG_PARAM_PATH + "haltPose/headAngleZ", -0.5, 0.01, 0.5, 0.0)
		 , haltLegAngleX        (CONFIG_PARAM_PATH + "haltPose/legAngleX", -0.1, 0.01, 0.6, 0.0)
		 , haltLegAngleY        (CONFIG_PARAM_PATH + "haltPose/legAngleY", -1.0, 0.005, 1.0, 0.0)
		 , haltLegAngleZ        (CONFIG_PARAM_PATH + "haltPose/legAngleZ", -0.6, 0.005, 0.6, 0.0)
		 , haltLegFootAngleX    (CONFIG_PARAM_PATH + "haltPose/legFootAngleX", -0.4, 0.005, 0.4, 0.0)
		 , haltLegFootAngleY    (CONFIG_PARAM_PATH + "haltPose/legFootAngleY", -0.4, 0.005, 0.4, 0.0)
		 , haltLegRetraction    (CONFIG_PARAM_PATH + "haltPose/legRetraction", 0.0, 0.005, 0.8, 0.0)

		 , haltEffortArm        (CONFIG_PARAM_PATH + "haltPoseEffort/effortArm", 0.0, 0.01, 1.0, 0.2)
		 , haltEffortHead       (CONFIG_PARAM_PATH + "haltPoseEffort/effortHead", 0.0, 0.01, 1.0, 0.2)
		 , haltEffortHipYaw     (CONFIG_PARAM_PATH + "haltPoseEffort/effortHipYaw", 0.0, 0.01, 1.5, 0.2)
		 , haltEffortHipRoll    (CONFIG_PARAM_PATH + "haltPoseEffort/effortHipRoll", 0.0, 0.01, 1.5, 0.2)
		 , haltEffortHipPitch   (CONFIG_PARAM_PATH + "haltPoseEffort/effortHipPitch", 0.0, 0.01, 1.5, 0.2)
		 , haltEffortKneePitch  (CONFIG_PARAM_PATH + "haltPoseEffort/effortKneePitch", 0.0, 0.01, 1.5, 0.2)
		 , haltEffortAnklePitch (CONFIG_PARAM_PATH + "haltPoseEffort/effortAnklePitch", 0.0, 0.01, 1.5, 0.2)
		 , haltEffortAnkleRoll  (CONFIG_PARAM_PATH + "haltPoseEffort/effortAnkleRoll", 0.0, 0.01, 1.5, 0.2)

		 , biasLegAngleX        (CONFIG_PARAM_PATH + "poseBias/legAngleX", -0.07, 0.005, 0.07, 0.0)
		 , biasFootAngleX       (CONFIG_PARAM_PATH + "poseBias/footAngleX", -0.07, 0.005, 0.07, 0.0)
		 , biasLegRet           (CONFIG_PARAM_PATH + "poseBias/legRet", -0.07, 0.005, 0.07, 0.0)

		 , limLegAngleXBuf      (CONFIG_PARAM_PATH + "poseLimits/legAngleX/buf", 0.0, 0.01, 0.3, 0.1)
		 , limLegAngleXMax      (CONFIG_PARAM_PATH + "poseLimits/legAngleX/max", 0.2, 0.01, 1.2, 0.5)
		 , limLegAngleXMin      (CONFIG_PARAM_PATH + "poseLimits/legAngleX/min", -0.5, 0.01, 0.2, -0.1)
		 , limLegAngleYBuf      (CONFIG_PARAM_PATH + "poseLimits/legAngleY/buf", 0.0, 0.01, 0.3, 0.1)
		 , limLegAngleYMax      (CONFIG_PARAM_PATH + "poseLimits/legAngleY/max", 0.0, 0.01, 1.2, 0.5)
		 , limLegAngleYMin      (CONFIG_PARAM_PATH + "poseLimits/legAngleY/min", -1.2, 0.01, 0.0, -0.5)
		 , limLegAngleZBuf      (CONFIG_PARAM_PATH + "poseLimits/legAngleZ/buf", 0.0, 0.01, 0.3, 0.1)
		 , limLegAngleZMax      (CONFIG_PARAM_PATH + "poseLimits/legAngleZ/max", 0.0, 0.01, 1.5, 1.0)
		 , limLegAngleZMin      (CONFIG_PARAM_PATH + "poseLimits/legAngleZ/min", -1.5, 0.01, 0.0, -1.0)
		 , limFootAngleXBuf     (CONFIG_PARAM_PATH + "poseLimits/footAngleX/buf", 0.0, 0.01, 0.3, 0.1)
		 , limFootAngleXMax     (CONFIG_PARAM_PATH + "poseLimits/footAngleX/max", 0.0, 0.01, 1.0, 0.5)
		 , limFootAngleXMin     (CONFIG_PARAM_PATH + "poseLimits/footAngleX/min", -1.0, 0.01, 0.0, -0.5)
		 , limFootAngleYBuf     (CONFIG_PARAM_PATH + "poseLimits/footAngleY/buf", 0.0, 0.01, 0.3, 0.1)
		 , limFootAngleYMax     (CONFIG_PARAM_PATH + "poseLimits/footAngleY/max", 0.0, 0.01, 1.0, 0.5)
		 , limFootAngleYMin     (CONFIG_PARAM_PATH + "poseLimits/footAngleY/min", -1.0, 0.01, 0.0, -0.5)
		 , limLegRetBuf         (CONFIG_PARAM_PATH + "poseLimits/legRet/buf", 0.0, 0.001, 0.08, 0.0)
		 , limLegRetMax         (CONFIG_PARAM_PATH + "poseLimits/legRet/max", 0.1, 0.01, 1.0, 1.0)
		 , limLegRetMin         (CONFIG_PARAM_PATH + "poseLimits/legRet/min", 0.0, 0.001, 0.08, 0.0)
		 , limAnklePosZMax      (CONFIG_PARAM_PATH + "poseLimits/anklePosZ/max", 0.5, 0.01, 1.0, 0.7)
		 , limShoulderPitchBuf  (CONFIG_PARAM_PATH + "poseLimits/shoulderPitch/buf", 0.0, 0.01, 0.3, 0.1)
		 , limShoulderPitchMax  (CONFIG_PARAM_PATH + "poseLimits/shoulderPitch/max", 0.5, 0.01, 2.0, 1.5)
		 , limShoulderPitchMin  (CONFIG_PARAM_PATH + "poseLimits/shoulderPitch/min", -2.0, 0.01, -0.5, -1.5)
		 , limShoulderRollBuf   (CONFIG_PARAM_PATH + "poseLimits/shoulderRoll/buf", 0.0, 0.01, 0.3, 0.1)
		 , limShoulderRollMax   (CONFIG_PARAM_PATH + "poseLimits/shoulderRoll/max", 0.5, 0.01, 1.55, 1.5)
		 , limShoulderRollMin   (CONFIG_PARAM_PATH + "poseLimits/shoulderRoll/min", -0.3, 0.01, 0.5, -0.1)
		 , limShoulderRollCoMBuf(CONFIG_PARAM_PATH + "poseLimits/shoulderRollCoM/buf", 0.0, 0.01, 0.3, 0.2)
		 , limShoulderRollCoMMax(CONFIG_PARAM_PATH + "poseLimits/shoulderRollCoM/max", 0.5, 0.01, 1.55, 1.5)
		 , limElbowPitchBuf     (CONFIG_PARAM_PATH + "poseLimits/elbowPitch/buf", 0.0, 0.01, 0.3, 0.1)
		 , limElbowPitchMax     (CONFIG_PARAM_PATH + "poseLimits/elbowPitch/max", 1.0, 0.01, 2.5, 2.0)
		 , limElbowPitchMin     (CONFIG_PARAM_PATH + "poseLimits/elbowPitch/min", 0.0, 0.01, 0.5, 0.0)
		{
			// Configure limit leg retraction callbacks
			boost::function<void (const float&)> limLegRetCBFunc = boost::bind(&KinConfig<serial::SerialKinematics>::limLegRetCB, this);
			limLegRetBuf.setCallback(limLegRetCBFunc);
			limLegRetMax.setCallback(limLegRetCBFunc);
			limLegRetMin.setCallback(limLegRetCBFunc);
			limLegRetCB();
		}

	public:
		// Get singleton instance of class
		static const KinConfig<serial::SerialKinematics>& getInstance() { static thread_local KinConfig<serial::SerialKinematics> konfig; return konfig; }

		// Constants
		const std::string TYPE_NAME;
		const std::string CONFIG_PARAM_PATH;

		// Halt pose position parameters
		config_server::Parameter<float> haltArmAngleX;         //!< @brief Halt pose: Roll angle of the arm axes (positive is away from the body for both arms)
		config_server::Parameter<float> haltArmAngleY;         //!< @brief Halt pose: Pitch angle of the arm axes (positive is moving the arms towards the back for both arms)
		config_server::Parameter<float> haltArmRetraction;     //!< @brief Halt pose: Retraction of the arms (0 = Fully extended, 1 = Fully contracted)
		config_server::Parameter<float> haltHeadAngleY;        //!< @brief Halt pose: Pitch angle of the head (positive is tilting the head down)
		config_server::Parameter<float> haltHeadAngleZ;        //!< @brief Halt pose: Yaw angle of the head (positive is panning the head left)
		config_server::Parameter<float> haltLegAngleX;         //!< @brief Halt pose: Roll angle of the leg axes (positive is away from the body for both legs)
		config_server::Parameter<float> haltLegAngleY;         //!< @brief Halt pose: Pitch angle of the leg axes (positive is moving the legs towards the back for both legs)
		config_server::Parameter<float> haltLegAngleZ;         //!< @brief Halt pose: Yaw angle of the leg axes (toe-out is positive for both legs)
		config_server::Parameter<float> haltLegFootAngleX;     //!< @brief Halt pose: Roll angle of the feet relative to the trunk (positive is tilting the inside foot edges down for both feet)
		config_server::Parameter<float> haltLegFootAngleY;     //!< @brief Halt pose: Pitch angle of the feet relative to the trunk (positive is tilting the front foot edges down for both feet)
		config_server::Parameter<float> haltLegRetraction;     //!< @brief Halt pose: Retraction of the legs (0 = Fully extended, 1 = Fully contracted)

		// Halt pose effort parameters
		config_server::Parameter<float> haltEffortArm;         //!< @brief Halt pose: Joint effort to use for the arms (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortHead;        //!< @brief Halt pose: Joint effort to use for the head (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortHipYaw;      //!< @brief Halt pose: Joint effort to use for the leg hip yaw (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortHipRoll;     //!< @brief Halt pose: Joint effort to use for the leg hip roll (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortHipPitch;    //!< @brief Halt pose: Joint effort to use for the leg hip pitch (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortKneePitch;   //!< @brief Halt pose: Joint effort to use for the leg knee pitch (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortAnklePitch;  //!< @brief Halt pose: Joint effort to use for the leg ankle pitch (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortAnkleRoll;   //!< @brief Halt pose: Joint effort to use for the leg ankle roll (in the range `[0,1]`)

		// Pose bias parameters
		config_server::Parameter<float> biasLegAngleX;         //!< @brief Bias to use for the leg angle X
		config_server::Parameter<float> biasFootAngleX;        //!< @brief Bias to use for the foot angle X
		config_server::Parameter<float> biasLegRet;            //!< @brief Bias to use for the leg retraction

		// Pose limit parameters
		config_server::Parameter<float> limLegAngleXBuf;       //!< @brief Buffer for soft limiting of the leg angle X
		config_server::Parameter<float> limLegAngleXMax;       //!< @brief Maximum allowed outwards leg angle X
		config_server::Parameter<float> limLegAngleXMin;       //!< @brief Minimum allowed outwards leg angle X
		config_server::Parameter<float> limLegAngleYBuf;       //!< @brief Buffer for soft limiting of the leg angle Y
		config_server::Parameter<float> limLegAngleYMax;       //!< @brief Maximum allowed leg angle Y
		config_server::Parameter<float> limLegAngleYMin;       //!< @brief Minimum allowed leg angle Y
		config_server::Parameter<float> limLegAngleZBuf;       //!< @brief Buffer for soft limiting of the leg angle Z
		config_server::Parameter<float> limLegAngleZMax;       //!< @brief Maximum allowed outwards leg angle Z
		config_server::Parameter<float> limLegAngleZMin;       //!< @brief Minimum allowed outwards leg angle Z
		config_server::Parameter<float> limFootAngleXBuf;      //!< @brief Buffer for soft limiting of the foot angle X
		config_server::Parameter<float> limFootAngleXMax;      //!< @brief Maximum allowed outwards foot angle X
		config_server::Parameter<float> limFootAngleXMin;      //!< @brief Minimum allowed outwards foot angle X
		config_server::Parameter<float> limFootAngleYBuf;      //!< @brief Buffer for soft limiting of the foot angle Y
		config_server::Parameter<float> limFootAngleYMax;      //!< @brief Maximum allowed foot angle Y
		config_server::Parameter<float> limFootAngleYMin;      //!< @brief Minimum allowed foot angle Y
		config_server::Parameter<float> limLegRetBuf;          //!< @brief Buffer for soft limiting of the leg retraction
		config_server::Parameter<float> limLegRetMax;          //!< @brief Maximum allowed leg retraction
		config_server::Parameter<float> limLegRetMin;          //!< @brief Minimum allowed leg retraction
		config_server::Parameter<float> limAnklePosZMax;       //!< @brief Maximum allowed ankle position z coordinate (in units of inverse leg scale)
		config_server::Parameter<float> limShoulderPitchBuf;   //!< @brief Buffer for soft limiting of the shoulder pitch
		config_server::Parameter<float> limShoulderPitchMax;   //!< @brief Maximum allowed shoulder pitch
		config_server::Parameter<float> limShoulderPitchMin;   //!< @brief Minimum allowed shoulder pitch
		config_server::Parameter<float> limShoulderRollBuf;    //!< @brief Buffer for soft limiting of the shoulder roll
		config_server::Parameter<float> limShoulderRollMax;    //!< @brief Maximum allowed outwards shoulder roll
		config_server::Parameter<float> limShoulderRollMin;    //!< @brief Minimum allowed outwards shoulder roll
		config_server::Parameter<float> limShoulderRollCoMBuf; //!< @brief Buffer for soft limiting of the shoulder roll when avoiding singularities while calculating it from a CoM
		config_server::Parameter<float> limShoulderRollCoMMax; //!< @brief Maximum allowed absolute shoulder roll when avoiding singularities while calculating it from a CoM
		config_server::Parameter<float> limElbowPitchBuf;      //!< @brief Buffer for soft limiting of the elbow pitch
		config_server::Parameter<float> limElbowPitchMax;      //!< @brief Maximum allowed contracting elbow pitch
		config_server::Parameter<float> limElbowPitchMin;      //!< @brief Minimum allowed contracting elbow pitch

		// Derived configuration parameters
		double limLegRetSoftMin;                               //!< @brief Minimum allowed leg retraction so that no soft limiting is required

	private:
		// Configuration parameter callbacks
		void limLegRetCB()
		{
			double maxBuf = std::max<double>(0.5*(limLegRetMax() - limLegRetMin()), 0.0);
			double buf = std::min<double>(limLegRetBuf(), maxBuf);
			limLegRetSoftMin = limLegRetMin() + buf;
		}
	};

	//
	// FeedKinematics<serial::SerialKinematics> class
	//

	// Gait halt pose functions
	template<> void FeedKinematics<serial::SerialKinematics>::getHaltPose(PoseCommand::DblVec& pos) const;
	template<> void FeedKinematics<serial::SerialKinematics>::getHaltPoseEffort(PoseCommand::DblVec& effort) const;
	template<> void FeedKinematics<serial::SerialKinematics>::getHaltPoseSuppCoeff(PoseCommand::SuppCoeff& suppCoeff) const;
	template<> void FeedKinematics<serial::SerialKinematics>::getHaltPose(JointPose& JP) const;
	template<> void FeedKinematics<serial::SerialKinematics>::getHaltPose(AbsPose& AP) const;
	template<> void FeedKinematics<serial::SerialKinematics>::getHaltPose(InvPose& IP) const;
	template<> void FeedKinematics<serial::SerialKinematics>::getHaltPoseEffort(JointEffort& JE) const;

	// Safe inverse kinematics functions
	template<> bool FeedKinematics<serial::SerialKinematics>::JointFromInvSafe(const InvLegPose& ILP, JointLegPose& JLP) const;
	template<> bool FeedKinematics<serial::SerialKinematics>::AbsFromInvSafe(const InvLegPose& ILP, AbsLegPose& ALP) const;
	template<> bool FeedKinematics<serial::SerialKinematics>::JointAbsFromInvSafe(const InvLegPose& ILP, JointLegPose& JLP, AbsLegPose& ALP) const;

	// CoM inverse kinematics functions
	template<> void FeedKinematics<serial::SerialKinematics>::JointFromCoMRay(const Vec3& CoMRay, const JointArmPose& JAPRef, JointArmPose& JAP) const;

	// Pose bias functions
	template<> void FeedKinematics<serial::SerialKinematics>::biasAbsPose(AbsLegPose& ALP) const;

	// Pose coercion functions
	template<> bool FeedKinematics<serial::SerialKinematics>::JointCoerceSoft(JointArmPose& JAP) const;
	template<> bool FeedKinematics<serial::SerialKinematics>::AbsCoerceSoft(AbsLegPose& ALP) const;
}

#endif
// EOF