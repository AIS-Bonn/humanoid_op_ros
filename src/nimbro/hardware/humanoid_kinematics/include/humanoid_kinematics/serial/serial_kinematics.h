// Humanoid kinematics - Serial kinematics
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef SERIAL_KINEMATICS_H
#define SERIAL_KINEMATICS_H

// Includes
#include <humanoid_kinematics/kinematics_common.h>
#include <humanoid_kinematics/robot_kinematics.h>
#include <humanoid_kinematics/serial/serial_pose_classes.h>
#include <rc_utils/math_funcs.h>

// Humanoid kinematics namespace
namespace humanoid_kinematics
{
	/**
	* @namespace serial
	*
	* @brief Serial kinematics namespace.
	**/
	namespace serial
	{
		/**
		* @class SerialConfig
		* 
		* @brief Serial kinematics configuration parameters class.
		**/
		class SerialConfig
		{
		private:
			// Constructor
			SerialConfig()
				: CONFIG_PARAM_PATH(ROOT_CONFIG_PARAM_PATH + "serial/")

				, legLinkLength(CONFIG_PARAM_PATH + "legLinkLength", 0.01, 0.005, 0.6, 0.2)
				, hipOffsetX   (CONFIG_PARAM_PATH + "hipOffsetX", -0.1, 0.001, 0.1, 0.0)
				, hipOffsetY   (CONFIG_PARAM_PATH + "hipOffsetY", -0.1, 0.001, 0.1, 0.0)
				, footOffsetX  (CONFIG_PARAM_PATH + "footOffsetX", -0.25, 0.005, 0.25, 0.0)
				, footOffsetY  (CONFIG_PARAM_PATH + "footOffsetY", -0.15, 0.005, 0.15, 0.0)
				, footOffsetZ  (CONFIG_PARAM_PATH + "footOffsetZ", 0.0, 0.005, 0.2, 0.0)

				, armLinkLength(CONFIG_PARAM_PATH + "armLinkLength", 0.01, 0.005, 0.6, 0.2)
			{
				// Configure leg callbacks
				boost::function<void (const float&)> legCBFunc = boost::bind(&SerialConfig::legCB, this);
				legLinkLength.setCallback(legCBFunc);
				hipOffsetX.setCallback(legCBFunc);
				hipOffsetY.setCallback(legCBFunc);
				footOffsetX.setCallback(legCBFunc);
				footOffsetY.setCallback(legCBFunc);
				footOffsetZ.setCallback(legCBFunc);
				legCB();

				// Configure arm callbacks
				boost::function<void (const float&)> armCBFunc = boost::bind(&SerialConfig::armCB, this);
				armLinkLength.setCallback(armCBFunc);
				armCB();
			}

			// Ensure class remains a singleton
			SerialConfig(const SerialConfig&) = delete;
			SerialConfig& operator=(const SerialConfig&) = delete;

		public:
			// Get singleton instance of class
			static const SerialConfig& getInstance() { static thread_local SerialConfig sconfig; return sconfig; }

			// Constants
			const std::string CONFIG_PARAM_PATH;

			// Robot leg specifications
			config_server::Parameter<float> legLinkLength; //!< @brief Length of each of the upper and lower leg links
			config_server::Parameter<float> hipOffsetX;    //!< @brief Forward x-offset from the hip point to the hip PR point (rotates with hip yaw)
			config_server::Parameter<float> hipOffsetY;    //!< @brief Outward y-offset from the hip point to the hip PR point (rotates with hip yaw, limb sign dependent)
			config_server::Parameter<float> footOffsetX;   //!< @brief Forward x-offset from the ankle point to the leg tip point (foot plate geometric center)
			config_server::Parameter<float> footOffsetY;   //!< @brief Outward y-offset from the ankle point to the leg tip point (foot plate geometric center)
			config_server::Parameter<float> footOffsetZ;   //!< @brief Downward z-offset from the ankle point to the leg tip point (foot plate geometric center)

			// Robot arm specifications
			config_server::Parameter<float> armLinkLength; //!< @brief Length of each of the upper and lower arm links

			// Derived configuration parameters
			double LL;                                     //!< @brief Alias for leg link length
			double LLdbl;                                  //!< @brief Double leg link length
			double LTS;                                    //!< @brief Leg tip scale
			rot_conv::Vec3 hipPos[NUM_LR];                 //!< @brief Position of the respective hip point relative to the corresponding leg cartesian origin
			rot_conv::Vec3 hipOffset[NUM_LR];              //!< @brief Offset from the respective hip point to the corresponding hip PR point (rotates with hip yaw)
			rot_conv::Vec3 footOffset[NUM_LR];             //!< @brief Offset from the respective ankle point to the corresponding leg tip point (foot plate geometric center)
			double LA;                                     //!< @brief Alias for arm link length
			double LAdbl;                                  //!< @brief Double arm link length
			rot_conv::Vec3 shoulderPos;                    //!< @brief Position of the respective shoulder point relative to the corresponding arm cartesian origin

		private:
			// Configuration parameter callbacks
			void legCB()
			{
				LL = legLinkLength();
				LLdbl = 2.0*LL;
				LTS = LLdbl + footOffsetZ();
				hipPos[LEFT] << -hipOffsetX(), LS_LEFT * -hipOffsetY(), LLdbl;
				hipPos[RIGHT] << -hipOffsetX(), LS_RIGHT * -hipOffsetY(), LLdbl;
				hipOffset[LEFT] << hipOffsetX(), LS_LEFT * hipOffsetY(), 0.0;
				hipOffset[RIGHT] << hipOffsetX(), LS_RIGHT * hipOffsetY(), 0.0;
				footOffset[LEFT] << footOffsetX(), LS_LEFT * footOffsetY(), -footOffsetZ();
				footOffset[RIGHT] << footOffsetX(), LS_RIGHT * footOffsetY(), -footOffsetZ();
			}
			void armCB()
			{
				LA = armLinkLength();
				LAdbl = 2.0*LA;
				shoulderPos << 0.0, 0.0, LAdbl;
			}
		};

		/**
		* @class SerialKinematics
		* 
		* @brief Serial robot kinematics class.
		**/
		class SerialKinematics : public RobotKinematics
		{
		public:
			// Typedefs - Joint types
			typedef serial_pose_classes::JointPose JointPose;
			typedef serial_pose_classes::JointLegPose JointLegPose;
			typedef serial_pose_classes::JointArmPose JointArmPose;
			typedef serial_pose_classes::JointHeadPose JointHeadPose;
			typedef serial_pose_classes::JointLegPoseVel JointLegPoseVel;
			typedef serial_pose_classes::JointPose JointEffort;
			typedef serial_pose_classes::JointLegPose JointLegEffort;
			typedef serial_pose_classes::JointArmPose JointArmEffort;
			typedef serial_pose_classes::JointHeadPose JointHeadEffort;

			// Typedefs - Abstract types
			typedef serial_pose_classes::AbsPose AbsPose;
			typedef serial_pose_classes::AbsLegPose AbsLegPose;
			typedef serial_pose_classes::AbsArmPose AbsArmPose;
			typedef serial_pose_classes::AbsHeadPose AbsHeadPose;
			typedef serial_pose_classes::AbsLegPoseVel AbsLegPoseVel;

			// Typedefs - Inverse types
			typedef serial_pose_classes::InvPose InvPose;
			typedef serial_pose_classes::InvLegPose InvLegPose;
			typedef serial_pose_classes::InvArmPose InvArmPose;
			typedef serial_pose_classes::InvHeadPose InvHeadPose;
			typedef serial_pose_classes::InvLegPoseVel InvLegPoseVel;

			// Typedefs - Tip types
			typedef serial_pose_classes::TipPose TipPose;
			typedef serial_pose_classes::LegTipPose LegTipPose;
			typedef serial_pose_classes::ArmTipPose ArmTipPose;
			typedef serial_pose_classes::HeadTipPose HeadTipPose;
			typedef serial_pose_classes::LegTipPoseVel LegTipPoseVel;

			// Typedefs - Extra velocity types
			typedef serial_pose_classes::LegVelVec LegVelVec;
			typedef serial_pose_classes::LegJacobian LegJacobian;
			typedef serial_pose_classes::LegJacobianSolver LegJacobianSolver;

			// Constructor/destructor
			SerialKinematics() : sconfig(SerialConfig::getInstance()) {}
			virtual ~SerialKinematics() = default;

			// Configuration parameters
			const SerialConfig& sconfig;

			// Robot scale functions
			virtual double legScaleInv() const override { return sconfig.LLdbl; }
			virtual double legScaleTip() const override { return sconfig.LTS; }
			virtual double armScaleInv() const override { return sconfig.LAdbl; }
			virtual double armScaleTip() const override { return sconfig.LAdbl; }

			// Cartesian origin functions
			virtual rot_conv::Vec3 originLeg(LimbIndex limbIndex) const override { return (limbIndex == RIGHT ? -sconfig.hipPos[RIGHT] : -sconfig.hipPos[LEFT]); }
			virtual rot_conv::Vec3 originArm(LimbIndex limbIndex) const override { return -sconfig.shoulderPos; }
			virtual rot_conv::Vec3 originHead(LimbIndex limbIndex) const override { return rot_conv::Vec3::Zero(); }
			virtual rot_conv::Vec3 hipPoint(LimbIndex limbIndex) const override { return (limbIndex == RIGHT ? sconfig.hipPos[RIGHT] : sconfig.hipPos[LEFT]); }
			virtual rot_conv::Vec3 shoulderPoint(LimbIndex limbIndex) const override { return sconfig.shoulderPos; }
			virtual rot_conv::Vec3 neckPoint(LimbIndex limbIndex) const override { return rot_conv::Vec3::Zero(); }

			//
			// Leg pose conversions
			//

			// Conversion: Joint --> Abstract
			void AbsFromJoint(const JointLegPose& JLP, AbsLegPose& ALP) const;
			inline AbsLegPose AbsFromJoint(const JointLegPose& JLP) const { AbsLegPose ALP(JLP.limbIndex); AbsFromJoint(JLP, ALP); return ALP; }

			// Conversion: Joint --> Inverse
			void InvFromJoint(const JointLegPose& JLP, InvLegPose& ILP) const;
			inline InvLegPose InvFromJoint(const JointLegPose& JLP) const { InvLegPose ILP(JLP.limbIndex); InvFromJoint(JLP, ILP); return ILP; }

			// Conversion: Joint --> Tip
			inline void TipFromJoint(const JointLegPose& JLP, LegTipPose& LTP) const { InvLegPose ILP(JLP.limbIndex); InvFromJoint(JLP, ILP); TipFromInv(ILP, LTP); } // Note: This internally uses InvFromJoint() and TipFromInv()
			inline LegTipPose TipFromJoint(const JointLegPose& JLP) const { InvLegPose ILP(JLP.limbIndex); InvFromJoint(JLP, ILP); return TipFromInv(ILP); } // Note: This internally uses InvFromJoint() and TipFromInv()

			// Conversion: Abstract --> Joint
			void JointFromAbs(const AbsLegPose& ALP, JointLegPose& JLP) const;
			inline JointLegPose JointFromAbs(const AbsLegPose& ALP) const { JointLegPose JLP(ALP.limbIndex); JointFromAbs(ALP, JLP); return JLP; }

			// Conversion: Abstract --> Inverse
			void InvFromAbs(const AbsLegPose& ALP, InvLegPose& ILP) const;
			inline InvLegPose InvFromAbs(const AbsLegPose& ALP) const { InvLegPose ILP(ALP.limbIndex); InvFromAbs(ALP, ILP); return ILP; }

			// Conversion: Abstract --> Tip
			inline void TipFromAbs(const AbsLegPose& ALP, LegTipPose& LTP) const { InvLegPose ILP(ALP.limbIndex); InvFromAbs(ALP, ILP); TipFromInv(ILP, LTP); } // Note: This internally uses InvFromAbs() and TipFromInv()
			inline LegTipPose TipFromAbs(const AbsLegPose& ALP) const { InvLegPose ILP(ALP.limbIndex); InvFromAbs(ALP, ILP); return TipFromInv(ILP); } // Note: This internally uses InvFromAbs() and TipFromInv()

			// Conversion: Inverse --> Joint
			inline bool JointFromInv(const InvLegPose& ILP, JointLegPose& JLP) const { AbsLegPose ALP(ILP.limbIndex); return JointAbsFromInv(ILP, JLP, ALP); } // Note: This internally uses JointAbsFromInv()
			inline JointLegPose JointFromInv(const InvLegPose& ILP) const { JointLegPose JLP(ILP.limbIndex); JointFromInv(ILP, JLP); return JLP; } // Note: This internally uses JointAbsFromInv()

			// Conversion: Inverse --> Abstract
			inline bool AbsFromInv(const InvLegPose& ILP, AbsLegPose& ALP) const { JointLegPose JLP(ILP.limbIndex); return JointAbsFromInv(ILP, JLP, ALP); } // Note: This internally uses JointAbsFromInv()
			inline AbsLegPose AbsFromInv(const InvLegPose& ILP) const { AbsLegPose ALP(ILP.limbIndex); AbsFromInv(ILP, ALP); return ALP; } // Note: This internally uses JointAbsFromInv()

			// Conversion: Inverse --> Joint and Abstract
			bool JointAbsFromInv(const InvLegPose& ILP, JointLegPose& JLP, AbsLegPose& ALP) const;

			// Conversion: Inverse --> Tip
			void TipFromInv(const InvLegPose& ILP, LegTipPose& LTP) const;
			inline LegTipPose TipFromInv(const InvLegPose& ILP) const { LegTipPose LTP(ILP.limbIndex); TipFromInv(ILP, LTP); return LTP; }

			// Conversion: Tip --> Joint
			inline void JointFromTip(const LegTipPose& LTP, JointLegPose& JLP) const { InvLegPose ILP(LTP.limbIndex); InvFromTip(LTP, ILP); JointFromInv(ILP, JLP); } // Note: This internally uses InvFromTip() and JointFromInv()
			inline JointLegPose JointFromTip(const LegTipPose& LTP) const { InvLegPose ILP(LTP.limbIndex); InvFromTip(LTP, ILP); return JointFromInv(ILP); } // Note: This internally uses InvFromTip() and JointFromInv()

			// Conversion: Tip --> Abstract
			inline void AbsFromTip(const LegTipPose& LTP, AbsLegPose& ALP) const { InvLegPose ILP(LTP.limbIndex); InvFromTip(LTP, ILP); AbsFromInv(ILP, ALP); } // Note: This internally uses InvFromTip() and AbsFromInv()
			inline AbsLegPose AbsFromTip(const LegTipPose& LTP) const { InvLegPose ILP(LTP.limbIndex); InvFromTip(LTP, ILP); return AbsFromInv(ILP); } // Note: This internally uses InvFromTip() and AbsFromInv()

			// Conversion: Tip --> Inverse
			void InvFromTip(const LegTipPose& LTP, InvLegPose& ILP) const;
			inline InvLegPose InvFromTip(const LegTipPose& LTP) const { InvLegPose ILP(LTP.limbIndex); InvFromTip(LTP, ILP); return ILP; }

			//
			// Extra leg pose conversions
			//

			// Conversion: Inverse --> Hip yaw
			void HipYawFromInv(const InvLegPose& ILP, double& hipYaw, double& hipYawAlt) const;
			inline void HipYawFromInv(const InvLegPose& ILP, double& hipYaw) const { double hipYawAlt; HipYawFromInv(ILP, hipYaw, hipYawAlt); }
			inline double HipYawFromInv(const InvLegPose& ILP) const { double hipYaw, hipYawAlt; HipYawFromInv(ILP, hipYaw, hipYawAlt); return hipYaw; }

			//
			// Arm pose conversions
			//

			// Conversion: Joint --> Abstract
			void AbsFromJoint(const JointArmPose& JAP, AbsArmPose& AAP) const;
			inline AbsArmPose AbsFromJoint(const JointArmPose& JAP) const { AbsArmPose AAP(JAP.limbIndex); AbsFromJoint(JAP, AAP); return AAP; }

			// Conversion: Joint --> Inverse
			void InvFromJoint(const JointArmPose& JAP, InvArmPose& IAP, rot_conv::Vec3& elbowPos) const;
			inline void InvFromJoint(const JointArmPose& JAP, InvArmPose& IAP) const { rot_conv::Vec3 elbowPos; InvFromJoint(JAP, IAP, elbowPos); }
			inline InvArmPose InvFromJoint(const JointArmPose& JAP) const { InvArmPose IAP(JAP.limbIndex); InvFromJoint(JAP, IAP); return IAP; }

			// Conversion: Joint --> Tip
			void TipFromJoint(const JointArmPose& JAP, ArmTipPose& ATP, rot_conv::Vec3& elbowPos) const;
			inline void TipFromJoint(const JointArmPose& JAP, ArmTipPose& ATP) const { rot_conv::Vec3 elbowPos; TipFromJoint(JAP, ATP, elbowPos); }
			inline ArmTipPose TipFromJoint(const JointArmPose& JAP) const { ArmTipPose ATP(JAP.limbIndex); TipFromJoint(JAP, ATP); return ATP; }

			// Conversion: Abstract --> Joint
			void JointFromAbs(const AbsArmPose& AAP, JointArmPose& JAP) const;
			inline JointArmPose JointFromAbs(const AbsArmPose& AAP) const { JointArmPose JAP(AAP.limbIndex); JointFromAbs(AAP, JAP); return JAP; }

			// Conversion: Abstract --> Inverse
			void InvFromAbs(const AbsArmPose& AAP, InvArmPose& IAP) const;
			inline InvArmPose InvFromAbs(const AbsArmPose& AAP) const { InvArmPose IAP(AAP.limbIndex); InvFromAbs(AAP, IAP); return IAP; }

			// Conversion: Abstract --> Tip
			inline void TipFromAbs(const AbsArmPose& AAP, ArmTipPose& ATP) const { JointArmPose JAP(AAP.limbIndex); JointFromAbs(AAP, JAP); TipFromJoint(JAP, ATP); } // Note: This internally uses JointFromAbs() and TipFromJoint()
			inline ArmTipPose TipFromAbs(const AbsArmPose& AAP) const { JointArmPose JAP(AAP.limbIndex); JointFromAbs(AAP, JAP); return TipFromJoint(JAP); } // Note: This internally uses JointFromAbs() and TipFromJoint()

			// Conversion: Inverse --> Joint
			void JointFromInv(const InvArmPose& IAP, JointArmPose& JAP) const;
			inline JointArmPose JointFromInv(const InvArmPose& IAP) const { JointArmPose JAP(IAP.limbIndex); JointFromInv(IAP, JAP); return JAP; }

			// Conversion: Inverse --> Abstract
			inline void AbsFromInv(const InvArmPose& IAP, AbsArmPose& AAP) const { JointArmPose JAP(IAP.limbIndex); JointAbsFromInv(IAP, JAP, AAP); } // Note: This internally uses JointAbsFromInv()
			inline AbsArmPose AbsFromInv(const InvArmPose& IAP) const { AbsArmPose AAP(IAP.limbIndex); AbsFromInv(IAP, AAP); return AAP; } // Note: This internally uses JointAbsFromInv()

			// Conversion: Inverse --> Joint and Abstract
			inline void JointAbsFromInv(const InvArmPose& IAP, JointArmPose& JAP, AbsArmPose& AAP) const { JointFromInv(IAP, JAP); AbsFromJoint(JAP, AAP); }

			// Conversion: Inverse --> Tip
			inline void TipFromInv(const InvArmPose& IAP, ArmTipPose& ATP) const { JointArmPose JAP(IAP.limbIndex); JointFromInv(IAP, JAP); TipFromJoint(JAP, ATP); } // Note: This internally uses JointFromInv() and TipFromJoint()
			inline ArmTipPose TipFromInv(const InvArmPose& IAP) const { JointArmPose JAP(IAP.limbIndex); JointFromInv(IAP, JAP); return TipFromJoint(JAP); } // Note: This internally uses JointFromInv() and TipFromJoint()

			// Conversion: Tip --> Joint
			inline void JointFromTip(const ArmTipPose& ATP, JointArmPose& JAP) const { InvArmPose IAP(ATP.limbIndex); InvFromTip(ATP, IAP); JointFromInv(IAP, JAP); } // Note: This internally uses InvFromTip() and JointFromInv()
			inline JointArmPose JointFromTip(const ArmTipPose& ATP) const { InvArmPose IAP(ATP.limbIndex); InvFromTip(ATP, IAP); return JointFromInv(IAP); } // Note: This internally uses InvFromTip() and JointFromInv()

			// Conversion: Tip --> Abstract
			inline void AbsFromTip(const ArmTipPose& ATP, AbsArmPose& AAP) const { InvArmPose IAP(ATP.limbIndex); InvFromTip(ATP, IAP); AbsFromInv(IAP, AAP); } // Note: This internally uses InvFromTip() and AbsFromInv()
			inline AbsArmPose AbsFromTip(const ArmTipPose& ATP) const { InvArmPose IAP(ATP.limbIndex); InvFromTip(ATP, IAP); return AbsFromInv(IAP); } // Note: This internally uses InvFromTip() and AbsFromInv()

			// Conversion: Tip --> Inverse
			void InvFromTip(const ArmTipPose& ATP, InvArmPose& IAP) const;
			inline InvArmPose InvFromTip(const ArmTipPose& ATP) const { InvArmPose IAP(ATP.limbIndex); InvFromTip(ATP, IAP); return IAP; }

			//
			// Extra arm pose conversions
			//

			// Conversion: Joint --> CoM
			void CoMFromJoint(const JointArmPose& JAP, rot_conv::Vec3& CoM) const;
			inline rot_conv::Vec3 CoMFromJoint(const JointArmPose& JAP) const { rot_conv::Vec3 CoM; CoMFromJoint(JAP, CoM); return CoM; }

			// Conversion: Abstract --> CoM
			inline void CoMFromAbs(const AbsArmPose& AAP, rot_conv::Vec3& CoM) const { JointArmPose JAP(AAP.limbIndex); JointFromAbs(AAP, JAP); CoMFromJoint(JAP, CoM); } // Note: This internally uses JointFromAbs() and CoMFromJoint()
			inline rot_conv::Vec3 CoMFromAbs(const AbsArmPose& AAP) const { JointArmPose JAP(AAP.limbIndex); JointFromAbs(AAP, JAP); return CoMFromJoint(JAP); } // Note: This internally uses JointFromAbs() and CoMFromJoint()

			// Conversion: CoMRay --> Joint
			void JointFromCoMRay(const rot_conv::Vec3& CoMRay, const JointArmPose& JAPRef, double shoulderRollMax, double shoulderRollBuf, JointArmPose& JAP) const;
			inline JointArmPose JointFromCoMRay(const rot_conv::Vec3& CoMRay, const JointArmPose& JAPRef, double shoulderRollMax, double shoulderRollBuf) const { JointArmPose JAP(JAPRef.limbIndex); JointFromCoMRay(CoMRay, JAPRef, shoulderRollMax, shoulderRollBuf, JAP); return JAP; }

			//
			// Head pose conversions
			//

		protected:
			// Head pose conversion helper functions
			void HeadRotFromAngles(double angleZ, double angleY, rot_conv::Quat& headRot) const;
			void DirnVecFromAngles(double angleZ, double angleY, rot_conv::Vec3& dirnVec) const;
			void AnglesFromDirnVec(const rot_conv::Vec3& dirnVec, double& angleZ, double& angleY) const;

		public:
			// Conversion: Joint --> Abstract
			void AbsFromJoint(const JointHeadPose& JHP, AbsHeadPose& AHP) const;
			inline AbsHeadPose AbsFromJoint(const JointHeadPose& JHP) const { AbsHeadPose AHP; AbsFromJoint(JHP, AHP); return AHP; }

			// Conversion: Joint --> Inverse
			void InvFromJoint(const JointHeadPose& JHP, InvHeadPose& IHP) const;
			inline InvHeadPose InvFromJoint(const JointHeadPose& JHP) const { InvHeadPose IHP; InvFromJoint(JHP, IHP); return IHP; }

			// Conversion: Joint --> Tip
			void TipFromJoint(const JointHeadPose& JHP, HeadTipPose& HTP) const;
			inline HeadTipPose TipFromJoint(const JointHeadPose& JHP) const { HeadTipPose HTP; TipFromJoint(JHP, HTP); return HTP; }

			// Conversion: Abstract --> Joint
			void JointFromAbs(const AbsHeadPose& AHP, JointHeadPose& JHP) const;
			inline JointHeadPose JointFromAbs(const AbsHeadPose& AHP) const { JointHeadPose JHP; JointFromAbs(AHP, JHP); return JHP; }

			// Conversion: Abstract --> Inverse
			void InvFromAbs(const AbsHeadPose& AHP, InvHeadPose& IHP) const;
			inline InvHeadPose InvFromAbs(const AbsHeadPose& AHP) const { InvHeadPose IHP; InvFromAbs(AHP, IHP); return IHP; }

			// Conversion: Abstract --> Tip
			void TipFromAbs(const AbsHeadPose& AHP, HeadTipPose& HTP) const;
			inline HeadTipPose TipFromAbs(const AbsHeadPose& AHP) const { HeadTipPose HTP; TipFromAbs(AHP, HTP); return HTP; }

			// Conversion: Inverse --> Joint
			void JointFromInv(const InvHeadPose& IHP, JointHeadPose& JHP) const;
			inline JointHeadPose JointFromInv(const InvHeadPose& IHP) const { JointHeadPose JHP; JointFromInv(IHP, JHP); return JHP; }

			// Conversion: Inverse --> Abstract
			void AbsFromInv(const InvHeadPose& IHP, AbsHeadPose& AHP) const;
			inline AbsHeadPose AbsFromInv(const InvHeadPose& IHP) const { AbsHeadPose AHP; AbsFromInv(IHP, AHP); return AHP; }

			// Conversion: Inverse --> Joint and Abstract
			void JointAbsFromInv(const InvHeadPose& IHP, JointHeadPose& JHP, AbsHeadPose& AHP) const;

			// Conversion: Inverse --> Tip
			void TipFromInv(const InvHeadPose& IHP, HeadTipPose& HTP) const;
			inline HeadTipPose TipFromInv(const InvHeadPose& IHP) const { HeadTipPose HTP; TipFromInv(IHP, HTP); return HTP; }

			// Conversion: Tip --> Joint
			void JointFromTip(const HeadTipPose& HTP, JointHeadPose& JHP) const;
			inline JointHeadPose JointFromTip(const HeadTipPose& HTP) const { JointHeadPose JHP; JointFromTip(HTP, JHP); return JHP; }

			// Conversion: Tip --> Abstract
			void AbsFromTip(const HeadTipPose& HTP, AbsHeadPose& AHP) const;
			inline AbsHeadPose AbsFromTip(const HeadTipPose& HTP) const { AbsHeadPose AHP; AbsFromTip(HTP, AHP); return AHP; }

			// Conversion: Tip --> Inverse
			void InvFromTip(const HeadTipPose& HTP, InvHeadPose& IHP) const;
			inline InvHeadPose InvFromTip(const HeadTipPose& HTP) const { InvHeadPose IHP; InvFromTip(HTP, IHP); return IHP; }

			//
			// Robot pose conversions
			//

			// Conversion: Joint --> Abstract
			void AbsFromJoint(const JointPose& JP, AbsPose& AP) const;
			inline AbsPose AbsFromJoint(const JointPose& JP) const { AbsPose AP; AbsFromJoint(JP, AP); return AP; }

			// Conversion: Joint --> Inverse
			void InvFromJoint(const JointPose& JP, InvPose& IP) const;
			inline InvPose InvFromJoint(const JointPose& JP) const { InvPose IP; InvFromJoint(JP, IP); return IP; }

			// Conversion: Joint --> Tip
			void TipFromJoint(const JointPose& JP, TipPose& TP) const;
			inline TipPose TipFromJoint(const JointPose& JP) const { TipPose TP; TipFromJoint(JP, TP); return TP; }

			// Conversion: Abstract --> Joint
			void JointFromAbs(const AbsPose& AP, JointPose& JP) const;
			inline JointPose JointFromAbs(const AbsPose& AP) const { JointPose JP; JointFromAbs(AP, JP); return JP; }

			// Conversion: Abstract --> Inverse
			void InvFromAbs(const AbsPose& AP, InvPose& IP) const;
			inline InvPose InvFromAbs(const AbsPose& AP) const { InvPose IP; InvFromAbs(AP, IP); return IP; }

			// Conversion: Abstract --> Tip
			void TipFromAbs(const AbsPose& AP, TipPose& TP) const;
			inline TipPose TipFromAbs(const AbsPose& AP) const { TipPose TP; TipFromAbs(AP, TP); return TP; }

			// Conversion: Inverse --> Joint
			void JointFromInv(const InvPose& IP, JointPose& JP) const;
			inline JointPose JointFromInv(const InvPose& IP) const { JointPose JP; JointFromInv(IP, JP); return JP; }

			// Conversion: Inverse --> Abstract
			void AbsFromInv(const InvPose& IP, AbsPose& AP) const;
			inline AbsPose AbsFromInv(const InvPose& IP) const { AbsPose AP; AbsFromInv(IP, AP); return AP; }

			// Conversion: Inverse --> Joint and Abstract
			void JointAbsFromInv(const InvPose& IP, JointPose& JP, AbsPose& AP) const;

			// Conversion: Inverse --> Tip
			void TipFromInv(const InvPose& IP, TipPose& TP) const;
			inline TipPose TipFromInv(const InvPose& IP) const { TipPose TP; TipFromInv(IP, TP); return TP; }

			// Conversion: Tip --> Joint
			void JointFromTip(const TipPose& TP, JointPose& JP) const;
			inline JointPose JointFromTip(const TipPose& TP) const { JointPose JP; JointFromTip(TP, JP); return JP; }

			// Conversion: Tip --> Abstract
			void AbsFromTip(const TipPose& TP, AbsPose& AP) const;
			inline AbsPose AbsFromTip(const TipPose& TP) const { AbsPose AP; AbsFromTip(TP, AP); return AP; }

			// Conversion: Tip --> Inverse
			void InvFromTip(const TipPose& TP, InvPose& IP) const;
			inline InvPose InvFromTip(const TipPose& TP) const { InvPose IP; InvFromTip(TP, IP); return IP; }

			//
			// Leg pose velocity conversions
			//

			// Conversion: Joint --> Abstract
		protected:
			void AbsFromJointJacobHelper(double A, LegJacobian& LJ) const;
			inline double AbsFromJointVelHelper(const JointLegPose& JLP) const { return 0.5*sin(0.5*rc_utils::picut(JLP.kneePitch)); }
			inline double AbsFromJointVelHelper(const AbsLegPose& ALP) const { return 0.5*sqrt(ALP.retraction*(2.0 - ALP.retraction)); }
			void AbsFromJointVelHelper(const JointLegPoseVel& JLPV, double A, AbsLegPoseVel& ALPV) const;
		public:
			inline void AbsFromJointJacob(const JointLegPose& JLP, LegJacobian& LJ) const { AbsFromJointJacobHelper(AbsFromJointVelHelper(JLP), LJ); }
			inline void AbsFromJointJacob(const AbsLegPose& ALP, LegJacobian& LJ) const { AbsFromJointJacobHelper(AbsFromJointVelHelper(ALP), LJ); }
			inline LegJacobian AbsFromJointJacob(const JointLegPose& JLP) const { LegJacobian LJ; AbsFromJointJacobHelper(AbsFromJointVelHelper(JLP), LJ); return LJ; }
			inline LegJacobian AbsFromJointJacob(const AbsLegPose& ALP) const { LegJacobian LJ; AbsFromJointJacobHelper(AbsFromJointVelHelper(ALP), LJ); return LJ; }
			inline void AbsFromJointVel(const JointLegPoseVel& JLPV, const LegJacobian& LJ, AbsLegPoseVel& ALPV) const { serial_pose_classes::AbsVelFromVec(LJ * serial_pose_classes::VecFromJointVel(JLPV), ALPV); }
			inline AbsLegPoseVel AbsFromJointVel(const JointLegPoseVel& JLPV, const LegJacobian& LJ) const { return serial_pose_classes::AbsVelFromVec(LJ * serial_pose_classes::VecFromJointVel(JLPV), JLPV.limbIndex); }
			inline void AbsFromJointVel(const JointLegPoseVel& JLPV, const JointLegPose& JLP, AbsLegPoseVel& ALPV) const { AbsFromJointVelHelper(JLPV, AbsFromJointVelHelper(JLP), ALPV); }
			inline void AbsFromJointVel(const JointLegPoseVel& JLPV, const AbsLegPose& ALP, AbsLegPoseVel& ALPV) const { AbsFromJointVelHelper(JLPV, AbsFromJointVelHelper(ALP), ALPV); }
			inline AbsLegPoseVel AbsFromJointVel(const JointLegPoseVel& JLPV, const JointLegPose& JLP) const { AbsLegPoseVel ALPV(JLPV.limbIndex); AbsFromJointVelHelper(JLPV, AbsFromJointVelHelper(JLP), ALPV); return ALPV; }
			inline AbsLegPoseVel AbsFromJointVel(const JointLegPoseVel& JLPV, const AbsLegPose& ALP) const { AbsLegPoseVel ALPV(JLPV.limbIndex); AbsFromJointVelHelper(JLPV, AbsFromJointVelHelper(ALP), ALPV); return ALPV; }

			// Conversion: Joint --> Inverse
		protected:
			void InvFromJointJacobHelper(const JointLegPose& JLP, LegJacobian& LJ, rot_conv::Vec3& PA, rot_conv::Rotmat& RF) const;
		public:
			inline void InvFromJointJacob(const JointLegPose& JLP, LegJacobian& LJ) const { rot_conv::Vec3 PA; rot_conv::Rotmat RF; InvFromJointJacobHelper(JLP, LJ, PA, RF); }
			inline void InvFromJointJacob(const JointLegPose& JLP, LegJacobian& LJ, InvLegPose& ILP) const { rot_conv::Rotmat RF; ILP.LegPose::operator=(JLP); InvFromJointJacobHelper(JLP, LJ, ILP.anklePos, RF); rot_conv::QuatFromRotmat(RF, ILP.footRot); }
			inline LegJacobian InvFromJointJacob(const JointLegPose& JLP) const { LegJacobian LJ; InvFromJointJacob(JLP, LJ); return LJ; }
			inline void InvFromJointVel(const JointLegPoseVel& JLPV, const LegJacobian& LJ, InvLegPoseVel& ILPV) const { serial_pose_classes::InvVelFromVec(LJ * serial_pose_classes::VecFromJointVel(JLPV), ILPV); }
			inline InvLegPoseVel InvFromJointVel(const JointLegPoseVel& JLPV, const LegJacobian& LJ) const { return serial_pose_classes::InvVelFromVec(LJ * serial_pose_classes::VecFromJointVel(JLPV), JLPV.limbIndex); }
			inline void InvFromJointVel(const JointLegPoseVel& JLPV, const JointLegPose& JLP, InvLegPoseVel& ILPV) const { InvFromJointVel(JLPV, InvFromJointJacob(JLP), ILPV); }
			inline void InvFromJointVel(const JointLegPoseVel& JLPV, const JointLegPose& JLP, InvLegPoseVel& ILPV, InvLegPose& ILP) const { LegJacobian LJ; InvFromJointJacob(JLP, LJ, ILP); InvFromJointVel(JLPV, LJ, ILPV); }
			inline void InvFromJointVel(const JointLegPoseVel& JLPV, const JointLegPose& JLP, InvLegPoseVel& ILPV, LegJacobian& LJ) const { InvFromJointJacob(JLP, LJ); InvFromJointVel(JLPV, LJ, ILPV); }
			inline void InvFromJointVel(const JointLegPoseVel& JLPV, const JointLegPose& JLP, InvLegPoseVel& ILPV, LegJacobian& LJ, InvLegPose& ILP) const { InvFromJointJacob(JLP, LJ, ILP); InvFromJointVel(JLPV, LJ, ILPV); }
			inline InvLegPoseVel InvFromJointVel(const JointLegPoseVel& JLPV, const JointLegPose& JLP) const { InvLegPoseVel ILPV(JLPV.limbIndex); InvFromJointVel(JLPV, JLP, ILPV); return ILPV; }

			// Conversion: Joint --> Tip
		protected:
			void TipFromJointJacobHelper(const JointLegPose& JLP, LegJacobian& LJ, rot_conv::Vec3& PF, rot_conv::Rotmat& RF) const;
		public:
			inline void TipFromJointJacob(const JointLegPose& JLP, LegJacobian& LJ) const { rot_conv::Vec3 PF; rot_conv::Rotmat RF; TipFromJointJacobHelper(JLP, LJ, PF, RF); }
			inline void TipFromJointJacob(const JointLegPose& JLP, LegJacobian& LJ, LegTipPose& LTP) const { rot_conv::Rotmat RF; LTP.LegPose::operator=(JLP); TipFromJointJacobHelper(JLP, LJ, LTP.pos, RF); rot_conv::QuatFromRotmat(RF, LTP.rot); }
			inline LegJacobian TipFromJointJacob(const JointLegPose& JLP) const { LegJacobian LJ; TipFromJointJacob(JLP, LJ); return LJ; }
			inline void TipFromJointVel(const JointLegPoseVel& JLPV, const LegJacobian& LJ, LegTipPoseVel& LTPV) const { serial_pose_classes::TipVelFromVec(LJ * serial_pose_classes::VecFromJointVel(JLPV), LTPV); }
			inline LegTipPoseVel TipFromJointVel(const JointLegPoseVel& JLPV, const LegJacobian& LJ) const { return serial_pose_classes::TipVelFromVec(LJ * serial_pose_classes::VecFromJointVel(JLPV), JLPV.limbIndex); }
			inline void TipFromJointVel(const JointLegPoseVel& JLPV, const JointLegPose& JLP, LegTipPoseVel& LTPV) const { TipFromJointVel(JLPV, TipFromJointJacob(JLP), LTPV); }
			inline void TipFromJointVel(const JointLegPoseVel& JLPV, const JointLegPose& JLP, LegTipPoseVel& LTPV, LegTipPose& LTP) const { LegJacobian LJ; TipFromJointJacob(JLP, LJ, LTP); TipFromJointVel(JLPV, LJ, LTPV); }
			inline void TipFromJointVel(const JointLegPoseVel& JLPV, const JointLegPose& JLP, LegTipPoseVel& LTPV, LegJacobian& LJ) const { TipFromJointJacob(JLP, LJ); TipFromJointVel(JLPV, LJ, LTPV); }
			inline void TipFromJointVel(const JointLegPoseVel& JLPV, const JointLegPose& JLP, LegTipPoseVel& LTPV, LegJacobian& LJ, LegTipPose& LTP) const { TipFromJointJacob(JLP, LJ, LTP); TipFromJointVel(JLPV, LJ, LTPV); }
			inline LegTipPoseVel TipFromJointVel(const JointLegPoseVel& JLPV, const JointLegPose& JLP) const { LegTipPoseVel LTPV(JLPV.limbIndex); TipFromJointVel(JLPV, JLP, LTPV); return LTPV; }

			// Conversion: Abstract --> Joint
		protected:
			void JointFromAbsJacobHelper(double A, LegJacobian& LJ) const;
			inline double JointFromAbsVelHelper(const JointLegPose& JLP) const { return 1.0 / sin(0.5*rc_utils::picut(JLP.kneePitch)); }
			inline double JointFromAbsVelHelper(const AbsLegPose& ALP) const { return 1.0 / sqrt(ALP.retraction*(2.0 - ALP.retraction)); }
			void JointFromAbsVelHelper(const AbsLegPoseVel& ALPV, double A, JointLegPoseVel& JLPV) const;
		public:
			inline void JointFromAbsJacob(const JointLegPose& JLP, LegJacobian& LJ) const { JointFromAbsJacobHelper(JointFromAbsVelHelper(JLP), LJ); }
			inline void JointFromAbsJacob(const AbsLegPose& ALP, LegJacobian& LJ) const { JointFromAbsJacobHelper(JointFromAbsVelHelper(ALP), LJ); }
			inline LegJacobian JointFromAbsJacob(const JointLegPose& JLP) const { LegJacobian LJ; JointFromAbsJacobHelper(JointFromAbsVelHelper(JLP), LJ); return LJ; }
			inline LegJacobian JointFromAbsJacob(const AbsLegPose& ALP) const { LegJacobian LJ; JointFromAbsJacobHelper(JointFromAbsVelHelper(ALP), LJ); return LJ; }
			inline void JointFromAbsVel(const AbsLegPoseVel& ALPV, const LegJacobian& LJ, JointLegPoseVel& JLPV) const { serial_pose_classes::JointVelFromVec(LJ * serial_pose_classes::VecFromAbsVel(ALPV), JLPV); }
			inline JointLegPoseVel JointFromAbsVel(const AbsLegPoseVel& ALPV, const LegJacobian& LJ) const { return serial_pose_classes::JointVelFromVec(LJ * serial_pose_classes::VecFromAbsVel(ALPV), ALPV.limbIndex); }
			inline void JointFromAbsVel(const AbsLegPoseVel& ALPV, const JointLegPose& JLP, JointLegPoseVel& JLPV) const { JointFromAbsVelHelper(ALPV, JointFromAbsVelHelper(JLP), JLPV); }
			inline void JointFromAbsVel(const AbsLegPoseVel& ALPV, const AbsLegPose& ALP, JointLegPoseVel& JLPV) const { JointFromAbsVelHelper(ALPV, JointFromAbsVelHelper(ALP), JLPV); }
			inline JointLegPoseVel JointFromAbsVel(const AbsLegPoseVel& ALPV, const JointLegPose& JLP) const { JointLegPoseVel JLPV(ALPV.limbIndex); JointFromAbsVelHelper(ALPV, JointFromAbsVelHelper(JLP), JLPV); return JLPV; }
			inline JointLegPoseVel JointFromAbsVel(const AbsLegPoseVel& ALPV, const AbsLegPose& ALP) const { JointLegPoseVel JLPV(ALPV.limbIndex); JointFromAbsVelHelper(ALPV, JointFromAbsVelHelper(ALP), JLPV); return JLPV; }

			// Conversion: Abstract --> Inverse
		protected:
			void InvFromAbsJacobHelper(const AbsLegPose& ALP, LegJacobian& LJ, rot_conv::Vec3& PA, rot_conv::Rotmat& RF) const;
		public:
			inline void InvFromAbsJacob(const AbsLegPose& ALP, LegJacobian& LJ) const { rot_conv::Vec3 PA; rot_conv::Rotmat RF; InvFromAbsJacobHelper(ALP, LJ, PA, RF); }
			inline void InvFromAbsJacob(const AbsLegPose& ALP, LegJacobian& LJ, InvLegPose& ILP) const { rot_conv::Rotmat RF; ILP.LegPose::operator=(ALP); InvFromAbsJacobHelper(ALP, LJ, ILP.anklePos, RF); rot_conv::QuatFromRotmat(RF, ILP.footRot); }
			inline LegJacobian InvFromAbsJacob(const AbsLegPose& ALP) const { LegJacobian LJ; InvFromAbsJacob(ALP, LJ); return LJ; }
			inline void InvFromAbsVel(const AbsLegPoseVel& ALPV, const LegJacobian& LJ, InvLegPoseVel& ILPV) const { serial_pose_classes::InvVelFromVec(LJ * serial_pose_classes::VecFromAbsVel(ALPV), ILPV); }
			inline InvLegPoseVel InvFromAbsVel(const AbsLegPoseVel& ALPV, const LegJacobian& LJ) const { return serial_pose_classes::InvVelFromVec(LJ * serial_pose_classes::VecFromAbsVel(ALPV), ALPV.limbIndex); }
			inline void InvFromAbsVel(const AbsLegPoseVel& ALPV, const AbsLegPose& ALP, InvLegPoseVel& ILPV) const { InvFromAbsVel(ALPV, InvFromAbsJacob(ALP), ILPV); }
			inline void InvFromAbsVel(const AbsLegPoseVel& ALPV, const AbsLegPose& ALP, InvLegPoseVel& ILPV, InvLegPose& ILP) const { LegJacobian LJ; InvFromAbsJacob(ALP, LJ, ILP); InvFromAbsVel(ALPV, LJ, ILPV); }
			inline void InvFromAbsVel(const AbsLegPoseVel& ALPV, const AbsLegPose& ALP, InvLegPoseVel& ILPV, LegJacobian& LJ) const { InvFromAbsJacob(ALP, LJ); InvFromAbsVel(ALPV, LJ, ILPV); }
			inline void InvFromAbsVel(const AbsLegPoseVel& ALPV, const AbsLegPose& ALP, InvLegPoseVel& ILPV, LegJacobian& LJ, InvLegPose& ILP) const { InvFromAbsJacob(ALP, LJ, ILP); InvFromAbsVel(ALPV, LJ, ILPV); }
			inline InvLegPoseVel InvFromAbsVel(const AbsLegPoseVel& ALPV, const AbsLegPose& ALP) const { InvLegPoseVel ILPV(ALPV.limbIndex); InvFromAbsVel(ALPV, ALP, ILPV); return ILPV; }

			// Conversion: Abstract --> Tip
		protected:
			void TipFromAbsJacobHelper(const AbsLegPose& ALP, LegJacobian& LJ, rot_conv::Vec3& PF, rot_conv::Rotmat& RF) const;
		public:
			inline void TipFromAbsJacob(const AbsLegPose& ALP, LegJacobian& LJ) const { rot_conv::Vec3 PF; rot_conv::Rotmat RF; TipFromAbsJacobHelper(ALP, LJ, PF, RF); }
			inline void TipFromAbsJacob(const AbsLegPose& ALP, LegJacobian& LJ, LegTipPose& LTP) const { rot_conv::Rotmat RF; LTP.LegPose::operator=(ALP); TipFromAbsJacobHelper(ALP, LJ, LTP.pos, RF); rot_conv::QuatFromRotmat(RF, LTP.rot); }
			inline LegJacobian TipFromAbsJacob(const AbsLegPose& ALP) const { LegJacobian LJ; TipFromAbsJacob(ALP, LJ); return LJ; }
			inline void TipFromAbsVel(const AbsLegPoseVel& ALPV, const LegJacobian& LJ, LegTipPoseVel& LTPV) const { serial_pose_classes::TipVelFromVec(LJ * serial_pose_classes::VecFromAbsVel(ALPV), LTPV); }
			inline LegTipPoseVel TipFromAbsVel(const AbsLegPoseVel& ALPV, const LegJacobian& LJ) const { return serial_pose_classes::TipVelFromVec(LJ * serial_pose_classes::VecFromAbsVel(ALPV), ALPV.limbIndex); }
			inline void TipFromAbsVel(const AbsLegPoseVel& ALPV, const AbsLegPose& ALP, LegTipPoseVel& LTPV) const { TipFromAbsVel(ALPV, TipFromAbsJacob(ALP), LTPV); }
			inline void TipFromAbsVel(const AbsLegPoseVel& ALPV, const AbsLegPose& ALP, LegTipPoseVel& LTPV, LegTipPose& LTP) const { LegJacobian LJ; TipFromAbsJacob(ALP, LJ, LTP); TipFromAbsVel(ALPV, LJ, LTPV); }
			inline void TipFromAbsVel(const AbsLegPoseVel& ALPV, const AbsLegPose& ALP, LegTipPoseVel& LTPV, LegJacobian& LJ) const { TipFromAbsJacob(ALP, LJ); TipFromAbsVel(ALPV, LJ, LTPV); }
			inline void TipFromAbsVel(const AbsLegPoseVel& ALPV, const AbsLegPose& ALP, LegTipPoseVel& LTPV, LegJacobian& LJ, LegTipPose& LTP) const { TipFromAbsJacob(ALP, LJ, LTP); TipFromAbsVel(ALPV, LJ, LTPV); }
			inline LegTipPoseVel TipFromAbsVel(const AbsLegPoseVel& ALPV, const AbsLegPose& ALP) const { LegTipPoseVel LTPV(ALPV.limbIndex); TipFromAbsVel(ALPV, ALP, LTPV); return LTPV; }

			// Conversion: Inverse --> Joint
			inline void JointFromInvSolver(const JointLegPose& JLP, LegJacobianSolver& LJS) const { LegJacobian LJinv; InvFromJointJacob(JLP, LJinv); LJS.compute(LJinv); }
			inline void JointFromInvSolver(const JointLegPose& JLP, LegJacobianSolver& LJS, InvLegPose& ILP) const { LegJacobian LJinv; InvFromJointJacob(JLP, LJinv, ILP); LJS.compute(LJinv); }
			inline LegJacobianSolver JointFromInvSolver(const JointLegPose& JLP) const { LegJacobian LJinv; InvFromJointJacob(JLP, LJinv); return LegJacobianSolver(LJinv); }
			inline LegJacobian JointFromInvJacob(const JointLegPose& JLP) const { return serial_pose_classes::LegJacobFromSolver(JointFromInvSolver(JLP)); } // Note: It is numerically advised whenever possible to use JointFromInvSolver() instead of this!
			inline void JointFromInvVel(const InvLegPoseVel& ILPV, const LegJacobianSolver& LJS, JointLegPoseVel& JLPV) const { serial_pose_classes::JointVelFromVec(LJS.solve(serial_pose_classes::VecFromInvVel(ILPV)), JLPV); }
			inline JointLegPoseVel JointFromInvVel(const InvLegPoseVel& ILPV, const LegJacobianSolver& LJS) const { return serial_pose_classes::JointVelFromVec(LJS.solve(serial_pose_classes::VecFromInvVel(ILPV)), ILPV.limbIndex); }
			inline void JointFromInvVel(const InvLegPoseVel& ILPV, const JointLegPose& JLP, JointLegPoseVel& JLPV) const { JointFromInvVel(ILPV, JointFromInvSolver(JLP), JLPV); }
			inline void JointFromInvVel(const InvLegPoseVel& ILPV, const JointLegPose& JLP, JointLegPoseVel& JLPV, InvLegPose& ILP) const { LegJacobianSolver LJS; JointFromInvSolver(JLP, LJS, ILP); JointFromInvVel(ILPV, LJS, JLPV); }
			inline void JointFromInvVel(const InvLegPoseVel& ILPV, const JointLegPose& JLP, JointLegPoseVel& JLPV, LegJacobianSolver& LJS) const { JointFromInvSolver(JLP, LJS); JointFromInvVel(ILPV, LJS, JLPV); }
			inline void JointFromInvVel(const InvLegPoseVel& ILPV, const JointLegPose& JLP, JointLegPoseVel& JLPV, LegJacobianSolver& LJS, InvLegPose& ILP) const { JointFromInvSolver(JLP, LJS, ILP); JointFromInvVel(ILPV, LJS, JLPV); }
			inline JointLegPoseVel JointFromInvVel(const InvLegPoseVel& ILPV, const JointLegPose& JLP) const { JointLegPoseVel JLPV(ILPV.limbIndex); JointFromInvVel(ILPV, JLP, JLPV); return JLPV; }

			// Conversion: Inverse --> Abstract
			inline void AbsFromInvSolver(const AbsLegPose& ALP, LegJacobianSolver& LJS) const { LegJacobian LJinv; InvFromAbsJacob(ALP, LJinv); LJS.compute(LJinv); }
			inline void AbsFromInvSolver(const AbsLegPose& ALP, LegJacobianSolver& LJS, InvLegPose& ILP) const { LegJacobian LJinv; InvFromAbsJacob(ALP, LJinv, ILP); LJS.compute(LJinv); }
			inline LegJacobianSolver AbsFromInvSolver(const AbsLegPose& ALP) const { LegJacobian LJinv; InvFromAbsJacob(ALP, LJinv); return LegJacobianSolver(LJinv); }
			inline LegJacobian AbsFromInvJacob(const AbsLegPose& ALP) const { return serial_pose_classes::LegJacobFromSolver(AbsFromInvSolver(ALP)); } // Note: It is numerically advised whenever possible to use AbsFromInvSolver() instead of this!
			inline void AbsFromInvVel(const InvLegPoseVel& ILPV, const LegJacobianSolver& LJS, AbsLegPoseVel& ALPV) const { serial_pose_classes::AbsVelFromVec(LJS.solve(serial_pose_classes::VecFromInvVel(ILPV)), ALPV); }
			inline AbsLegPoseVel AbsFromInvVel(const InvLegPoseVel& ILPV, const LegJacobianSolver& LJS) const { return serial_pose_classes::AbsVelFromVec(LJS.solve(serial_pose_classes::VecFromInvVel(ILPV)), ILPV.limbIndex); }
			inline void AbsFromInvVel(const InvLegPoseVel& ILPV, const AbsLegPose& ALP, AbsLegPoseVel& ALPV) const { AbsFromInvVel(ILPV, AbsFromInvSolver(ALP), ALPV); }
			inline void AbsFromInvVel(const InvLegPoseVel& ILPV, const AbsLegPose& ALP, AbsLegPoseVel& ALPV, InvLegPose& ILP) const { LegJacobianSolver LJS; AbsFromInvSolver(ALP, LJS, ILP); AbsFromInvVel(ILPV, LJS, ALPV); }
			inline void AbsFromInvVel(const InvLegPoseVel& ILPV, const AbsLegPose& ALP, AbsLegPoseVel& ALPV, LegJacobianSolver& LJS) const { AbsFromInvSolver(ALP, LJS); AbsFromInvVel(ILPV, LJS, ALPV); }
			inline void AbsFromInvVel(const InvLegPoseVel& ILPV, const AbsLegPose& ALP, AbsLegPoseVel& ALPV, LegJacobianSolver& LJS, InvLegPose& ILP) const { AbsFromInvSolver(ALP, LJS, ILP); AbsFromInvVel(ILPV, LJS, ALPV); }
			inline AbsLegPoseVel AbsFromInvVel(const InvLegPoseVel& ILPV, const AbsLegPose& ALP) const { AbsLegPoseVel ALPV(ILPV.limbIndex); AbsFromInvVel(ILPV, ALP, ALPV); return ALPV; }

			// Conversion: Inverse --> Tip
			void TipFromInvJacob(const rot_conv::Quat& footRot, LimbIndex limbIndex, LegJacobian& LJ) const;
			inline void TipFromInvJacob(const InvLegPose& ILP, LegJacobian& LJ) const { TipFromInvJacob(ILP.footRot, ILP.limbIndex, LJ); }
			inline void TipFromInvJacob(const LegTipPose& LTP, LegJacobian& LJ) const { TipFromInvJacob(LTP.rot, LTP.limbIndex, LJ); }
			inline LegJacobian TipFromInvJacob(const rot_conv::Quat& footRot, LimbIndex limbIndex) const { LegJacobian LJ; TipFromInvJacob(footRot, limbIndex, LJ); return LJ; }
			inline LegJacobian TipFromInvJacob(const InvLegPose& ILP) const { LegJacobian LJ; TipFromInvJacob(ILP.footRot, ILP.limbIndex, LJ); return LJ; }
			inline LegJacobian TipFromInvJacob(const LegTipPose& LTP) const { LegJacobian LJ; TipFromInvJacob(LTP.rot, LTP.limbIndex, LJ); return LJ; }
			inline void TipFromInvVel(const InvLegPoseVel& ILPV, const LegJacobian& LJ, LegTipPoseVel& LTPV) const { serial_pose_classes::TipVelFromVec(LJ * serial_pose_classes::VecFromInvVel(ILPV), LTPV); }
			inline LegTipPoseVel TipFromInvVel(const InvLegPoseVel& ILPV, const LegJacobian& LJ) const { return serial_pose_classes::TipVelFromVec(LJ * serial_pose_classes::VecFromInvVel(ILPV), ILPV.limbIndex); }
			void TipFromInvVel(const InvLegPoseVel& ILPV, const rot_conv::Quat& footRot, LimbIndex limbIndex, LegTipPoseVel& LTPV) const;
			inline void TipFromInvVel(const InvLegPoseVel& ILPV, const rot_conv::Quat& footRot, LegTipPoseVel& LTPV) const { TipFromInvVel(ILPV, footRot, ILPV.limbIndex, LTPV); }
			inline void TipFromInvVel(const InvLegPoseVel& ILPV, const InvLegPose& ILP, LegTipPoseVel& LTPV) const { TipFromInvVel(ILPV, ILP.footRot, ILP.limbIndex, LTPV); }
			inline void TipFromInvVel(const InvLegPoseVel& ILPV, const LegTipPose& LTP, LegTipPoseVel& LTPV) const { TipFromInvVel(ILPV, LTP.rot, LTP.limbIndex, LTPV); }
			inline LegTipPoseVel TipFromInvVel(const InvLegPoseVel& ILPV, const rot_conv::Quat& footRot, LimbIndex limbIndex) const { LegTipPoseVel LTPV(ILPV.limbIndex); TipFromInvVel(ILPV, footRot, limbIndex, LTPV); return LTPV; }
			inline LegTipPoseVel TipFromInvVel(const InvLegPoseVel& ILPV, const rot_conv::Quat& footRot) const { LegTipPoseVel LTPV(ILPV.limbIndex); TipFromInvVel(ILPV, footRot, ILPV.limbIndex, LTPV); return LTPV; }
			inline LegTipPoseVel TipFromInvVel(const InvLegPoseVel& ILPV, const InvLegPose& ILP) const { LegTipPoseVel LTPV(ILPV.limbIndex); TipFromInvVel(ILPV, ILP.footRot, ILP.limbIndex, LTPV); return LTPV; }
			inline LegTipPoseVel TipFromInvVel(const InvLegPoseVel& ILPV, const LegTipPose& LTP) const { LegTipPoseVel LTPV(ILPV.limbIndex); TipFromInvVel(ILPV, LTP.rot, LTP.limbIndex, LTPV); return LTPV; }

			// Conversion: Tip --> Joint
			inline void JointFromTipSolver(const JointLegPose& JLP, LegJacobianSolver& LJS) const { LegJacobian LJinv; TipFromJointJacob(JLP, LJinv); LJS.compute(LJinv); }
			inline void JointFromTipSolver(const JointLegPose& JLP, LegJacobianSolver& LJS, LegTipPose& LTP) const { LegJacobian LJinv; TipFromJointJacob(JLP, LJinv, LTP); LJS.compute(LJinv); }
			inline LegJacobianSolver JointFromTipSolver(const JointLegPose& JLP) const { LegJacobian LJinv; TipFromJointJacob(JLP, LJinv); return LegJacobianSolver(LJinv); }
			inline LegJacobian JointFromTipJacob(const JointLegPose& JLP) const { return serial_pose_classes::LegJacobFromSolver(JointFromTipSolver(JLP)); } // Note: It is numerically advised whenever possible to use JointFromTipSolver() instead of this!
			inline void JointFromTipVel(const LegTipPoseVel& LTPV, const LegJacobianSolver& LJS, JointLegPoseVel& JLPV) const { serial_pose_classes::JointVelFromVec(LJS.solve(serial_pose_classes::VecFromTipVel(LTPV)), JLPV); }
			inline JointLegPoseVel JointFromTipVel(const LegTipPoseVel& LTPV, const LegJacobianSolver& LJS) const { return serial_pose_classes::JointVelFromVec(LJS.solve(serial_pose_classes::VecFromTipVel(LTPV)), LTPV.limbIndex); }
			inline void JointFromTipVel(const LegTipPoseVel& LTPV, const JointLegPose& JLP, JointLegPoseVel& JLPV) const { JointFromTipVel(LTPV, JointFromTipSolver(JLP), JLPV); }
			inline void JointFromTipVel(const LegTipPoseVel& LTPV, const JointLegPose& JLP, JointLegPoseVel& JLPV, LegTipPose& LTP) const { LegJacobianSolver LJS; JointFromTipSolver(JLP, LJS, LTP); JointFromTipVel(LTPV, LJS, JLPV); }
			inline void JointFromTipVel(const LegTipPoseVel& LTPV, const JointLegPose& JLP, JointLegPoseVel& JLPV, LegJacobianSolver& LJS) const { JointFromTipSolver(JLP, LJS); JointFromTipVel(LTPV, LJS, JLPV); }
			inline void JointFromTipVel(const LegTipPoseVel& LTPV, const JointLegPose& JLP, JointLegPoseVel& JLPV, LegJacobianSolver& LJS, LegTipPose& LTP) const { JointFromTipSolver(JLP, LJS, LTP); JointFromTipVel(LTPV, LJS, JLPV); }
			inline JointLegPoseVel JointFromTipVel(const LegTipPoseVel& LTPV, const JointLegPose& JLP) const { JointLegPoseVel JLPV(LTPV.limbIndex); JointFromTipVel(LTPV, JLP, JLPV); return JLPV; }

			// Conversion: Tip --> Abstract
			inline void AbsFromTipSolver(const AbsLegPose& ALP, LegJacobianSolver& LJS) const { LegJacobian LJinv; TipFromAbsJacob(ALP, LJinv); LJS.compute(LJinv); }
			inline void AbsFromTipSolver(const AbsLegPose& ALP, LegJacobianSolver& LJS, LegTipPose& LTP) const { LegJacobian LJinv; TipFromAbsJacob(ALP, LJinv, LTP); LJS.compute(LJinv); }
			inline LegJacobianSolver AbsFromTipSolver(const AbsLegPose& ALP) const { LegJacobian LJinv; TipFromAbsJacob(ALP, LJinv); return LegJacobianSolver(LJinv); }
			inline LegJacobian AbsFromTipJacob(const AbsLegPose& ALP) const { return serial_pose_classes::LegJacobFromSolver(AbsFromTipSolver(ALP)); } // Note: It is numerically advised whenever possible to use AbsFromTipSolver() instead of this!
			inline void AbsFromTipVel(const LegTipPoseVel& LTPV, const LegJacobianSolver& LJS, AbsLegPoseVel& ALPV) const { serial_pose_classes::AbsVelFromVec(LJS.solve(serial_pose_classes::VecFromTipVel(LTPV)), ALPV); }
			inline AbsLegPoseVel AbsFromTipVel(const LegTipPoseVel& LTPV, const LegJacobianSolver& LJS) const { return serial_pose_classes::AbsVelFromVec(LJS.solve(serial_pose_classes::VecFromTipVel(LTPV)), LTPV.limbIndex); }
			inline void AbsFromTipVel(const LegTipPoseVel& LTPV, const AbsLegPose& ALP, AbsLegPoseVel& ALPV) const { AbsFromTipVel(LTPV, AbsFromTipSolver(ALP), ALPV); }
			inline void AbsFromTipVel(const LegTipPoseVel& LTPV, const AbsLegPose& ALP, AbsLegPoseVel& ALPV, LegTipPose& LTP) const { LegJacobianSolver LJS; AbsFromTipSolver(ALP, LJS, LTP); AbsFromTipVel(LTPV, LJS, ALPV); }
			inline void AbsFromTipVel(const LegTipPoseVel& LTPV, const AbsLegPose& ALP, AbsLegPoseVel& ALPV, LegJacobianSolver& LJS) const { AbsFromTipSolver(ALP, LJS); AbsFromTipVel(LTPV, LJS, ALPV); }
			inline void AbsFromTipVel(const LegTipPoseVel& LTPV, const AbsLegPose& ALP, AbsLegPoseVel& ALPV, LegJacobianSolver& LJS, LegTipPose& LTP) const { AbsFromTipSolver(ALP, LJS, LTP); AbsFromTipVel(LTPV, LJS, ALPV); }
			inline AbsLegPoseVel AbsFromTipVel(const LegTipPoseVel& LTPV, const AbsLegPose& ALP) const { AbsLegPoseVel ALPV(LTPV.limbIndex); AbsFromTipVel(LTPV, ALP, ALPV); return ALPV; }

			// Conversion: Tip --> Inverse
			void InvFromTipJacob(const rot_conv::Quat& rot, LimbIndex limbIndex, LegJacobian& LJ) const;
			inline void InvFromTipJacob(const InvLegPose& ILP, LegJacobian& LJ) const { InvFromTipJacob(ILP.footRot, ILP.limbIndex, LJ); }
			inline void InvFromTipJacob(const LegTipPose& LTP, LegJacobian& LJ) const { InvFromTipJacob(LTP.rot, LTP.limbIndex, LJ); }
			inline LegJacobian InvFromTipJacob(const rot_conv::Quat& rot, LimbIndex limbIndex) const { LegJacobian LJ; InvFromTipJacob(rot, limbIndex, LJ); return LJ; }
			inline LegJacobian InvFromTipJacob(const InvLegPose& ILP) const { LegJacobian LJ; InvFromTipJacob(ILP.footRot, ILP.limbIndex, LJ); return LJ; }
			inline LegJacobian InvFromTipJacob(const LegTipPose& LTP) const { LegJacobian LJ; InvFromTipJacob(LTP.rot, LTP.limbIndex, LJ); return LJ; }
			inline void InvFromTipVel(const LegTipPoseVel& LTPV, const LegJacobian& LJ, InvLegPoseVel& ILPV) const { serial_pose_classes::InvVelFromVec(LJ * serial_pose_classes::VecFromTipVel(LTPV), ILPV); }
			inline InvLegPoseVel InvFromTipVel(const LegTipPoseVel& LTPV, const LegJacobian& LJ) const { return serial_pose_classes::InvVelFromVec(LJ * serial_pose_classes::VecFromTipVel(LTPV), LTPV.limbIndex); }
			void InvFromTipVel(const LegTipPoseVel& LTPV, const rot_conv::Quat& rot, LimbIndex limbIndex, InvLegPoseVel& ILPV) const;
			inline void InvFromTipVel(const LegTipPoseVel& LTPV, const rot_conv::Quat& rot, InvLegPoseVel& ILPV) const { InvFromTipVel(LTPV, rot, LTPV.limbIndex, ILPV); }
			inline void InvFromTipVel(const LegTipPoseVel& LTPV, const InvLegPose& ILP, InvLegPoseVel& ILPV) const { InvFromTipVel(LTPV, ILP.footRot, ILP.limbIndex, ILPV); }
			inline void InvFromTipVel(const LegTipPoseVel& LTPV, const LegTipPose& LTP, InvLegPoseVel& ILPV) const { InvFromTipVel(LTPV, LTP.rot, LTP.limbIndex, ILPV); }
			inline InvLegPoseVel InvFromTipVel(const LegTipPoseVel& LTPV, const rot_conv::Quat& rot, LimbIndex limbIndex) const { InvLegPoseVel ILPV(LTPV.limbIndex); InvFromTipVel(LTPV, rot, limbIndex, ILPV); return ILPV; }
			inline InvLegPoseVel InvFromTipVel(const LegTipPoseVel& LTPV, const rot_conv::Quat& rot) const { InvLegPoseVel ILPV(LTPV.limbIndex); InvFromTipVel(LTPV, rot, LTPV.limbIndex, ILPV); return ILPV; }
			inline InvLegPoseVel InvFromTipVel(const LegTipPoseVel& LTPV, const InvLegPose& ILP) const { InvLegPoseVel ILPV(LTPV.limbIndex); InvFromTipVel(LTPV, ILP.footRot, ILP.limbIndex, ILPV); return ILPV; }
			inline InvLegPoseVel InvFromTipVel(const LegTipPoseVel& LTPV, const LegTipPose& LTP) const { InvLegPoseVel ILPV(LTPV.limbIndex); InvFromTipVel(LTPV, LTP.rot, LTP.limbIndex, ILPV); return ILPV; }
		};
	}
}

#endif
// EOF