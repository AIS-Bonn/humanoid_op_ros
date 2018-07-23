// Humanoid kinematics - Generic robot kinematics
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

// Includes
#include <humanoid_kinematics/kinematics_common.h>
#include <humanoid_kinematics/pose_classes.h>
#include <config_server/parameter.h>
#include <type_traits>
#include <memory>
#include <vector>

// Humanoid kinematics namespace
namespace humanoid_kinematics
{
	/**
	* @class RobotConfig
	* 
	* @brief Robot kinematics configuration parameters class.
	**/
	class RobotConfig
	{
	private:
		// Constructor
		RobotConfig()
			: CONFIG_PARAM_PATH(ROOT_CONFIG_PARAM_PATH + "common/")
			, hipWidth        (CONFIG_PARAM_PATH + "hipWidth", 0.01, 0.005, 0.6, 0.2)
			, trunkLinkOffsetX(CONFIG_PARAM_PATH + "trunkLinkOffsetX", -0.3, 0.005, 0.3, 0.0)
			, trunkLinkOffsetY(CONFIG_PARAM_PATH + "trunkLinkOffsetY", -0.3, 0.005, 0.3, 0.0)
			, trunkLinkOffsetZ(CONFIG_PARAM_PATH + "trunkLinkOffsetZ", -0.3, 0.005, 0.7, 0.0)
		{
			// Configure callbacks
			hipWidth.setCallback(boost::bind(&RobotConfig::hipWidthCB, this), true);
			boost::function<void (const float&)> trunkLinkOffsetCBFunc = boost::bind(&RobotConfig::trunkLinkOffsetCB, this);
			trunkLinkOffsetX.setCallback(trunkLinkOffsetCBFunc);
			trunkLinkOffsetY.setCallback(trunkLinkOffsetCBFunc);
			trunkLinkOffsetZ.setCallback(trunkLinkOffsetCBFunc);
			trunkLinkOffsetCB();
		}

		// Ensure class remains a singleton
		RobotConfig(const RobotConfig&) = delete;
		RobotConfig& operator=(const RobotConfig&) = delete;

	public:
		// Get singleton instance of class
		static const RobotConfig& getInstance() { static thread_local RobotConfig rconfig; return rconfig; }

		// Constants
		const std::string CONFIG_PARAM_PATH;

		// Configuration parameters
		config_server::Parameter<float> hipWidth;         //!< @brief Length of the pure y-axis separation between the two hip points (length of the hip line)
		config_server::Parameter<float> trunkLinkOffsetX; //!< @brief Forward offset of the trunk link tf frame from the hip centre point
		config_server::Parameter<float> trunkLinkOffsetY; //!< @brief Leftward offset of the trunk link tf frame from the hip centre point
		config_server::Parameter<float> trunkLinkOffsetZ; //!< @brief Upward offset of the trunk link tf frame from the hip centre point

		// Derived configuration parameters
		double H;                                         //!< @brief Half of the hip width (i.e. the distance between the hip centre point and each hip point)
		rot_conv::Vec3 Hvec[NUM_LR];                      //!< @brief Vector from the hip centre point to the respective hip point
		rot_conv::Vec3 trunkLinkOffset;                   //!< @brief Vector offset from the hip centre point to the trunk link tf frame

	private:
		// Configuration parameter callbacks
		void hipWidthCB() { H = 0.5*hipWidth(); Hvec[LEFT] << 0.0, LS_LEFT * H, 0.0; Hvec[RIGHT] << 0.0, LS_RIGHT * H, 0.0; }
		void trunkLinkOffsetCB() { trunkLinkOffset << trunkLinkOffsetX(), trunkLinkOffsetY(), trunkLinkOffsetZ(); }
	};

	/**
	* @class RobotKinematics
	* 
	* @brief Generic base class to encapsulate the kinematics of a humanoid robot.
	**/
	class RobotKinematics
	{
	public:
		// Typedefs
		typedef pose_classes::LegTipPoint LegTipPoint;
		typedef pose_classes::ArmTipPoint ArmTipPoint;
		typedef pose_classes::HeadTipPoint HeadTipPoint;
		typedef pose_classes::LegTipPointVel LegTipPointVel;
		typedef pose_classes::ArmTipPointVel ArmTipPointVel;
		typedef pose_classes::HeadTipPointVel HeadTipPointVel;

		// Constructor/destructor
		RobotKinematics() : rconfig(RobotConfig::getInstance()) {}
		virtual ~RobotKinematics() = default;

		// Configuration parameters
		const RobotConfig& rconfig;

		// Robot scale functions
		virtual double legScaleInv() const = 0; //!< @brief Returns a measure of scale of the robot legs as relevant for inverse leg poses
		virtual double legScaleTip() const = 0; //!< @brief Returns a measure of scale of the robot legs as relevant for leg tip poses
		virtual double armScaleInv() const = 0; //!< @brief Returns a measure of scale of the robot arms as relevant for inverse arm poses
		virtual double armScaleTip() const = 0; //!< @brief Returns a measure of scale of the robot arms as relevant for arm tip poses

		// Cartesian origin functions
		rot_conv::Vec3 origin(LimbIndex limbIndex, LimbType limbType) const;
		virtual rot_conv::Vec3 originLeg(LimbIndex limbIndex) const = 0; //!< @brief Returns the coordinates of the required leg cartesian origin relative to the corresponding hip point
		virtual rot_conv::Vec3 originArm(LimbIndex limbIndex) const = 0; //!< @brief Returns the coordinates of the required arm cartesian origin relative to the corresponding shoulder point
		virtual rot_conv::Vec3 originHead(LimbIndex limbIndex) const = 0; //!< @brief Returns the coordinates of the required head cartesian origin relative to the corresponding neck point
		virtual rot_conv::Vec3 hipPoint(LimbIndex limbIndex) const = 0; //!< @brief Returns the coordinates of the required hip point relative to the corresponding leg cartesian origin
		virtual rot_conv::Vec3 shoulderPoint(LimbIndex limbIndex) const = 0; //!< @brief Returns the coordinates of the required shoulder point relative to the corresponding arm cartesian origin
		virtual rot_conv::Vec3 neckPoint(LimbIndex limbIndex) const = 0; //!< @brief Returns the coordinates of the required neck point relative to the corresponding head cartesian origin
	};

	// Typedefs
	typedef std::shared_ptr<RobotKinematics> RobotKinematicsPtr;

	/**
	* @class KinematicsWrapperBase
	* 
	* @brief Base class for the templated KinematicsWrapper class.
	**/
	class KinematicsWrapperBase
	{
	protected:
		// Constructor/destructor
		KinematicsWrapperBase() = default;
		virtual ~KinematicsWrapperBase() = default;

	public:
		// Joint information
		virtual void getJointInfo(std::vector<JointInfo>& info) const = 0;

		// Field names
		virtual void getJointFields(std::vector<std::string>& names) const = 0;
		virtual void getJointFieldsShort(std::vector<std::string>& names) const = 0;
		virtual void getAbsFields(std::vector<std::string>& names) const = 0;
		virtual void getAbsFieldsShort(std::vector<std::string>& names) const = 0;
		virtual void getInvFields(std::vector<std::string>& names) const = 0;
		virtual void getInvFieldsShort(std::vector<std::string>& names) const = 0;
		virtual void getTipFields(std::vector<std::string>& names) const = 0;
		virtual void getTipFieldsShort(std::vector<std::string>& names) const = 0;
	};

	// Typedefs
	typedef std::shared_ptr<KinematicsWrapperBase> KinematicsWrapperBasePtr;

	/**
	* @class KinematicsWrapper
	* 
	* @brief Templated wrapper class for the robot kinematics, i.e. the RobotKinematics class or any class derived thereof.
	**/
	template<class Kinematics> class KinematicsWrapper : public KinematicsWrapperBase
	{
	private:
		// Static assertions
		static_assert(std::is_base_of<RobotKinematics, Kinematics>::value, "The Kinematics template parameter must derive from RobotKinematics");

	public:
		// Typedefs
		typedef typename Kinematics::JointPose JointPose;
		typedef typename Kinematics::AbsPose AbsPose;
		typedef typename Kinematics::InvPose InvPose;
		typedef typename Kinematics::TipPose TipPose;

		// Constructor/destructor
		KinematicsWrapper() = default;
		virtual ~KinematicsWrapper() = default;

		// Joint information
		virtual void getJointInfo(std::vector<JointInfo>& info) const override { m_JP.getJointInfo(info); }

		// Field names
		virtual void getJointFields(std::vector<std::string>& names) const override { m_JP.getFieldNames(names); }
		virtual void getJointFieldsShort(std::vector<std::string>& names) const override { m_JP.getFieldNamesShort(names); }
		virtual void getAbsFields(std::vector<std::string>& names) const override { m_AP.getFieldNames(names); }
		virtual void getAbsFieldsShort(std::vector<std::string>& names) const override { m_AP.getFieldNamesShort(names); }
		virtual void getInvFields(std::vector<std::string>& names) const override { m_IP.getFieldNames(names); }
		virtual void getInvFieldsShort(std::vector<std::string>& names) const override { m_IP.getFieldNamesShort(names); }
		virtual void getTipFields(std::vector<std::string>& names) const override { m_TP.getFieldNames(names); }
		virtual void getTipFieldsShort(std::vector<std::string>& names) const override { m_TP.getFieldNamesShort(names); }

	private:
		// Static pose instances
		static const JointPose m_JP;
		static const AbsPose m_AP;
		static const InvPose m_IP;
		static const TipPose m_TP;
	};

	// Static pose instances
	template<class Kinematics> const typename KinematicsWrapper<Kinematics>::JointPose KinematicsWrapper<Kinematics>::m_JP;
	template<class Kinematics> const typename KinematicsWrapper<Kinematics>::AbsPose KinematicsWrapper<Kinematics>::m_AP;
	template<class Kinematics> const typename KinematicsWrapper<Kinematics>::InvPose KinematicsWrapper<Kinematics>::m_IP;
	template<class Kinematics> const typename KinematicsWrapper<Kinematics>::TipPose KinematicsWrapper<Kinematics>::m_TP;
}

#endif
// EOF