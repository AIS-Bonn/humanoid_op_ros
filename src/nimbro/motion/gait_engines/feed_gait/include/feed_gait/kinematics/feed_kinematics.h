// Feedback gait kinematics
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_KINEMATICS_H
#define FEED_KINEMATICS_H

// Includes
#include <feed_gait/feed_common.h>
#include <feed_gait/kinematics/feed_kinematics_base.h>
#include <humanoid_kinematics/robot_kinematics.h>
#include <type_traits>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @class KinConfigBase
	* 
	* @brief Kinematics interface configuration parameters base class.
	**/
	class KinConfigBase
	{
	protected:
		// Constructor/destructor
		KinConfigBase() = default;
		virtual ~KinConfigBase() = default;

	private:
		// Ensure all derived classes remain a singleton
		KinConfigBase(const KinConfigBase&) = delete;
		KinConfigBase& operator=(const KinConfigBase&) = delete;
	};

	/**
	* @class KinConfig
	* 
	* @brief Kinematics interface configuration parameters class.
	**/
	template<class Kinematics> class KinConfig : public KinConfigBase
	{
	private:
		// Static assertions
		static_assert(std::is_base_of<humanoid_kinematics::RobotKinematics, Kinematics>::value, "The Kinematics template parameter must derive from humanoid_kinematics::RobotKinematics");

		// Constructor
		KinConfig() = delete;

		// Get singleton instance of class
		static const KinConfig<Kinematics>& getInstance() = delete;
	};

	/**
	* @class FeedKinematics
	*
	* @brief Feedback gait kinematics class.
	**/
	template<class Kinematics> class FeedKinematics : public FeedKinematicsBase
	{
	private:
		// Static assertions
		static_assert(std::is_base_of<humanoid_kinematics::RobotKinematics, Kinematics>::value, "The Kinematics template parameter must derive from humanoid_kinematics::RobotKinematics");

	public:
		// Typedefs
		typedef typename Kinematics::JointPose JointPose;
		typedef typename Kinematics::JointLegPose JointLegPose;
		typedef typename Kinematics::JointArmPose JointArmPose;
		typedef typename Kinematics::JointEffort JointEffort;
		typedef typename Kinematics::AbsPose AbsPose;
		typedef typename Kinematics::AbsLegPose AbsLegPose;
		typedef typename Kinematics::InvPose InvPose;
		typedef typename Kinematics::InvLegPose InvLegPose;

		// Constructor/destructor
		FeedKinematics();
		virtual ~FeedKinematics() = default;

		// Configuration parameters
		const KinConfig<Kinematics>& konfig;

		// Kinematics class object
		const Kinematics K;
		const hk::RobotKinematics& RK;

		// Gait halt pose functions
		using FeedKinematicsBase::getHaltPose;
		virtual void getHaltPose(PoseCommand::DblVec& pos) const override;
		virtual void getHaltPoseEffort(PoseCommand::DblVec& effort) const override;
		virtual void getHaltPoseSuppCoeff(PoseCommand::SuppCoeff& suppCoeff) const override;
		void getHaltPose(JointPose& JP) const = delete;
		void getHaltPose(AbsPose& AP) const = delete;
		void getHaltPose(InvPose& IP) const = delete;
		void getHaltPoseEffort(JointEffort& JE) const = delete;

		// Safe inverse kinematics functions
		bool JointFromInvSafe(const InvLegPose& ILP, JointLegPose& JLP) const = delete;
		bool AbsFromInvSafe(const InvLegPose& ILP, AbsLegPose& ALP) const = delete;
		bool JointAbsFromInvSafe(const InvLegPose& ILP, JointLegPose& JLP, AbsLegPose& ALP) const = delete;

		// CoM inverse kinematics functions
		void JointFromCoMRay(const Vec3& CoMRay, const JointArmPose& JAPRef, JointArmPose& JAP) const = delete;
		JointArmPose JointFromCoMRay(const Vec3& CoMRay, const JointArmPose& JAPRef) const { JointArmPose JAP(JAPRef.limbIndex); JointFromCoMRay(CoMRay, JAPRef, JAP); return JAP; }

		// Pose bias functions
		void biasAbsPose(AbsLegPose& ALP) const = delete;

		// Pose coercion functions
		bool JointCoerceSoft(JointArmPose& JAP) const = delete;
		bool AbsCoerceSoft(AbsLegPose& ALP) const = delete;
	};
}

// Include template specialisations for various kinematics
#include <feed_gait/kinematics/serial/feed_kinematics_impl_serial.h>

// Include implementations that should occur in the header (must occur after all template specialisations are complete)
#include <feed_gait/kinematics/feed_kinematics_impl.h>

#endif
// EOF