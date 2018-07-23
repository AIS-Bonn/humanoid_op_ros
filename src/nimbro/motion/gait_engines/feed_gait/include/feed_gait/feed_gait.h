// Feedback gait
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_GAIT_H
#define FEED_GAIT_H

// Includes
#include <feed_gait/feed_common.h>
#include <feed_gait/feed_config.h>
#include <feed_gait/feed_utils.h>
#include <feed_gait/feed_plot.h>
#include <feed_gait/feed_gait_common.h>
#include <feed_gait/kinematics/feed_kinematics_base.h>
#include <feed_gait/trajectory/feed_trajectory_base.h>
#include <feed_gait/odometry/feed_odometry_base.h>
#include <feed_gait/model/feed_model_base.h>
#include <humanoid_kinematics/robot_kinematics.h>
#include <rc_utils/wlbf_filter_nd.h>
#include <gait/gait_engine.h>

/**
* @namespace feed_gait
*
* @brief Namespace that defines everything that is required for the feedback gait.
**/
namespace feed_gait
{
	/**
	* @class FeedGait
	*
	* @brief A feedback gait, implemented as a `GaitEngine` plugin for the `Gait` motion module.
	**/
	class FeedGait : public gait::GaitEngine
	{
	public:
		//
		// General
		//

		// Constructor/destructor
		FeedGait();
		virtual ~FeedGait();

		//
		// Gait engine overrides
		//

		// Reset function
		virtual void reset() override;

		// Step function
		virtual void step() override;

		// Halt pose function
		virtual void updateHaltPose() override;

		// Set odometry function
		virtual void setOdometry(double posX, double posY, double rotZ) override;

		// Update odometry function
		virtual void updateOdometry() override;

		// Handle joystick button function
		virtual void handleJoystickButton(int button) override;

	private:
		//
		// Configuration
		//

		// Configuration parameters
		FeedConfig config;

		//
		// Dynamic objects
		//

		// Reset function for dynamic objects
		void resetDynamicObjects();

		// Current dynamic object types
		KinematicsType m_currentKinType;        // Current kinematics type
		TrajectoryType m_currentTrajType;       // Current trajectory generation type
		OdometryType m_currentOdomType;         // Current odometry type

		// Dynamic objects
		hk::KinematicsWrapperBasePtr KW;        // Kinematics wrapper class instance
		FeedKinematicsBasePtr KI;               // Kinematics interface class instance
		FeedTrajectoryBasePtr T;                // Trajectory generation class instance
		FeedOdometryBasePtr O;                  // Odometry class instance
		std::vector<FeedModelBasePtr> m_models; // Model classes

		// Halt pose
		PoseCommand m_haltPose;           // Last updated halt pose of the robot

		// Kinematics wrapper class factory function
		hk::KinematicsWrapperBasePtr createKinematicsWrapper() const { return createKinematicsWrapper(m_currentKinType); }
		hk::KinematicsWrapperBasePtr createKinematicsWrapper(KinematicsType kinType) const;

		// Kinematics interface class factory function
		FeedKinematicsBasePtr createKinematicsInterface() const { return createKinematicsInterface(m_currentKinType); }
		FeedKinematicsBasePtr createKinematicsInterface(KinematicsType kinType) const;

		// Trajectory generation class factory function
		FeedTrajectoryBasePtr createTrajectory() const { return createTrajectory(m_currentTrajType, m_currentKinType); }
		FeedTrajectoryBasePtr createTrajectory(TrajectoryType trajType) const { return createTrajectory(trajType, m_currentKinType); }
		FeedTrajectoryBasePtr createTrajectory(TrajectoryType trajType, KinematicsType kinType) const;
		template<class Kinematics> FeedTrajectoryBasePtr createTrajectory(TrajectoryType trajType) const;

		// Odometry class factory function
		FeedOdometryBasePtr createOdometry() const { return createOdometry(m_currentOdomType, m_currentKinType); }
		FeedOdometryBasePtr createOdometry(OdometryType odomType) const { return createOdometry(odomType, m_currentKinType); }
		FeedOdometryBasePtr createOdometry(OdometryType odomType, KinematicsType kinType) const;
		template<class Kinematics> FeedOdometryBasePtr createOdometry(OdometryType odomType) const;

		// Model class factory function
		FeedModelBasePtr createModel(ModelType modelType) const { return createModel(modelType, m_currentKinType); }
		FeedModelBasePtr createModel(ModelType modelType, KinematicsType kinType) const;
		template<class Kinematics> FeedModelBasePtr createModel(ModelType modelType) const;

		// Recreate model classes function
		void recreateModels() { recreateModels(m_currentKinType); }
		void recreateModels(KinematicsType kinType);

		// Joint information
		static const std::size_t NullJointIndex = (std::size_t) -1;
		typedef std::vector<hk::JointInfo> JointInfoVec;
		typedef std::map<std::size_t, std::size_t> JointIndexMap;
		JointInfoVec m_jointInfo; // Joint information in Kinematics order
		std::size_t numKinJoints() const { return m_jointInfo.size(); }
		JointIndexMap m_jointIndexMapGK; // Maps joint indices from Gait to Kinematics order (mapped indices are NullJointIndex if no mapping exists)
		JointIndexMap m_jointIndexMapKG; // Maps joint indices from Kinematics to Gait order (mapped indices are NullJointIndex if no mapping exists)
		void updateJointInformation();

		// Conversion functions between joint positions and efforts in Gait and Kinematics order
		void jointPosGaitToKin(const double (&gaitPos)[gait::NUM_JOINTS], std::vector<double>& kinPos) const;
		void jointPosKinToGait(const std::vector<double>& kinPos, double (&gaitPos)[gait::NUM_JOINTS]) const;
		void jointEffortGaitToKin(const double (&gaitEffort)[gait::NUM_JOINTS], std::vector<double>& kinEffort) const;
		void jointEffortKinToGait(const std::vector<double>& kinEffort, double (&gaitEffort)[gait::NUM_JOINTS]) const;
		void jointPosEffortGaitToKin(const double (&gaitPos)[gait::NUM_JOINTS], const double (&gaitEffort)[gait::NUM_JOINTS], std::vector<double>& kinPos, std::vector<double>& kinEffort) const;
		void jointPosEffortKinToGait(const std::vector<double>& kinPos, const std::vector<double>& kinEffort, double (&gaitPos)[gait::NUM_JOINTS], double (&gaitEffort)[gait::NUM_JOINTS]) const;
		void jointPosEffortGaitToKin(const double (&gaitPos)[gait::NUM_JOINTS], const double (&gaitEffort)[gait::NUM_JOINTS], PoseCommand& kin) const { jointPosEffortGaitToKin(gaitPos, gaitEffort, kin.pos, kin.effort); }
		void jointPosEffortKinToGait(const PoseCommand& kin, double (&gaitPos)[gait::NUM_JOINTS], double (&gaitEffort)[gait::NUM_JOINTS]) const { jointPosEffortKinToGait(kin.pos, kin.effort, gaitPos, gaitEffort); }

		//
		// Step functions
		//

		// Step function constants
		static constexpr double MinGaitFrequency = 1e-8;

		// Input processing functions
		GaitInput calcGaitInputs() const;
		OdometryInput calcOdomInputs(const GaitInput& gaitInput) const;
		ModelInput calcModelInputs(const OdometryInput& odomInput, const GaitInput& gaitInput, const TrajInfo& trajInfo) const;

		// Update functions
		void updateGaitPhaseInfo() { m_gaitPhaseInfo.setGaitPhase(m_gaitPhase); }
		TimingOutput updateTiming();
		StepSizeOutput updateStepSize(const GaitInput& gaitInput, const TimingOutput& timingOutput);

		// Output processing functions
		void updateGaitOutputs(const PoseCommand& poseCmd);

		//
		// Walking
		//

		// Reset function for walking
		void resetWalking();

		// Set function for walking
		void setWalking(bool walking);

		// Walking flags
		bool m_walking;

		// Gait phase variables
		bool m_leftLegFirst;           // True => Left leg has swing phase first / False => Right leg has swing phase first
		double m_gaitPhase;            // (0,pi] => Swing phase of left leg, support phase of right leg / (-pi,0] => Swing phase of right leg, support phase of left leg
		GaitPhaseInfo m_gaitPhaseInfo; // Information about the gait phase, including its value, phase-dependent support leg sign, and remaining gait phase to step

		// Gait command vector variables
		GcvCommand m_currentGcvCmd;
		Vec3 m_currentGcv;

		// Gait command acceleration variables
		rc_utils::WLBFFilter3D m_gcvLFAccFilter;
		Vec3 m_currentGcvLFAcc;
		void updateGcvAcc();

		// Halt pose blender
		static constexpr double GenBlendFactor = 0.0;
		static constexpr double HaltBlendFactor = 1.0;
		PoseCommandBlender m_haltBlender;

		//
		// Plotting
		//

		// Plot manager
		FeedPlotManager* const m_PM;
		void callbackPlotData();
	};
}

// Include implementations that should occur in the header
#include <feed_gait/feed_gait_impl.h>

#endif
// EOF