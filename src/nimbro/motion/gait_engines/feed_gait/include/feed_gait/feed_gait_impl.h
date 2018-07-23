// Feedback gait implementation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_GAIT_IMPL_H
#define FEED_GAIT_IMPL_H

// Includes
#include <feed_gait/feed_gait.h>
#include <feed_gait/trajectory/feed_trajectory.h>
#include <feed_gait/odometry/feed_odometry.h>
#include <feed_gait/model/feed_model.h>
#include <type_traits>
#include <memory>

// Feedback gait namespace
namespace feed_gait
{
	//
	// FeedGait class
	//

	// Trajectory generation class factory function
	template<class Kinematics> FeedTrajectoryBasePtr FeedGait::createTrajectory(TrajectoryType trajType) const
	{
		// Static assertions
		static_assert(std::is_base_of<humanoid_kinematics::RobotKinematics, Kinematics>::value, "The Kinematics template parameter must derive from humanoid_kinematics::RobotKinematics");

		// Create the required trajectory generation object
		if(trajType == TT_TRIVIAL)
			return std::make_shared<trivial_traj::FeedTrivialTraj<Kinematics>>(m_PM);
		else if(trajType == TT_KEYPOINT)
			return std::make_shared<keypoint_traj::FeedKeypointTraj<Kinematics>>(m_PM);
		else
		{
			ROS_ERROR("Attempted to create trajectory generation class of unknown trajectory type %d => Something is wrong!", trajType);
			return std::make_shared<trivial_traj::FeedTrivialTraj<Kinematics>>(m_PM);
		}
	}

	// Odometry class factory function
	template<class Kinematics> FeedOdometryBasePtr FeedGait::createOdometry(OdometryType odomType) const
	{
		// Static assertions
		static_assert(std::is_base_of<humanoid_kinematics::RobotKinematics, Kinematics>::value, "The Kinematics template parameter must derive from humanoid_kinematics::RobotKinematics");

		// Create the required odometry object
		if(odomType == OT_TRIVIAL)
			return std::make_shared<trivial_odom::FeedTrivialOdom>(m_PM);
		else if(odomType == OT_SIMPLE)
			return std::make_shared<simple_odom::FeedSimpleOdom<Kinematics>>(m_PM);
		else
		{
			ROS_ERROR("Attempted to create odometry class of unknown odometry type %d => Something is wrong!", odomType);
			return std::make_shared<trivial_odom::FeedTrivialOdom>(m_PM);
		}
	}

	// Model class factory function
	template<class Kinematics> FeedModelBasePtr FeedGait::createModel(ModelType modelType) const
	{
		// Static assertions
		static_assert(std::is_base_of<humanoid_kinematics::RobotKinematics, Kinematics>::value, "The Kinematics template parameter must derive from humanoid_kinematics::RobotKinematics");

		// Create the required model object
		if(modelType == MT_TRIVIAL)
			return std::make_shared<trivial_model::FeedTrivialModel>(m_PM);
		else if(modelType == MT_TILT_PHASE)
			return std::make_shared<tilt_phase_model::FeedTiltPhaseModel<Kinematics>>(m_PM);
		else
		{
			ROS_ERROR("Attempted to create model class of unknown model type %d => Something is wrong!", modelType);
			return std::make_shared<trivial_model::FeedTrivialModel>(m_PM);
		}
	}
}

#endif
// EOF