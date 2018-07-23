// Feedback gait trivial trajectory generation implementation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_TRIVIAL_TRAJ_IMPL_H
#define FEED_TRIVIAL_TRAJ_IMPL_H

// Includes
#include <feed_gait/trajectory/trivial/feed_trivial_traj.h>

// Feedback gait namespace
namespace feed_gait
{
	// Trivial trajectory generation namespace
	namespace trivial_traj
	{
		//
		// FeedTrivialTraj class
		//

		// Initialisation function
		template<class Kinematics> void FeedTrivialTraj<Kinematics>::init()
		{
			// Initialise the halt pose
			getHaltPose(m_haltPose);
		}

		// Information function
		template<class Kinematics> void FeedTrivialTraj<Kinematics>::getInfo(TrajInfo& trajInfo) const
		{
			// Populate the required information
			trajInfo.reset();
		}

		// Gait halt pose function
		template<class Kinematics> void FeedTrivialTraj<Kinematics>::getHaltPose(PoseCommand& haltPose) const
		{
			// Calculate the required halt pose
			KI.getHaltPose(haltPose);
		}

		// Trajectory generation function
		template<class Kinematics> void FeedTrivialTraj<Kinematics>::generate(const TrajCommand& trajCmd)
		{
			// Compute the halt pose
			getHaltPose(m_haltPose);
		}

		// Evaluate function
		template<class Kinematics> void FeedTrivialTraj<Kinematics>::evaluate(double gaitPhase, PoseCommand& poseCmd) const
		{
			// Evaluate the trajectory
			poseCmd = m_haltPose;
		}
	}
}

#endif
// EOF