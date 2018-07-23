// Feedback gait kinematics base class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/kinematics/feed_kinematics_base.h>

// Namespaces
using namespace feed_gait;

//
// FeedKinematicsBase class
//

// Gait halt pose function: PoseCommand
void FeedKinematicsBase::getHaltPose(PoseCommand& haltPose) const
{
	// Calculate the required halt pose
	getHaltPose(haltPose.pos);
	getHaltPoseEffort(haltPose.effort);
	getHaltPoseSuppCoeff(haltPose.suppCoeff);
}
// EOF