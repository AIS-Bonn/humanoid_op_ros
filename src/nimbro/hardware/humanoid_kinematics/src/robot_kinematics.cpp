// Humanoid kinematics - Generic robot kinematics
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <humanoid_kinematics/robot_kinematics.h>

// Humanoid kinematics namespace
namespace humanoid_kinematics
{
	//
	// RobotKinematics class
	//

	// Cartesian origin function
	rot_conv::Vec3 RobotKinematics::origin(LimbIndex limbIndex, LimbType limbType) const
	{
		// Return the required cartesian origin
		if(limbType == LT_LEG)
			return originLeg(limbIndex);
		else if(limbType == LT_ARM)
			return originArm(limbIndex);
		else if(limbType == LT_HEAD)
			return originHead(limbIndex);
		else
			return rot_conv::Vec3::Zero();
	}
}
// EOF