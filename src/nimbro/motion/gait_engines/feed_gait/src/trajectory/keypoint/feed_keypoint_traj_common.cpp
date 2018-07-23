// Feedback gait keypoint trajectory generation common
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/trajectory/keypoint/feed_keypoint_traj_common.h>

// Namespaces
using namespace feed_gait;
using namespace feed_gait::keypoint_traj;

//
// NominalFootTilt struct
//

// Set function with tilt angles
void NominalFootTilt::set(const rot_conv::TiltAngles& t)
{
	// Set the data members as required
	tilt = t;
	rot_conv::AbsYawTiltFromTilt(tilt, absTilt);
}

// Set function with quaternion
void NominalFootTilt::set(const Quat& q)
{
	// Set the data members as required
	rot_conv::TiltFromQuat(q, tilt);
	rot_conv::AbsYawTiltFromTilt(tilt, absTilt);
}
// EOF