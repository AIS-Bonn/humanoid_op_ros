// Feedback gait trivial odometry
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/odometry/trivial/feed_trivial_odom.h>

// Namespaces
using namespace feed_gait;
using namespace feed_gait::trivial_odom;

//
// FeedTrivialOdom class
//

// Reset members function
void FeedTrivialOdom::resetMembers(const OdometryInput& odomInput)
{
	// Reset the last orientation input
	m_lastRobotOrient = odomInput.robotOrient;

	// Reset the odometry output struct
	m_out.pos2D << 0.0, 0.0;
	m_out.pos3D << 0.0, 0.0, toconfig.odomPosZ();
	m_out.rot2D = 0.0;
	m_out.rot3D = rot_conv::QuatFromFused(m_out.rot2D, m_lastRobotOrient.fusedPitch, m_lastRobotOrient.fusedRoll, m_lastRobotOrient.hemi);
	m_out.supportLeg.set(hk::LEFT);
}

// Set function for the 2D pose
void FeedTrivialOdom::setPose2D(double posX, double posY, double rotZ)
{
	// Set the 2D pose
	m_out.pos2D << posX, posY;
	m_out.pos3D << posX, posY, toconfig.odomPosZ();
	m_out.rot2D = rotZ;
	m_out.rot3D = rot_conv::QuatFromFused(m_out.rot2D, m_lastRobotOrient.fusedPitch, m_lastRobotOrient.fusedRoll, m_lastRobotOrient.hemi);
}

// Update function
void FeedTrivialOdom::update(const OdometryInput& odomInput)
{
	// Update the last orientation input
	m_lastRobotOrient = odomInput.robotOrient;

	// Update the odometry output struct
	m_out.pos3D.z() = toconfig.odomPosZ();
	m_out.rot3D = rot_conv::QuatFromFused(m_out.rot2D, m_lastRobotOrient.fusedPitch, m_lastRobotOrient.fusedRoll, m_lastRobotOrient.hemi);
}
// EOF