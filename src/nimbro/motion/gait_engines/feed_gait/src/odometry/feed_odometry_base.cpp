// Feedback gait odometry base class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/odometry/feed_odometry_base.h>

// Namespaces
using namespace feed_gait;

//
// FeedOdometryBase class
//

// Call update wrapper function
void FeedOdometryBase::callUpdate(const OdometryInput& odomInput)
{
	// Initialise the class with the first received odometry inputs
	if(!m_inited)
	{
		reset(odomInput);
		m_inited = true;
	}

	// Call the virtual update function
	update(odomInput);
}
// EOF