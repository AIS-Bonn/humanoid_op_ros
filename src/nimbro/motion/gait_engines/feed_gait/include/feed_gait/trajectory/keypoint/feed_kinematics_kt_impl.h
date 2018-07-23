// Feedback gait kinematics for keypoint trajectory generation implementation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_KINEMATICS_KT_IMPL_H
#define FEED_KINEMATICS_KT_IMPL_H

// Includes
#include <feed_gait/trajectory/keypoint/feed_kinematics_kt.h>

// Feedback gait namespace
namespace feed_gait
{
	// Keypoint trajectory generation namespace
	namespace keypoint_traj
	{
		//
		// FeedKinematicsKT class
		//

		// Constructor
		template<class Kinematics> FeedKinematicsKT<Kinematics>::FeedKinematicsKT(const KeypointTrajConfig& ktconfig)
			: FeedKinematics<Kinematics>()
			, ktkonfig(KinKTConfig<Kinematics>::getInstance())
			, ktconfig(ktconfig)
		{
			// Note: This static assert must occur after all template specialisations of KinKTConfig are complete
			static_assert(std::is_base_of<KinConfigBase, KinKTConfig<Kinematics>>::value, "The KinKTConfig class must always derive from KinConfigBase");
		}
	}
}

#endif
// EOF