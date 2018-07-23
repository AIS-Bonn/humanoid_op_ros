// Feedback gait kinematics implementation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_KINEMATICS_IMPL_H
#define FEED_KINEMATICS_IMPL_H

// Includes
#include <feed_gait/kinematics/feed_kinematics.h>
#include <type_traits>

// Feedback gait namespace
namespace feed_gait
{
	//
	// FeedKinematics class
	//

	// Constructor
	template<class Kinematics> FeedKinematics<Kinematics>::FeedKinematics()
		: FeedKinematicsBase()
		, konfig(KinConfig<Kinematics>::getInstance())
		, K()
		, RK(K)
	{
		// Note: This static assert must occur after all template specialisations of KinConfig are complete
		static_assert(std::is_base_of<KinConfigBase, KinConfig<Kinematics>>::value, "The KinConfig class must always derive from KinConfigBase");
	}
}

#endif
// EOF