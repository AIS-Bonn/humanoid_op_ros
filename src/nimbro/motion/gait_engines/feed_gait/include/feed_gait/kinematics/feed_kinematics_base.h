// Feedback gait kinematics base class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_KINEMATICS_BASE_H
#define FEED_KINEMATICS_BASE_H

// Includes
#include <feed_gait/feed_common.h>
#include <config_server/parameter.h>
#include <memory>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @class CommonKinConfig
	* 
	* @brief Common kinematics interface configuration parameters class.
	**/
	class CommonKinConfig
	{
	private:
		// Constructor
		CommonKinConfig()
			: CONFIG_PARAM_PATH(KIN_CONFIG_PARAM_PATH + "common/")
		{}

		// Ensure class remains a singleton
		CommonKinConfig(const CommonKinConfig&) = delete;
		CommonKinConfig& operator=(const CommonKinConfig&) = delete;

	public:
		// Get singleton instance of class
		static const CommonKinConfig& getInstance() { static thread_local CommonKinConfig ckonfig; return ckonfig; }

		// Constants
		const std::string CONFIG_PARAM_PATH;
	};

	/**
	* @class FeedKinematicsBase
	*
	* @brief Feedback gait kinematics base class.
	**/
	class FeedKinematicsBase
	{
	public:
		// Constructor/destructor
		FeedKinematicsBase() : ckonfig(CommonKinConfig::getInstance()) {}
		virtual ~FeedKinematicsBase() = default;

		// Gait halt pose functions
		void getHaltPose(PoseCommand& haltPose) const;
		virtual void getHaltPose(PoseCommand::DblVec& pos) const = 0; // This function should write the required halt pose joint positions to pos
		virtual void getHaltPoseEffort(PoseCommand::DblVec& effort) const = 0; // This function should write the required halt pose joint efforts to effort
		virtual void getHaltPoseSuppCoeff(PoseCommand::SuppCoeff& suppCoeff) const = 0; // This function should write the required halt pose support coefficients to suppCoeff

		// Configuration parameters
		const CommonKinConfig& ckonfig;
	};

	// Typedefs
	typedef std::shared_ptr<FeedKinematicsBase> FeedKinematicsBasePtr;
}

#endif
// EOF