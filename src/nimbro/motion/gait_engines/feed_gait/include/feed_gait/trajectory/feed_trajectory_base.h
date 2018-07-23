// Feedback gait trajectory generation base class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_TRAJECTORY_BASE_H
#define FEED_TRAJECTORY_BASE_H

// Includes
#include <feed_gait/feed_common.h>
#include <feed_gait/feed_plot.h>
#include <feed_gait/trajectory/feed_trajectory_common.h>
#include <config_server/parameter.h>
#include <memory>
#include <vector>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @class CommonTrajConfig
	* 
	* @brief Common trajectory generation configuration parameters class.
	**/
	class CommonTrajConfig
	{
	private:
		// Constructor
		CommonTrajConfig()
			: CONFIG_PARAM_PATH(TRAJ_CONFIG_PARAM_PATH + "common/")
		{}

		// Ensure class remains a singleton
		CommonTrajConfig(const CommonTrajConfig&) = delete;
		CommonTrajConfig& operator=(const CommonTrajConfig&) = delete;

	public:
		// Get singleton instance of class
		static const CommonTrajConfig& getInstance() { static thread_local CommonTrajConfig ctconfig; return ctconfig; }

		// Constants
		const std::string CONFIG_PARAM_PATH;
	};

	/**
	* @class FeedTrajectoryBase
	*
	* @brief Feedback gait trajectory base class.
	**/
	class FeedTrajectoryBase
	{
	public:
		// Constructor/destructor
		explicit FeedTrajectoryBase(FeedPlotManager* PM) : ctconfig(CommonTrajConfig::getInstance()), m_PM(PM) {}
		virtual ~FeedTrajectoryBase() = default;

		// Configuration parameters
		const CommonTrajConfig& ctconfig;

		// Information function
		virtual void getInfo(TrajInfo& trajInfo) const = 0; // This function should write the required information to trajInfo

		// Gait halt pose function
		virtual void getHaltPose(PoseCommand& haltPose) const = 0; // This function should write the required halt pose to haltPose (Note: The halt pose does NOT necessarily need to be recalculated with the latest config values, as long as the halt pose was properly calculated in the class constructor, and that calculated pose is returned here!)

		// Trajectory generation function
		virtual void generate(const TrajCommand& trajCmd) = 0; // This function should regenerate the trajectory internally based on the given trajectory command (essentially resetting all state of the class), so that future calls to evaluate() can immediately evaluate the trajectory at given gait phases

		// Evaluate function
		virtual void evaluate(double gaitPhase, PoseCommand& poseCmd) const = 0; // This function should evaluate the last-generated trajectory at the given gait phase, and write the result to poseCmd

	protected:
		// Plot manager
		FeedPlotManager* const m_PM;
	};

	// Typedefs
	typedef std::shared_ptr<FeedTrajectoryBase> FeedTrajectoryBasePtr;
}

#endif
// EOF