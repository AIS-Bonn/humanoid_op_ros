// Feedback gait trivial odometry
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_TRIVIAL_ODOM_H
#define FEED_TRIVIAL_ODOM_H

// Includes
#include <feed_gait/feed_common.h>
#include <feed_gait/odometry/feed_odometry_base.h>
#include <config_server/parameter.h>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @namespace trivial_odom
	* 
	* @brief Trivial odometry namespace.
	**/
	namespace trivial_odom
	{
		/**
		* @class TrivialOdomConfig
		* 
		* @brief Trivial odometry configuration parameters class.
		**/
		class TrivialOdomConfig
		{
		private:
			// Constructor
			TrivialOdomConfig()
				: TYPE_NAME(odometryTypeName(OT_TRIVIAL))
				, CONFIG_PARAM_PATH(ODOM_CONFIG_PARAM_PATH + TYPE_NAME + "/")
				, odomPosZ(CONFIG_PARAM_PATH + "odomPosZ", 0.0, 0.01, 2.0, 0.0)
			{}

			// Ensure class remains a singleton
			TrivialOdomConfig(const TrivialOdomConfig&) = delete;
			TrivialOdomConfig& operator=(const TrivialOdomConfig&) = delete;

		public:
			// Get singleton instance of class
			static const TrivialOdomConfig& getInstance() { static thread_local TrivialOdomConfig toconfig; return toconfig; }

			// Constants
			const std::string TYPE_NAME;
			const std::string CONFIG_PARAM_PATH;

			// Configuration parameters
			config_server::Parameter<float> odomPosZ; //!< @brief Fixed z-coordinate to use for the robot odometry position
		};

		/**
		* @class FeedTrivialOdom
		*
		* @brief Feedback gait trivial odometry class.
		**/
		class FeedTrivialOdom : public FeedOdometryBase
		{
		public:
			// Constructor/destructor
			explicit FeedTrivialOdom(FeedPlotManager* PM) : FeedOdometryBase(PM), toconfig(TrivialOdomConfig::getInstance()) { OdometryInput odomInput; resetMembers(odomInput); }
			virtual ~FeedTrivialOdom() = default;

			// Configuration parameters
			const TrivialOdomConfig& toconfig;

			// Reset function
			virtual void reset(const OdometryInput& odomInput) override { FeedOdometryBase::reset(odomInput); resetMembers(odomInput); }

			// Set function for the 2D pose
			virtual void setPose2D(double posX, double posY, double rotZ) override;

			// Update function
			virtual void update(const OdometryInput& odomInput) override;

		protected:
			// Last orientation input
			rot_conv::FusedAngles m_lastRobotOrient;

		private:
			// Reset members function
			void resetMembers(const OdometryInput& odomInput);
		};
	}
}

#endif
// EOF