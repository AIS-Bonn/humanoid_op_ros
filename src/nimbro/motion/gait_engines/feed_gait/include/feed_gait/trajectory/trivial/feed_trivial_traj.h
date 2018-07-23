// Feedback gait trivial trajectory generation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_TRIVIAL_TRAJ_H
#define FEED_TRIVIAL_TRAJ_H

// Includes
#include <feed_gait/feed_common.h>
#include <feed_gait/trajectory/feed_trajectory_base.h>
#include <feed_gait/kinematics/feed_kinematics.h>
#include <humanoid_kinematics/robot_kinematics.h>
#include <config_server/parameter.h>
#include <type_traits>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @namespace trivial_traj
	* 
	* @brief Trivial trajectory generation namespace.
	**/
	namespace trivial_traj
	{
		/**
		* @class TrivialTrajConfig
		*
		* @brief Trivial trajectory generation configuration parameters class.
		**/
		class TrivialTrajConfig
		{
		private:
			// Constructor
			TrivialTrajConfig()
				: TYPE_NAME(trajectoryTypeName(TT_TRIVIAL))
				, CONFIG_PARAM_PATH(TRAJ_CONFIG_PARAM_PATH + TYPE_NAME + "/")
			{}

			// Ensure class remains a singleton
			TrivialTrajConfig(const TrivialTrajConfig&) = delete;
			TrivialTrajConfig& operator=(const TrivialTrajConfig&) = delete;

		public:
			// Get singleton instance of class
			static const TrivialTrajConfig& getInstance() { static thread_local TrivialTrajConfig ttconfig; return ttconfig; }

			// Constants
			const std::string TYPE_NAME;
			const std::string CONFIG_PARAM_PATH;
		};

		/**
		* @class FeedTrivialTraj
		*
		* @brief Feedback gait trivial trajectory generation class.
		**/
		template<class Kinematics> class FeedTrivialTraj : public FeedTrajectoryBase
		{
		private:
			// Static assertions
			static_assert(std::is_base_of<humanoid_kinematics::RobotKinematics, Kinematics>::value, "The Kinematics template parameter must derive from humanoid_kinematics::RobotKinematics");

		public:
			// Typedefs
			typedef FeedKinematics<Kinematics> KinematicsInterface;

			// Constructor/destructor
			explicit FeedTrivialTraj(FeedPlotManager* PM) : FeedTrajectoryBase(PM), ttconfig(TrivialTrajConfig::getInstance()), KI(), K(KI.K) { init(); }
			virtual ~FeedTrivialTraj() = default;

			// Configuration parameters
			const TrivialTrajConfig& ttconfig;

			// Kinematics interface
			const KinematicsInterface KI;
			const Kinematics& K;

			// Information function
			virtual void getInfo(TrajInfo& trajInfo) const override;

			// Gait halt pose function
			virtual void getHaltPose(PoseCommand& haltPose) const override;

			// Trajectory generation function
			virtual void generate(const TrajCommand& trajCmd) override;

			// Evaluate function
			virtual void evaluate(double gaitPhase, PoseCommand& poseCmd) const override;

		protected:
			// Initialisation function
			void init();

			// Trajectory variables
			PoseCommand m_haltPose;
		};
	}
}

// Include implementations that should occur in the header
#include <feed_gait/trajectory/trivial/feed_trivial_traj_impl.h>

#endif
// EOF