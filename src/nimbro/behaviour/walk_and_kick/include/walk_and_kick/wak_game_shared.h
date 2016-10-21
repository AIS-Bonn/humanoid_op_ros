// Walk and kick: Class for shared walk and kick game state variables
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef WAK_GAME_SHARED_H
#define WAK_GAME_SHARED_H

// Includes
#include <walk_and_kick/wak_common.h>
#include <walk_and_kick/wak_config.h>
#include <walk_and_kick/wak_game_vars.h>
#include <walk_and_kick/wak_sensor_vars.h>
#include <walk_and_kick/wak_ros_interface.h>
#include <walk_and_kick/wak_utils.h>
#include <walk_and_kick/wak_vis.h>
#include <rc_utils/math_funcs.h>
#include <rc_utils/ros_time.h>

// Walk and kick namespace
namespace walk_and_kick
{
	// Class declarations
	class WAKGameState;
	class WAKGameManager;

	/**
	* @class WAKGameShared
	* 
	* @brief A class that shares the required walk and kick variables amongst the game state classes.
	**/
	class WAKGameShared
	{
	public:
		// Constructor
		WAKGameShared(WAKConfig& config, const SensorVars& SV, const WAKRosInterface& RI, WAKGameManager& GM);

		// Field dimensions
		const FieldDimensions field;

		// Plot manager
		plot_msgs::PlotManagerFS& PM;

		// Marker manager
		WAKMarkerMan& MM;

		// Game state registration function
		void registerState(WAKGameState* state, int ID, const std::string& name) const;

		// Reset function
		void resetShared();

		// Update function
		void updateShared();

		// Get functions
		bool gameStateIsNew() const;

		// Cycle numbers and times
		cycle_t wakCycle() const;
		cycle_t stateCycle() const;
		float wakTime() const;
		float stateTime() const;

		// Shared game state functions
		bool obstacleBallHandling(GameVars& GV) const;
		Vec2f applyGoalSign(const Vec2f& pose) const { return (SV.goalSign > 0 ? pose : Vec2f(-pose.x(), -pose.y())); }
		Vec3f applyGoalSign(const Vec3f& pose) const { return (SV.goalSign > 0 ? Vec3f(pose.x(), pose.y(), rc_utils::picut(pose.z())) : Vec3f(-pose.x(), -pose.y(), rc_utils::picut(pose.z() + M_PI))); }
		Vec3f getKickoffPoseTarget() const;
		bool poseIsLegalForKickoff(const Vec3f& pose) const;

		// Shared game state variables
		/* None yet */

	private:
		// Config parameters
		WAKConfig& config;

		// Sensor variables
		const SensorVars& SV;

		// ROS interface
		const WAKRosInterface& RI;

		// Game manager
		WAKGameManager& GM;
	};

	/**
	* @class DynamicTargetPose
	* 
	* @brief Simple class that encapsulates the variables needed to easily dynamically calculate a target pose.
	**/
	class DynamicTargetPose
	{
	public:
		// Constructor
		DynamicTargetPose() { clear(); }

		// Set functions
		void clear() { m_targetPose.setZero(); m_targetPoseExists = false; rc_utils::zeroRosTime(m_targetPoseTime); }
		void set(const Vec3f& targetPose, const ros::Time& now) { m_targetPose = targetPose; m_targetPoseExists = true; m_targetPoseTime = now; }

		// Get functions
		bool exists() const { return m_targetPoseExists; }
		const Vec3f& value() const { return m_targetPose; }
		float timeSinceSet(const ros::Time& now) const { return (m_targetPoseExists ? (now - m_targetPoseTime).toSec() : INFINITY); }
		bool hasExpired(float expireTime, const ros::Time& now) { return (!m_targetPoseExists || (now - m_targetPoseTime).toSec() >= expireTime); }

	private:
		// Internal variables
		Vec3f m_targetPose;
		bool m_targetPoseExists;
		ros::Time m_targetPoseTime;
	};
}

#endif
// EOF