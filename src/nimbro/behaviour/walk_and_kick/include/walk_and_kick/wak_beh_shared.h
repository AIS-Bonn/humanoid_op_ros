// Walk and kick: Class for shared walk and kick behaviour state variables
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef WAK_BEH_SHARED_H
#define WAK_BEH_SHARED_H

// Includes
#include <walk_and_kick/wak_common.h>
#include <walk_and_kick/wak_config.h>
#include <walk_and_kick/wak_game_vars.h>
#include <walk_and_kick/wak_sensor_vars.h>
#include <walk_and_kick/wak_actuator_vars.h>
#include <walk_and_kick/wak_ros_interface.h>
#include <walk_and_kick/wak_utils.h>
#include <walk_and_kick/wak_vis.h>

// Walk and kick namespace
namespace walk_and_kick
{
	// Class declarations
	class WAKBehState;
	class WAKBehManager;

	// Ball action type enumeration
	enum BAType
	{
		BA_UNKNOWN = 0,
		BA_KICK,        // Look to kick the ball
		BA_DRIBBLE,     // Look to dribble the ball
		BA_COUNT,
		BA_DEFAULT = BA_KICK
	};
	const std::string BATypeName[BA_COUNT] = {
		"Unknown",
		"Kick",
		"Dribble"
	};
	inline bool ballActionTypeValid(int type) { return (type > BA_UNKNOWN && type < BA_COUNT); }
	inline bool ballActionTypeValid(BAType type) { return ballActionTypeValid((int) type); }
	inline const std::string& ballActionTypeName(BAType type) { if(ballActionTypeValid(type)) return BATypeName[type]; else return BATypeName[BA_UNKNOWN]; }

	/**
	* @class WAKBehShared
	* 
	* @brief A class that shares the required walk and kick variables amongst the behaviour state classes.
	**/
	class WAKBehShared
	{
	public:
		// Constructor
		WAKBehShared(WAKConfig& config, const SensorVars& SV, const WAKRosInterface& RI, WAKBehManager& BM);

		// Field dimensions
		const FieldDimensions field;

		// Plot manager
		plot_msgs::PlotManagerFS& PM;

		// Marker manager
		WAKMarkerMan& MM;

		// Behaviour state registration function
		void registerState(WAKBehState* state, int ID, const std::string& name) const;

		// Reset function
		void resetShared();

		// Update function
		void updateShared(const GameVars& GV);

		// Game variable inputs
		GameVars GV;

		// Get functions
		BAType ballAction() const;

		// Set functions
		void setWalkingTarget(const Vec2f& target, float tol = -1.0f) const; // Set the walking intent of the robot in the egocentric body-fixed frame
		void setWalkingTargetTol(float tol) const;

		// Cycle numbers and times
		cycle_t wakCycle() const;
		cycle_t stateCycle() const;
		float wakTime() const;
		float stateTime() const;

		// Shared behaviour state functions
		float walkToGlobalPose(ActuatorVars& AV, float targetX, float targetY) const { return walkToGlobalPose(AV, targetX, targetY, 0.0f, false); }
		float walkToGlobalPose(ActuatorVars& AV, float targetX, float targetY, float targetZ, bool useZ = true) const;
		bool obstacleAvoidance(Vec3f& GCV, const Vec2f& walkingTarget) const;
		bool gazeAtBall(ActuatorVars& AV, const ActuatorVars& lastAV) const;
		Vec2f calcGcvXY(float maxGcvX, float maxGcvY, float angle) const;

		// Shared behaviour state variables
		float reqBallDirMidY;
		Vec2f reqBallDirLeftKb;
		Vec2f reqBallDirRightKb;
		Vec2f reqBallDirLeftDb;
		Vec2f reqBallDirRightDb;
		bool haveBallTarget;
		float ballTargetDist;
		float ballTargetAngle;
		Vec2f ballToTargetDir;
		float ballToTargetDist;
		float ballToTargetAngle;
		float ballToTargetAngleOffsetKick;

	private:
		// Config parameters
		WAKConfig& config;

		// Sensor variables
		const SensorVars& SV;

		// ROS interface
		const WAKRosInterface& RI;

		// Behaviour manager
		WAKBehManager& BM;
	};
}

#endif
// EOF