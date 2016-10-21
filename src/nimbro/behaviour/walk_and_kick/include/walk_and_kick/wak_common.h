// Walk and kick: Common include file
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef WAK_COMMON_H
#define WAK_COMMON_H

// Includes
#include <walk_and_kick/wak_plot.h>
#include <ros/console.h>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>
#include <string>

// Defines
#define TINC            0.040f    // System iteration time (relevant for instance for velocities and accelerations)
#define LED_PERIOD      1.0f      // Time period for the regular updates to the LED state
#define INVALID_DIST    100000.0f // Used to signify an invalid distance

// Macros
#define TO_COUNT(time)  ((int)((time)/(TINC) + 0.5f)) // This assumes for the rounding that time is always positive!

// Walk and kick namespace
namespace walk_and_kick
{
	// Kickoff type enumeration
	enum KickoffType
	{
		KT_UNKNOWN = 0,  // Unknown kickoff type
		KT_DEFENDING,    // We are defending a kickoff
		KT_ATTACKING,    // We are taking a kickoff
		KT_DROPBALL,     // We are playing a drop ball
		KT_MANUAL,       // Not a kickoff (manual start of the robot)
		KT_COUNT,
		KT_DEFAULT = KT_MANUAL
	};
	const std::string KickoffTypeName[KT_COUNT] = {
		"Unknown",
		"Defending",
		"Attacking",
		"DropBall",
		"Manual"
	};
	inline bool kickoffTypeValid(int type) { return (type > KT_UNKNOWN && type < KT_COUNT); }
	inline bool kickoffTypeValid(KickoffType type) { return kickoffTypeValid((int) type); }
	inline const std::string& kickoffTypeName(KickoffType type) { if(kickoffTypeValid(type)) return KickoffTypeName[type]; else return KickoffTypeName[KT_UNKNOWN]; }

	// Button state enumeration
	enum ButtonState
	{
		BTN_UNKNOWN = 0,  // Unknown button state
		BTN_STOP,         // Stopped
		BTN_PLAY,         // Play soccer
		BTN_GOALIE,       // Play soccer and force being a goalie
		BTN_POS,          // Perform auto-positioning
		BTN_COUNT
	};
	const std::string ButtonStateName[BTN_COUNT] = {
		"Unknown Mode",
		"Halt Mode",
		"Play Mode",
		"Goalie Mode",
		"Positioning Mode"
	};
	inline bool buttonStateValid(int state) { return (state > BTN_UNKNOWN && state < BTN_COUNT); }
	inline bool buttonStateValid(ButtonState state) { return buttonStateValid((int) state); }
	inline const std::string& buttonStateName(ButtonState state) { if(buttonStateValid(state)) return ButtonStateName[state]; else return ButtonStateName[BTN_UNKNOWN]; }

	// Game command enumeration
	enum GameCommand
	{
		CMD_UNKNOWN = 0,  // Unknown game command
		CMD_STOP,         // The robot should be stopped no matter what
		CMD_POS,          // The robot should perform positioning no matter what
		CMD_PLAY,         // The robot should play soccer in the appropriate mode, role and state
		CMD_COUNT
	};
	const std::string GameCommandName[CMD_COUNT] = {
		"UnknownCmd",
		"Stop",
		"Positioning",
		"Play"
	};
	inline bool gameCommandValid(int command) { return (command > CMD_UNKNOWN && command < CMD_COUNT); }
	inline bool gameCommandValid(GameCommand command) { return gameCommandValid((int) command); }
	inline const std::string& gameCommandName(GameCommand command) { if(gameCommandValid(command)) return GameCommandName[command]; else return GameCommandName[CMD_UNKNOWN]; }

	// Game role enumeration
	enum GameRole
	{
		ROLE_UNKNOWN = 0,          // Unknown game role
		ROLE_FIELDPLAYER,          // Field player for normal soccer gameplay
		ROLE_GOALIE,               // Goalie for normal soccer gameplay
		ROLE_COUNT,
		ROLE_DEFAULT = ROLE_FIELDPLAYER
	};
	const std::string GameRoleName[ROLE_COUNT] = {
		"UnknownRole",
		"FieldPlayer",
		"Goalie",
	};
	inline bool gameRoleValid(int role) { return (role > ROLE_UNKNOWN && role < ROLE_COUNT); }
	inline bool gameRoleValid(GameRole role) { return gameRoleValid((int) role); }
	inline const std::string& gameRoleName(GameRole role) { if(gameRoleValid(role)) return GameRoleName[role]; else return GameRoleName[ROLE_UNKNOWN]; }
	inline bool gameRoleIsGoalie(GameRole role) { return (role == ROLE_GOALIE); }
	inline bool gameRoleIsFieldPlayer(GameRole role) { return (gameRoleValid(role) && !gameRoleIsGoalie(role)); }

	// Play state enumeration
	enum PlayState
	{
		PS_UNKNOWN = 0,  // Unknown play state
		PS_STOP,         // Generic stopped state
		PS_TIMEOUT,      // State during a timeout
		PS_READY,        // Getting ready for the kickoff (e.g. by auto-positioning)
		PS_SET,          // Stand in place and wait for the signal to begin play
		PS_BEGIN_PLAY,   // Game play has started but you are not allowed to move yet
		PS_PLAY,         // Normal game play
		PS_COUNT
	};
	const std::string PlayStateName[PS_COUNT] = {
		"UnknownPlayState",
		"Stop",
		"Timeout",
		"Ready",
		"Set",
		"BeginPlay",
		"Play"
	};
	inline bool playStateValid(int state) { return (state > PS_UNKNOWN && state < PS_COUNT); }
	inline bool playStateValid(PlayState state) { return playStateValid((int) state); }
	inline const std::string& playStateName(PlayState state) { if(playStateValid(state)) return PlayStateName[state]; else return PlayStateName[PS_UNKNOWN]; }

	// Dive direction enumeration
	enum DiveDirection
	{
		DD_NONE = 0,
		DD_LEFT,
		DD_RIGHT,
		DD_SIT,
		DD_COUNT
	};
	const std::string DiveDirectionName[DD_COUNT] = {
		"No Dive",
		"Left Dive",
		"Right Dive",
		"Sit Dive"
	};
	inline bool diveDirectionValid(int dirn) { return (dirn >= DD_NONE && dirn < DD_COUNT); }
	inline bool diveDirectionValid(DiveDirection dirn) { return diveDirectionValid((int) dirn); }
	inline const std::string& diveDirectionName(DiveDirection dirn) { if(diveDirectionValid(dirn)) return DiveDirectionName[dirn]; else return DiveDirectionName[DD_NONE]; }

	// Typedefs
	typedef Eigen::Vector2f Vec2f;
	typedef Eigen::Vector3f Vec3f;
	typedef std::vector<Vec2f, Eigen::aligned_allocator<Vec2f> > Vec2fArray;
	typedef std::vector<Vec3f, Eigen::aligned_allocator<Vec3f> > Vec3fArray;
	typedef long long int cycle_t; // Note: This should be signed and not unsigned!

	// Goal post struct
	struct GoalPost
	{
		GoalPost() : vec(0.0f, 0.0f), conf(0.0f) {}
		Vec2f vec;
		float conf;
	};
	typedef std::vector<GoalPost> GoalPostList;

	// Obstacle struct
	struct Obstacle
	{
		Obstacle() { reset(); }
		void reset() { vec.setZero(); dist = 0.0f; conf = 0.0f; id = -1; }
		bool valid() const { return (conf > 0.0f && id >= 0); }
		void setVec(float x, float y) { vec << x, y; dist = vec.norm(); }
		Vec2f vec;
		float dist;
		float conf;
		int id;
	};
	typedef std::vector<Obstacle> ObstacleList;
}

#endif
// EOF