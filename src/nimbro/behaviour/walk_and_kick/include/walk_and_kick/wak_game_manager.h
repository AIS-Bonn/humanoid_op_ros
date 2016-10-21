// Walk and kick: Class for management of the game states
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef WAK_GAME_MANAGER_H
#define WAK_GAME_MANAGER_H

// Includes - General
#include <walk_and_kick/wak_common.h>
#include <walk_and_kick/wak_config.h>
#include <walk_and_kick/wak_game_vars.h>
#include <walk_and_kick/wak_sensor_vars.h>
#include <walk_and_kick/wak_actuator_vars.h>
#include <walk_and_kick/wak_ros_interface.h>
#include <walk_and_kick/wak_game_shared.h>
#include <walk_and_kick/wak_game_state.h>

// Includes - Game states
#include <walk_and_kick/game_states/game_unknown_state.h>
#include <walk_and_kick/game_states/game_stopped.h>
#include <walk_and_kick/game_states/game_panic_attack.h>
#include <walk_and_kick/game_states/game_positioning.h>
#include <walk_and_kick/game_states/game_gaze_for_ball.h>
#include <walk_and_kick/game_states/game_wait_for_ball_in_play.h>
#include <walk_and_kick/game_states/game_default_ball_handling.h>
#include <walk_and_kick/game_states/game_penalty_ball_handling.h>
#include <walk_and_kick/game_states/game_default_goalie.h>
#include <walk_and_kick/game_states/game_penalty_goalie.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class WAKGameManager
	* 
	* @brief A class that manages and executes the walk and kick game states.
	**/
	class WAKGameManager
	{
	public:
		// Game state ID enumeration
		enum GameStateID
		{
			GS_UNKNOWN = 0,
			GS_STOPPED,               // Abbreviation: STP
			GS_PANIC_ATTACK,          // Abbreviation: PA
			GS_POSITIONING,           // Abbreviation: POS
			GS_GAZE_FOR_BALL,         // Abbreviation: GFB
			GS_WAIT_FOR_BALL_IN_PLAY, // Abbreviation: WFBIP
			GS_DEFAULT_BALL_HANDLING, // Abbreviation: DBH
			GS_PENALTY_BALL_HANDLING, // Abbreviation: PBH
			GS_DEFAULT_GOALIE,        // Abbreviation: DG
			GS_PENALTY_GOALIE,        // Abbreviation: PG
			GS_COUNT
		};
		static bool gameStateValid(int GSI) { return (GSI > GS_UNKNOWN && GSI < GS_COUNT); }
		static bool gameStateValid(GameStateID GSI) { return gameStateValid((int) GSI); }
		static const std::string& gameStateName(GameStateID GSI) { return gameStateName((int) GSI); }
		static const std::string& gameStateName(int GSI) { if(gameStateValid(GSI)) return GameStateName[GSI]; else return GameStateName[GS_UNKNOWN]; }
	private:
		static const std::string GameStateName[GS_COUNT];

	public:
		// Constructor
		WAKGameManager(WAKConfig& config, const SensorVars& SV, const WAKRosInterface& RI);
		virtual ~WAKGameManager();

		// Reset functions
		void reset();             // Resets all normal variables, the state machine and all updated variables
		void resetVars();         // Resets all normal variables
		void resetStateMachine(); // Resets all normal variables and the state machine

		// Game state registration function
		void registerState(WAKGameState* state, int ID, const std::string& name);

		// Constant access to game states
		const WAKGameState* currentState() const { return m_curState; }
		const GameStopped* stoppedState() const { return Stopped; }

		// Get functions
		const WAKGameShared& getWGS() const { return WGS; }
		const GameVars& GV() const { return m_GV; }
		const GameVars& lastGV() const { return m_lastGV; }
		cycle_t wakCycle() const { return m_wakCycle; }
		cycle_t stateCycle() const { return m_stateCycle; }

		// Update function
		void updateManager(cycle_t wakCycle);

		// Execute function
		void execute();

	private:
		// State decision function
		WAKGameState* decideState();

		// Config parameters
		WAKConfig& config;

		// Sensor variables
		const SensorVars& SV;

		// ROS interface
		const WAKRosInterface& RI;

		// Shared game variables
		WAKGameShared WGS;

		// Game states
		GameUnknownState* UnknownState;
		GameStopped* Stopped;
		GamePanicAttack* PanicAttack;
		GamePositioning* Positioning;
		GameGazeForBall* GazeForBall;
		GameWaitForBallInPlay* WaitForBallInPlay;
		GameDefaultBallHandling* DefaultBallHandling;
		GamePenaltyBallHandling* PenaltyBallHandling;
		GameDefaultGoalie* DefaultGoalie;
		GamePenaltyGoalie* PenaltyGoalie;

		// Current game state
		WAKGameState* m_curState;

		// Game state retrieval
		WAKGameState* stateForID(int ID);
		std::string stateNameForID(int ID);
		std::map<int, WAKGameState*> m_stateMap;
		std::map<int, std::string> m_stateNameMap;

		// Game variable outputs
		GameVars m_GV;
		GameVars m_lastGV;

		// Decide state variables
		TheWorm m_poseLegalWorm;
		bool m_poseLegal;

		// Cycle numbers
		cycle_t m_wakCycle;
		cycle_t m_stateCycle;

		// Miscellaneous variables
		PlayState m_lastPlayState;
		bool m_stateIsNew;

		// Friend classes
		friend class WalkAndKick;
		friend class WAKGameShared;
	};
}

#endif
// EOF
