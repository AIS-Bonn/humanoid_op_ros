// Walk and kick game state: Default ball handling
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef GAME_DEFAULT_BALL_HANDLING_H
#define GAME_DEFAULT_BALL_HANDLING_H

// Includes
#include <walk_and_kick/wak_game_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GameDefaultBallHandling
	* 
	* @brief A walk and kick game state that implements default ball handling for playing soccer.
	**/
	class GameDefaultBallHandling : public WAKGameState
	{
	public:
		// Constructor
		GameDefaultBallHandling(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(GameVars& GV, const GameVars& lastGV, bool justActivated);

		// Ball target calculation using a local ball position
		void calcCompassBallTarget(GameVars& GV, const Vec2f& ballDir, int goalSign) const;

		// Ball target calculation using a global ball pose
		void calcPoseBallTarget(GameVars& GV, const Vec2f& ballPose, int goalSign) const;
		bool ballInCorner(const Vec2f& ballPoseGS, float postY, int cornerSign) const;

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);
	};
}

#endif
// EOF