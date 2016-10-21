// Walk and kick game state: Positioning
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef GAME_POSITIONING_H
#define GAME_POSITIONING_H

// Includes
#include <walk_and_kick/wak_game_state.h>
#include <rc_utils/ros_time.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GamePositioning
	* 
	* @brief A walk and kick game state that forces the robot to the positioning behaviour state.
	**/
	class GamePositioning : public WAKGameState
	{
	public:
		// Constructor
		GamePositioning(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(GameVars& GV, const GameVars& lastGV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);

	private:
		// Set the special target pose to one that quickly makes the robot legally positioned
		void setTargetToQuickLegalPose();

		// Special target pose
		DynamicTargetPose m_specialTarget;
	};
}

#endif
// EOF