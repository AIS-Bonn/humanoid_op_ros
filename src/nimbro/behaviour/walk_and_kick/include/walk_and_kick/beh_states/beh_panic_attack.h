// Walk and kick behaviour state: Panic attack
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef BEH_PANIC_ATTACK_H
#define BEH_PANIC_ATTACK_H

// Includes
#include <walk_and_kick/wak_beh_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class BehPanicAttack
	* 
	* @brief A walk and kick behaviour state that induces a panic attack and stops the robot still.
	**/
	class BehPanicAttack : public WAKBehState
	{
	public:
		// Constructor
		BehPanicAttack(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);
	};
}

#endif
// EOF