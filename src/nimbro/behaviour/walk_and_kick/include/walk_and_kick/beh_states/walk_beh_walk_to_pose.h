// Walk and kick walk behaviour state: Walk to pose
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef WALK_BEH_WALK_TO_POSE_H
#define WALK_BEH_WALK_TO_POSE_H

// Includes
#include <walk_and_kick/wak_beh_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class WalkBehWalkToPose
	* 
	* @brief A walk and kick walk behaviour state that makes the robot walk to a desired target pose on the field.
	**/
	class WalkBehWalkToPose : public virtual WAKBehState
	{
	public:
		// Constructor
		WalkBehWalkToPose(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID);
		virtual ~WalkBehWalkToPose() {}

		// Execute function
		virtual void execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);

	private:
		// State variables
		bool m_arrived;
		TheWorm m_arrivedWorm;
	};
}

#endif
// EOF