// Walk and kick gaze behaviour state: Look around
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef GAZE_BEH_LOOK_AROUND_H
#define GAZE_BEH_LOOK_AROUND_H

// Includes
#include <walk_and_kick/wak_beh_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GazeBehLookAround
	* 
	* @brief A walk and kick gaze behaviour state that looks around.
	**/
	class GazeBehLookAround : public virtual WAKBehState
	{
	public:
		// Constructor
		GazeBehLookAround(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID);
		virtual ~GazeBehLookAround() {}

		// Execute function
		virtual void execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);

		// Reset functions
		void resetGazeSpline();

	private:
		// State variables
		Vec2f m_gazeInit;
		float m_gazeMag;
		int m_gazeTargetID;   // -1 = Look right, 0 = Look down, +1 = Look left
		int m_gazeTargetDirn; // -1 = CCW, +1 = CW
		TrapVelSpline2D m_gazeSpline;
	};
}


#endif
// EOF