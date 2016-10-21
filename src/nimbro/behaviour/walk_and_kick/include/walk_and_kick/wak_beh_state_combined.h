// Walk and kick: Combined behaviour state
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef WAK_BEH_STATE_COMBINED_H
#define WAK_BEH_STATE_COMBINED_H

// Includes
#include <walk_and_kick/wak_beh_state.h>
#include <boost/type_traits/is_virtual_base_of.hpp>
#include <boost/utility/enable_if.hpp>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class WAKBehStateCombined
	* 
	* @brief A walk and kick behaviour state that combines two others (which are assumed to operate independently).
	**/
	template<class BS1, class BS2>
	class WAKBehStateCombined : public boost::enable_if<boost::is_virtual_base_of<WAKBehState, BS1>, BS1>::type, public boost::enable_if<boost::is_virtual_base_of<WAKBehState, BS2>, BS2>::type
	{
	public:
		// Constructor
		WAKBehStateCombined(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID) : WAKBehState(config, SV, WBS, WGS, ID), BS1(config, SV, WBS, WGS, ID), BS2(config, SV, WBS, WGS, ID) {}
		virtual ~WAKBehStateCombined() {}

		// Execute function
		virtual void execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated) { BS1::execute(AV, lastAV, justActivated); BS2::execute(AV, lastAV, justActivated); }

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive) { BS1::handleActivation(nowActive); BS2::handleActivation(nowActive); }
	};
}

#endif
// EOF