// Feedback gait model base class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/model/feed_model_base.h>

// Namespaces
using namespace feed_gait;

//
// FeedModelBase class
//

// Call update wrapper function
void FeedModelBase::callUpdate(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput)
{
	// Initialise the class with the first received model inputs
	if(!m_inited)
	{
		reset(phaseInfo, modelInput);
		m_inited = true;
	}

	// Call the virtual update function
	update(phaseInfo, modelInput);

	// Mask the actions command flags to just the ones that exist
	m_out.useActions.applyMask();

	// Validate the generated model output for non-finite values
	if(!m_out.validate())
		ROS_WARN_THROTTLE(0.5, "Model output contains non-finite values => Ignoring the respective parts!");
}
// EOF