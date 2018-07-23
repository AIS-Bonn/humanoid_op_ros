// Feedback gait trivial model
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/model/trivial/feed_trivial_model.h>

// Namespaces
using namespace feed_gait;
using namespace feed_gait::trivial_model;

//
// FeedTrivialModel class
//

// Reset members function
void FeedTrivialModel::resetMembers(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput)
{
}

// Update function
void FeedTrivialModel::update(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput)
{
	// Reset the model output
	m_out.reset();

	// Do nothing if the model is not enabled
	if(!tmconfig.enable())
	{
		resetMembers(phaseInfo, modelInput);
		return;
	}

	// Calculate the required timing command
	m_out.timingCmd.reset();
	m_out.useTiming = false;

	// Calculate the required step size command
	m_out.stepSizeCmd.X.set(0.0, 0.0, tmconfig.gcvX());
	m_out.stepSizeCmd.Y.set(0.0, 0.0, tmconfig.gcvY());
	m_out.stepSizeCmd.Z.set(0.0, 0.0, tmconfig.gcvZ());
	m_out.useStepSize = true;

	// Calculate the required actions command
	m_out.actionsCmd.reset(modelInput.trajInfo.fusedPitchN);
	m_out.actionsCmd.fusedPitchS = tmconfig.fusedPitchS();
	m_out.actionsCmd.fusedRollS = tmconfig.fusedRollS();
	m_out.actionsCmd.footTiltCts.set(tmconfig.footTiltCtsGamma(), tmconfig.footTiltCtsAlpha());
	m_out.actionsCmd.footTiltSupp.set(tmconfig.footTiltSuppGamma(), tmconfig.footTiltSuppAlpha());
	m_out.actionsCmd.swingOut.set(tmconfig.swingOutGamma(), tmconfig.swingOutAlpha());
	m_out.actionsCmd.leanTilt.set(tmconfig.leanTiltGamma(), tmconfig.leanTiltAlpha());
	m_out.actionsCmd.hipShift << tmconfig.hipShiftX(), tmconfig.hipShiftY();
	m_out.actionsCmd.hipHeightMax = tmconfig.hipHeightMax();
	m_out.actionsCmd.armTilt.set(tmconfig.armTiltGamma(), tmconfig.armTiltAlpha());
	m_out.useActions = ACFlag::ALL;

	// Disable certain model outputs if the configuration parameters say so
	if(!tmconfig.useTiming())   m_out.useTiming = false;
	if(!tmconfig.useStepSize()) m_out.useStepSize = false;
	if(!tmconfig.useActions())  m_out.useActions = ACFlag::NONE;
}
// EOF