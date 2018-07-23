// Feedback gait utilities
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/feed_utils.h>
#include <rc_utils/math_funcs.h>

// Namespaces
using namespace feed_gait;

//
// PoseCommandBlender class
//

// Internal reset function
void PoseCommandBlender::resetInternal(double factor)
{
	// Reset all member variables
	m_blending = false;
	m_currentFactor = m_initialFactor = m_targetFactor = rc_utils::coerce(factor, 0.0, 1.0);
	m_currentPhase = m_targetPhase = 0.0;
}

// Set target function
void PoseCommandBlender::setTarget(double factor, double phase)
{
	// Coerce the target blend factor to the required range
	factor = rc_utils::coerce(factor, 0.0, 1.0);

	// Instantaneously update the current blend factor if required
	if(phase <= 0.0 || factor == m_currentFactor)
	{
		resetInternal(factor);
		return;
	}

	// Initialise all member variables for the new target
	m_blending = true;
	m_initialFactor = m_currentFactor;
	m_targetFactor = factor;
	m_currentPhase = 0.0;
	m_targetPhase = phase;
}

// Set target scaled function
void PoseCommandBlender::setTargetScaled(double factor, double fullPhase)
{
	// Coerce the target blend factor to the required range
	factor = rc_utils::coerce(factor, 0.0, 1.0);

	// Scale the phase argument as appropriate
	double phase = fabs(factor - m_currentFactor) * fullPhase;
	setTarget(factor, phase);
}

// Update function
void PoseCommandBlender::update(double phaseInc)
{
	// Nothing to update if no blend is currently in progress
	if(!m_blending) return;

	// Update the current blend phase
	m_currentPhase += rc_utils::coerceMin(phaseInc, 0.0);

	// Update all member variables as appropriate
	if(m_currentPhase < 0.0) // Should never happen...
	{
		m_currentPhase = 0.0;
		m_currentFactor = m_initialFactor;
	}
	else if(m_currentPhase >= m_targetPhase) // Blend finished...
	{
		resetInternal(m_targetFactor);
	}
	else // Blend in process...
	{
		double u = sin(M_PI_2 * m_currentPhase / m_targetPhase);
		m_currentFactor = m_initialFactor + (u*u)*(m_targetFactor - m_initialFactor);
	}
}

// Blend function (in-place)
void PoseCommandBlender::blend(PoseCommand& cmd, const PoseCommand& cmdBlend) const
{
	// Don't do anything if no blending is required
	if(!requireBlend()) return;

	// Get the blend coefficients
	double b = rc_utils::coerce(m_currentFactor, 0.0, 1.0);
	double B = 1.0 - b;

	// Blend the positions
	std::size_t numPos = std::min(cmd.pos.size(), cmdBlend.pos.size());
	for(std::size_t i = 0; i < numPos; i++)
		cmd.pos[i] = B*cmd.pos[i] + b*cmdBlend.pos[i];

	// Blend the efforts
	std::size_t numEffort = std::min(cmd.effort.size(), cmdBlend.effort.size());
	for(std::size_t i = 0; i < numEffort; i++)
		cmd.effort[i] = B*cmd.effort[i] + b*cmdBlend.effort[i];

	// Blend the support coefficients
	for(int l = 0; l < hk::NUM_LR; l++)
		cmd.suppCoeff[l] = B*cmd.suppCoeff[l] + b*cmdBlend.suppCoeff[l];
}
// EOF