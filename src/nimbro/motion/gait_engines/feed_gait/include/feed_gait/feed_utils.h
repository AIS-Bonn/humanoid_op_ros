// Feedback gait utilities
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_UTILS_H
#define FEED_UTILS_H

// Includes
#include <feed_gait/feed_common.h>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @class PoseCommandBlender
	*
	* @brief Pose command blender class.
	**/
	class PoseCommandBlender
	{
	public:
		// Constructor
		PoseCommandBlender() { reset(0.0); }

		// Reset function
		void reset(double factor) { resetInternal(factor); resetSource(); }
		void reset(double factor, const PoseCommand& cmdSource) { resetInternal(factor); setSource(cmdSource); }

		// Set functions
		void setTarget(double factor, double phase);
		void setTargetScaled(double factor, double fullPhase); // The argument fullPhase is how much phase should be used for a full blend 0.0 <-> 1.0. This is scaled as appropriate for smaller blends.
		void update(double phaseInc);

		// Get functions
		bool blending() const { return m_blending; }
		bool requireBlend() const { return (m_currentFactor > 0.0); }
		double currentFactor() const { return m_currentFactor; }
		double currentPhase() const { return m_currentPhase; }
		double targetFactor() const { return m_targetFactor; }
		double targetPhase() const { return m_targetPhase; }
		PoseCommand source() const { return m_cmdSource; }

		// Blend functions
		void blend(PoseCommand& cmd, const PoseCommand& cmdBlend) const;
		void blend(const PoseCommand& cmd, const PoseCommand& cmdBlend, PoseCommand& cmdOut) const { cmdOut = cmd; blend(cmdOut, cmdBlend); }

		// Blend functions with saved source pose command
		void resetSource() { m_cmdSource.reset(); }
		void setSource(const PoseCommand& cmdSource) { m_cmdSource = cmdSource; }
		void blendSource(const PoseCommand& cmdBlend, PoseCommand& cmdOut) const { cmdOut = m_cmdSource; blend(cmdOut, cmdBlend); }

	private:
		// Internal reset function
		void resetInternal(double factor);

		// Blending flag
		bool m_blending;

		// Blending target variables
		double m_initialFactor;
		double m_targetFactor;
		double m_targetPhase;

		// Blending state variables
		double m_currentFactor;
		double m_currentPhase;

		// Source pose command
		PoseCommand m_cmdSource;
	};
}

#endif
// EOF