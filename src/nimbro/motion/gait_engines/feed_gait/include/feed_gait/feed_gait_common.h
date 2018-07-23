// Feedback gait common
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_GAIT_COMMON_H
#define FEED_GAIT_COMMON_H

// Includes
#include <feed_gait/feed_common.h>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @struct GaitInput
	*
	* @brief Gait input struct.
	**/
	struct GaitInput : public CommonInput
	{
		// Constructor and reset
		GaitInput() : CommonInput() { resetMembers(); }
		virtual void reset() override { CommonInput::reset(); resetMembers(); }

		// Data members
		Vec3 inputGcv;

	private:
		// Reset members function
		void resetMembers() { inputGcv.setZero(); }
	};

	/**
	* @struct TimingOutput
	*
	* @brief Timing output struct.
	**/
	struct TimingOutput
	{
		// Constructor and reset
		TimingOutput() { reset(); }
		void reset()
		{
			gaitFrequency = 0.0;
			gaitPhaseInc = gaitPhaseIncMax = 0.0;
			timeToStep = 0.0;
		}

		// Data members
		double gaitFrequency;
		double gaitPhaseInc;
		double gaitPhaseIncMax;
		double timeToStep;
	};

	/**
	* @struct StepSizeOutput
	*
	* @brief Step size output struct.
	**/
	struct StepSizeOutput
	{
		// Constructor and reset
		StepSizeOutput() { reset(); }
		void reset() { targetGcv.setZero(); }

		// Data members
		Vec3 targetGcv;
	};

	/**
	* @struct GcvCommand
	*
	* @brief Gait command vector command struct.
	**/
	struct GcvCommand
	{
		// Constructor and reset
		GcvCommand() { reset(); }
		void reset()
		{
			gcvLF.setZero();
			gcvHF.setZero();
			gcvEOS.setZero();
		}

		// Get functions
		Vec3 totalGcv() const { return gcvLF + gcvHF + gcvEOS; }

		// Data members
		Vec3 gcvLF;
		Vec3 gcvHF;
		Vec3 gcvEOS;
	};
}

#endif
// EOF