// Feedback gait trajectory generation common
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_TRAJECTORY_COMMON_H
#define FEED_TRAJECTORY_COMMON_H

// Includes
#include <feed_gait/feed_common.h>
#include <feed_gait/model/feed_model_common.h>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @struct TrajCommand
	*
	* @brief Trajectory generation command struct.
	**/
	struct TrajCommand : public ActionsCommand
	{
		// Constructor and reset
		explicit TrajCommand(double fusedPitchN) : ActionsCommand(fusedPitchN) { resetMembers(); }
		virtual void reset(double fusedPitchN) override { ActionsCommand::reset(fusedPitchN); resetMembers(); }

		// Data members
		Vec3 gcv;      //!< @brief Desired dimensionless walking velocity in the x, y and yaw axes, relative to the body-fixed axes untilted relative to the true ground (i.e. the true S plane)
		Vec3 gcvLFAcc; //!< @brief Dimensionless low frequency walking acceleration calculated from #gcv in the x, y and yaw axes, relative to the body-fixed axes untilted relative to the true ground (i.e. the true S plane)

	private:
		// Reset members function
		void resetMembers()
		{
			gcv.setZero();
			gcvLFAcc.setZero();
		}
	};
}

#endif
// EOF