// Gait command structure for the generic gait motion module
// File: gait_command.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef GAIT_COMMAND_H
#define GAIT_COMMAND_H

// Gait namespace
namespace gait
{
	/**
	* @struct GaitCommand
	*
	* @brief Gait command data structure.
	**/
	struct GaitCommand
	{
		//! Default constructor
		GaitCommand() { reset(); }

		//! Reset function
		void reset(bool shouldWalk = false)
		{
			linVelX = 0.0;
			linVelY = 0.0;
			angVelZ = 0.0;
			walk = shouldWalk;
		}

		// Gait command velocity vector
		float linVelX;         //!< Commanded linear x-velocity
		float linVelY;         //!< Commanded linear y-velocity
		float angVelZ;         //!< Commanded angular z-velocity

		// Flags
		bool walk;             //!< Flag whether to walk or not
	};
}

#endif /* GAIT_COMMAND_H */
// EOF