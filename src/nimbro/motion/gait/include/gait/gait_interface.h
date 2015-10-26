// Interface classes for data exchange between the generic gait motion module and gait engines
// File: gait_interface.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef GAIT_INTERFACE_H
#define GAIT_INTERFACE_H

// Includes
#include <gait/gait_common.h>
#include <gait/gait_command.h>

// Gait namespace
namespace gait
{
	/**
	* @struct GaitEngineInput
	*
	* @brief Data struct for passing gait engine input data from the
	* generic gait motion module to the gait engine that it manages.
	**/
	struct GaitEngineInput
	{
		//! Default constructor
		GaitEngineInput() { reset(); }

		//! Reset function
		void reset()
		{
			for(int i = 0; i < NUM_JOINTS; i++)
				jointPos[i] = 0.0;
			timestamp = 0.0;
			nominaldT = 0.0;
			truedT = 0.0;
			gaitCmd.reset();
			motionPending = false;
			motionID = MID_NONE;
			motionStance = STANCE_DEFAULT;
			motionAdjustLeftFoot = false;
			motionAdjustRightFoot = false;
		}

		// Joint states
		double jointPos[NUM_JOINTS]; //!< @brief Current measured position of each joint (indexed by the `JointID` enum, in `rad`).

		// System parameters
		double timestamp;            //!< @brief The current time in seconds (guaranteed to be monotonic increasing)
		double nominaldT;            //!< @brief The nominal time between calls to the gait engine's `step()` function.
		double truedT;               //!< @brief The true time since the last call to the gait engine's `step()` function. This value is coerced to avoid spikes.

		// Gait command
		GaitCommand gaitCmd;         //!< @brief Gait command (e.g. desired walking velocity and so on).

		// Motion parameters
		bool motionPending;          //!< @brief Boolean flag whether a motion is pending.
		MotionID motionID;           //!< @brief The ID of the motion that is pending.
		MotionStance motionStance;   //!< @brief The stopping stance required for the playing of the pending motion.
		bool motionAdjustLeftFoot;   //!< @brief Boolean flag whether the left foot should be used to adjust the stopping stance.
		bool motionAdjustRightFoot;  //!< @brief Boolean flag whether the right foot should be used to adjust the stopping stance.
	};

	/**
	* @struct GaitEngineOutput
	*
	* @brief Data struct for passing gait engine output data from the
	* gait engine to the generic gait motion module that manages it.
	**/
	struct GaitEngineOutput
	{
		//! Default constructor
		GaitEngineOutput() { reset(); }

		//! Reset function
		void reset()
		{
			for(int i = 0; i < NUM_JOINTS; i++)
			{
				jointCmd[i] = 0.0;
				jointEffort[i] = 0.0;
			}
			useRawJointCmds = false;
			walking = false;
			supportCoeffLeftLeg = 0.0;
			supportCoeffRightLeg = 0.0;
			for(int i = 0; i < 3; i++)
			{
				odomPosition[i] = 0.0;
				odomOrientation[i+1] = 0.0;
			}
			odomOrientation[0] = 1.0;
		}

		// Joint commands
		double jointCmd[NUM_JOINTS];     //!< @brief Commanded position for each joint (indexed by the `JointID` enum, in `rad`).
		double jointEffort[NUM_JOINTS];  //!< @brief Commanded joint effort (indexed by the `JointID` enum, in the range `[0,1]`).
		bool   useRawJointCmds;          //!< @brief Apply the joint commands directly to the hardware, without using compensation or actuator controller(s) in-between.

		// Status flags
		bool walking;                    //!< @brief Flag specifying whether the gait is currently active and walking (`true`) or halted (`false`).

		// Support coefficients
		double supportCoeffLeftLeg;      //!< @brief Current support coefficient of the left leg.
		double supportCoeffRightLeg;     //!< @brief Current support coefficient of the right leg.

		// Robot odometry transform
		double odomPosition[3];          //!< @brief Position `(x,y,z)` of the robot's body-fixed base transform (centred at the robot's centre of mass) in global odometry coordinates.
		double odomOrientation[4];       //!< @brief Orientation `(w,x,y,z)` of the robot's body-fixed base transform (centred at the robot's centre of mass) relative to the global odometry frame.
	};
}

#endif /* GAIT_INTERFACE_H */
// EOF