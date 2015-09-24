// Base class for all gait engines
// File: gait_engine.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef GAIT_ENGINE_H
#define GAIT_ENGINE_H

// Includes
#include <gait/gait_common.h>
#include <gait/gait_interface.h>

// Forwards declarations
namespace robotcontrol
{
	class RobotModel;
}

// Gait namespace
namespace gait
{
	// Class forward declarations
	class Gait;
	
	/**
	* @class GaitEngine
	*
	* @brief Base implementation of a gait engine, made to work with the `Gait` motion module.
	**/
	class GaitEngine
	{
	public:
		//! Default constructor
		GaitEngine() : model(NULL), haltJointCmd(), haltJointEffort(), haltUseRawJointCmds(false), m_gait(NULL), m_posX(0.0), m_posY(0.0), m_rotZ(0.0)
		{
			in.reset();
			out.reset();
			GaitEngine::updateHaltPose();
			GaitEngine::updateOdometry();
		}

		//! Default destructor
		virtual ~GaitEngine() {}

		/**
		* @brief Reset the gait engine.
		*
		* This function is used by the `Gait` class, in conjunction with the `resetBase()` function,
		* to reset the GaitEngine object in terms of both its derived and base components respectively.
		* This function must be able to clean up and reset the gait engine instance, no matter what
		* state it is currently in.
		**/
		virtual void reset() {}

		/**
		* @brief Update the halt pose desired by the gait engine.
		*
		* This function should update the halt pose of the gait engine if/as required. The halt pose is
		* stored in three variables, namely #haltJointCmd, #haltJointEffort and #haltUseRawJointCmds.
		* This should be the robot pose from which the gait engine is nominally intended to be started
		* and stopped from. The halt pose should normally be relatively constant during execution, but
		* may for example depend on configuration parameters, and so is allowed to dynamically change.
		*
		* This function is intended for use by the gait engine itself as well, such as for example at
		* the beginning of the derived `step()` function override. Make no assumptions about when this
		* function is called externally.
		**/
		virtual void updateHaltPose();

		/**
		* @brief Main step function of the gait engine.
		*
		* The step function is called in every execution cycle that the gait is required to be active.
		* The command inputs for the gait engine can be retrieved from the `in` class member, which
		* ideally should only be read from, and the outputs of the gait engine should be written into
		* the `out` class member. Ideally *all* members of the `out` struct should be written to from
		* within the `step()` function, as the members may contain arbitrary values until written to.
		* The latest robot state information can be read from the `model` class member, which is a
		* const pointer to the required RobotModel object. The pointer is guaranteed to be valid before
		* the `step()` function is called for the first time.
		**/
		virtual void step();

		/**
		* @brief Set the CoM odometry to a particular 2D position and orientation.
		*
		* Specifies the required `(x,y)` position and yaw rotation for the CoM odometry. How exactly the
		* yaw parameter is interpreted in the light of additional pitch and roll rotations is up to the
		* implementation. It is recommended that the parameter is treated as fused yaw. Ideally, after
		* calling this function the next retrieval of the robot odometry should reveal exactly @p posX
		* and @p posY in the position vector (see `GaitEngineOutput::odomPosition`). If you override this
		* function then you need to override `updateOdometry` too.
		**/
		virtual void setOdometry(double posX, double posY, double rotZ);

		/**
		* @brief Force an update of the CoM odometry in terms of 3D position and orientation.
		*
		* This function should update the `GaitEngineOutput::odomPosition` and `GaitEngineOutput::odomOrientation`
		* members of the @c out member of the `GaitEngine` class. Most of the time these two data fields
		* will already be up-to-date anyway, but after calling this function the caller should be able
		* to take for granted that the odometry stored in @c out is up-to-date, and some valid value.
		* The default implementation simply writes the last set odometry (`setOdometry`) into the fields.
		**/
		virtual void updateOdometry();

		// Gait engine data interface structs
		GaitEngineInput in;   //!< Gait engine input data struct.
		GaitEngineOutput out; //!< Gait engine output data struct.

	protected:
		/**
		* @brief Pointer to the RobotModel object to use for retrieving state information in each step.
		*
		* This parameter is guaranteed to be set by the `Gait` class prior to any other function being
		* called. For obvious reasons it is heavily discouraged for a gait engine to write to this variable.
		**/
		const robotcontrol::RobotModel* model;

		/**
		* @brief Reset the GaitEngine base class.
		*
		* This function is used by the `Gait` class, in conjunction with the virtual `reset()` function,
		* to reset the GaitEngine object in terms of both its base and derived components respectively.
		**/
		void resetBase()
		{
			in.reset();
			out.reset();
			updateHaltPose();
			m_posX = 0.0;
			m_posY = 0.0;
			m_rotZ = 0.0;
		}

		// Halt pose specification
		double haltJointCmd[NUM_JOINTS];    //!< Commanded halt position for each joint (indexed by the `JointID` enum, in `rad`).
		double haltJointEffort[NUM_JOINTS]; //!< Commanded halt joint effort (indexed by the `JointID` enum, in the range `[0,1]`).
		bool   haltUseRawJointCmds;         //!< Apply the joint commands directly to the hardware in the halt pose, without using compensation or actuator controller(s) in-between.
		
		// Pointer to the owning gait class
		const Gait* m_gait;

	private:
		// Internal variables
		double m_posX;
		double m_posY;
		double m_rotZ;
		
		// Function to set gait owner
		void setGaitOwner(const Gait* gaitPtr) { m_gait = gaitPtr; }

		// Friend classes
		friend class Gait;
	};
}

#endif /* GAIT_ENGINE_H */
// EOF