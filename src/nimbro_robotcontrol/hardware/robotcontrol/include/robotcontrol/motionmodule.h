// Interface for robotcontrol motion module plugins
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//         Sebastian Schüller <schuell1@cs.uni-bonn.de>

// Ensure header is only included once
#ifndef MOTIONMODULE_H
#define MOTIONMODULE_H

// Includes
#include <string>

// Robotcontrol namespace
namespace robotcontrol
{
	// Class declarations
	class RobotModel;

	/**
	* @class MotionModule
	* 
	* @brief Abstract base class for all motion modules.
	**/
	class MotionModule
	{
	public:
		//! @brief Default constructor
		MotionModule() : m_model(NULL), m_paramString("") {}

		//! @brief Default destructor
		virtual ~MotionModule() {}

		/**
		* @brief Initialisation function of the motion module
		*
		* This function is intended for overriding by derived motion module classes, to allow these subclasses to
		* perform their required initialisation. Note however that the base implementation must be explicitly called
		* first thing by all overrides of this function, and return `false` if it returns `false`. Something like:
		* @code
		* bool DerivedMotionModule::init(robotcontrol::RobotModel* model)
		* {
		* 	// Initialise the base class
		* 	if(!MotionModule::init(model)) return false;
		*
		* 	// Extra initialisation
		* 	...
		*
		* 	// Return that initialisation was successful
		* 	return true;
		* }
		* @endcode
		* 
		* @param model The instance of RobotModel to operate on.
		* @return `true` if initialisation is successful, `false` otherwise.
		**/
		virtual bool init(RobotModel* model);

		/**
		* @brief Trigger function
		*
		* Returns whether the motion module wishes to be active and execute in the current time step.
		* The default implementation always returns `true`, meaning that `step()` always executes.
		**/
		virtual bool isTriggered();

		/**
		* @brief Main step function of the motion module
		*
		* This function is called whenever the motion module is triggered, and should calculate and set joint commands
		* appropriate for the required action of the motion module.
		*
		* The `step()` function is pure virtual and must be overridden in derived classes.
		**/
		virtual void step() = 0;

		/**
		* @brief Function to publish the required frame transforms
		*
		* This function can be used to, for example, publish TF frame transforms for the frames that the motion module
		* defines and is responsible for. It should not be assumed that this function is called every time that `step()`
		* is called for the motion module.
		**/
		virtual void publishTransforms() {}

		//! @brief Return the internal instance of RobotModel that is being operated on
		RobotModel* model() { return m_model; }

		//! @brief Return the internal instance of RobotModel that is being operated on (constant reference)
		const RobotModel* model() const { return m_model; }

		/**
		 * @brief Determine if current situation is safe
		 *
		 * If this function returns false, a fade-in is prevented. This can be
		 * used to recognize some invalid robot state, and deny any fade-in
		 * request until the situation is manually resolved.
		 *
		 * @note If not overriden, this method always returns true.
		 **/
		virtual bool isSafeToFadeIn();

		//! @brief Get motion module name (class name)
		const std::string& name() const
		{ return m_name; }

		/**
		 * @brief Handle emergency stop
		 *
		 * If you need to reset any internal state when the emergency stop is
		 * active (see HardwareInterface::emergencyStopActive()), you can do
		 * it here.
		 **/
		virtual void handleEmergencyStop() {}

	protected:
		/**
		* @brief Set a joint command based on position and effort only
		*
		* @param index Index of the joint to command in the `RobotModel::m_joints[]` array. See `RobotModel::jointIndex()`
		* and `RobotModel::joint()` for more information on how to obtain the joint index you need.
		* @param pos The position of the joint to command (`rad`).
		* @param effort The effort to use in trying to follow the joint command (in the range `[0,1]`).
		* @param raw Boolean flag specifying whether the joint command should be interpreted as a raw (i.e. direct) command (Default: False).
		**/
		void setJointCommand(int index, double pos, double effort, bool raw = false);

		//! @brief Get the motion module parameter string (intended for use by overrides of the `init()` function, valid only if the init() function is being or has been called)
		std::string getParamString() const { return m_paramString; }

	private:
		//! @brief Set the motion module's name for easier logging
		void setName(const std::string& name) { m_name = name; }

		//! @brief Set the motion module parameter string (intended for use by `RobotControl::initModules()` only)
		void setParamString(const std::string& paramString) { m_paramString = paramString; }

		//! @brief Internal `RobotModel` reference
		RobotModel* m_model;

		//! @brief Motion module name (class name)
		std::string m_name;

		//! @brief Motion module parameter string
		std::string m_paramString;

		// Friend classes
		friend class RobotControl;
	};
}

#endif /* MOTIONMODULE_H */
// EOF