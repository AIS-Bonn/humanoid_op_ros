// Hardware interface base class
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef RC_HARDWAREINTERFACE_H
#define RC_HARDWAREINTERFACE_H

#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <robotcontrol/Diagnostics.h>

namespace robotcontrol
{
class RobotModel;
struct Joint;

/**
 * @brief Abstract base for hardware interfaces
 *
 * All hardware interfaces need to implement this interface and register
 * themselves via pluginlib (see robotcontrol::DummyInterface for an example).
 **/
class HardwareInterface
{
public:
	virtual ~HardwareInterface() {}

	/**
	 * @brief Initialization
	 *
	 * Do everything here that needs to do some initialization depending on
	 * the robot model.
	 *
	 * @warning This method gets called **after** createJoint()!
	 **/
	virtual bool init(RobotModel* model) = 0;

	/**
	 * @brief Joint factory method
	 *
	 * Should create a robotcontrol::Joint object for the specified joint name.
	 * This is used to create Joint subclasses in hardware interfaces which
	 * need to store extra information per joint.
	 *
	 * @note The method does not need to fill in any joint fields (in particular
	 *  the joint name)
	 *
	 * @param name URDF name of the joint to be constructed
	 * @return a robotcontrol::Joint instance
	 **/
	virtual boost::shared_ptr<Joint> createJoint(const std::string& name) = 0;

	/**
	 * @brief Send position feedback
	 *
	 * Should send the position commands from the robotcontrol::Joint structs
	 * to the robot actuators.
	 *
	 * @return true on success
	 **/
	virtual bool sendJointTargets() = 0;

	/**
	 * @brief Get position feedback
	 *
	 * Should store the retrieved joint positions in the appropiate
	 * robotcontrol::Joint::Feedback struct.
	 *
	 * @return true on success
	 **/
	virtual bool readJointStates() = 0;

	/**
	 * @brief Set robot stiffness
	 *
	 * This is used during initial torque fade-in at robot startup.
	 *
	 * @param torque Relative stiffness (in range 0..1)
	 * @return true on success
	 **/
	virtual bool setStiffness(float torque) = 0;

	/**
	 * @brief Get hardware diagnostics
	 *
	 * @param ptr Diagnostics message to fill in
	 **/
	virtual void getDiagnostics(robotcontrol::DiagnosticsPtr ptr) = 0;
	
	/**
	 * @brief Query emergency stop state
	 * 
	 * If this function returns true, execution of motion modules is halted.
	 * Note: the default implementati
	 **/
	virtual bool emergencyStopActive();
};

}

#endif
