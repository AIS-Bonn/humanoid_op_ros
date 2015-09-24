// Behaviour Control Framework - BehaviourManager class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <behaviour_control/behaviour_control.h>

// Namespaces
using namespace std;
using namespace behaviourcontrol;

// Constants
const index_t BehaviourManager::DEF_LLIST_CAPACITY = 4;
const std::string BehaviourManager::DEF_MANAGER_NAME = "BehaviourManager";

//
// Constructors
//

// Default constructor
/**
* Note that although the default value for the @p name parameter is @c nullString, this will not
* actually become the behaviour manager's name if the parameter is omitted. This is just used to signal
* to the class's internals that it should use the default name
* (@ref BehaviourManager::DEF_MANAGER_NAME "DEF_MANAGER_NAME") when none is provided.
*
* @param name The human-friendly name to give the behaviour manager (null means use default)
**/
BehaviourManager::BehaviourManager(const std::string& name)
	: name(name.empty() ? DEF_MANAGER_NAME : name)
	, initialised(false)
	, initialising(false)
	, icycleID(0)
	, fatalErrorOccurred(false)
	, nonfatalErrorOccurred(false)
{
	// Initialise layer list capacity
	LList.reserve(DEF_LLIST_CAPACITY);
}

// Destructor
BehaviourManager::~BehaviourManager()
{
}

//
// Initialisation functions
//

// Function: BehaviourManager::initialiseArchitecture()
/**
* The initialisation order is as follows:
* - %Behaviour manager base: `initBase()`
* - %Behaviour layer base's: `BehaviourLayer::initAllBase()`
* - Derived behaviour manager: `init()`
* - Derived behaviour layers: `BehaviourLayer::initAll()`
* 
* The function can only be called once, or #RET_ALREADY_INITIALISED is returned.
*
* @return A @c #ret_t type that specifies whether the initialisation was successful.
**/
ret_t BehaviourManager::initialiseArchitecture()
{
	// Check whether there were any fatal construction (or prior partial-initialisation) errors
	if(fatalErrorOccurred)
	{
		REPORT_WARNING(this, "Attempted to initialise the architecture '" + name + "' despite a previous fatal error!");
		return RET_PREVIOUS_FATAL_ERROR;
	}

	// Ensure that this is the first and only time that this function is called
	if(initialised || initialising)
	{
		REPORT_WARNING(this, "Attempted to initialise the architecture '" + name + "' more than once!");
		return RET_ALREADY_INITIALISED;
	}

	// Declare variables
	ret_t ret;
	index_t i;

	// Flag that initialisation is starting
	initialising = true;

	// Initialise the internals of the behaviour manager
	ret = initBase();
	if(ret != RET_OK)
	{
		initialising = false;
		return ret;
	}

	// Initialise the internals of all the child behaviour layers
	for(i = 0;i < LList.size();i++)
	{
		ret = LList[i]->initAllBase();
		if(ret != RET_OK)
		{
			initialising = false;
			return ret;
		}
	}

	// Initialise the user's derived behaviour manager
	ret = init();
	if(ret != RET_OK)
	{
		initialising = false;
		return ret;
	}

	// Initialise the user's derived child behaviour layers
	for(i = 0;i < LList.size();i++)
	{
		ret = LList[i]->initAll();
		if(ret != RET_OK)
		{
			initialising = false;
			return ret;
		}
	}

	// Flag that initialisation was completed successfully
	initialising = false;
	initialised = true;

	// Success if we got this far
	return RET_OK;
}

// Initialise the internals of the behaviour manager
ret_t BehaviourManager::initBase()
{
	// Nothing required for now
	return RET_OK;
}

//
// Update and step functions
//

// Function: BehaviourManager::step()
/**
* This function is generally placed in a timed loop that controls the step rate of the architecture.
* This function does nothing if a fatal error has previously been encountered.
* 
* The order of execution is as follows:
* - Pre-step callback: `preStepCallback()`
* - For each <b>interface layer</b>:
*   - Update the actuators for a new step: `ActuatorManager::updateActuators()`
*   - Update the interface layer: `BehaviourLayer::update()`
*   - Read the new external data: `ActuatorManager::readExternalData()`
* - For each <b>normal layer</b> call `BehaviourLayer::step()`:
*   - Update the sensor manager: `SensorManager::updateSensors()`
*   - Update the actuator manager: `ActuatorManager::updateActuators()`
*   - Update the layer: `BehaviourLayer::update()`
*   - For each <b>child behaviour</b>:
*     - Update the behaviour: `Behaviour::update()`
*   - Compute the raw activation levels of each behaviour: `Behaviour::computeActivationLevel()`
*   - Apply the inhibitions of the layer to get the true activation levels (the result of which can be retrieved later using `Behaviour::getA()`)
*   - For each <b>child behaviour</b>:
*     - If `Behaviour::getA() > 0.0` then execute the activated behaviour: `Behaviour::execute()`
*     - Otherwise execute the inhibited behaviour: `Behaviour::inhibited()`
*   - Invoke post-execute callback: `BehaviourLayer::postExecuteCallback()`
* - For each <b>interface layer</b>:
*   - Update the sensors for a new step: `SensorManager::updateSensors()`
*   - Write the new external data: `SensorManager::writeExternalData()`
*   - Invoke post-execute callback: `BehaviourLayer::postExecuteCallback()`
* - Post-step callback: `postStepCallback()`
**/
void BehaviourManager::step()
{
	// Declare variables
	index_t i;

	// Check whether there were any fatal construction, initialisation or stepping errors
	ASSERT_WARNING(!fatalErrorOccurred, this, "Attempted to step the architecture '" + name + "' despite a previous fatal error!");

	// Increment the cycle ID
	icycleID++;
	
	// Call the pre-step user callback
	preStepCallback();

	// Retrieve external data via the interface layers and re-publish internally
	for(i = 0;i < LList.size();i++)
	{
		if(LList[i]->isInterface && LList[i]->haveAM)
		{
			LList[i]->AMBase->updateActuators();
			LList[i]->update();
			LList[i]->AMBase->readExternalData();
		}
	}

	// Perform a step on each normal (non-interface) child layer
	for(i = 0;i < LList.size();i++)
	{
		if(!LList[i]->isInterface)
			LList[i]->step();
	}

	// Re-publish internal data externally via the interface layers
	for(i = 0;i < LList.size();i++)
	{
		if(LList[i]->isInterface && LList[i]->haveSM)
		{
			LList[i]->SMBase->updateSensors();
			LList[i]->SMBase->writeExternalData();
			LList[i]->postExecuteCallback();
		}
	}

	// Call the post-step user callback
	postStepCallback();
}

//
// Error notification functions
//

/**
* @param msg A `std::string` message that describes the error.
* @param fatal A flag specifying whether the error is fatal (i.e. an @e error) or non-fatal (i.e. just a @e warning).
* @param funcName The name of the function in which the error was encountered (generally @c \__func__, @c \__FUNCTION__ or similar).
* @param fileName The name of the file in which the error was encountered (generally @c \__FILE__).
* @param line The line number at which the error was encountered (generally @c \__LINE__).
**/
void BehaviourManager::reportError(const std::string& msg, bool fatal, const std::string& funcName, const std::string& fileName, int line)
{
	// Set the appropriate flag(s)
	if(fatal)
	{
		fatalErrorOccurred = true;
		nonfatalErrorOccurred = true;
	}
	else nonfatalErrorOccurred = true;

	// Call the user's error notification function
	reportErrorUser(msg, fatal, funcName, fileName, line);
}

// Clear any current error conditions
void BehaviourManager::clearErrorStatus()
{
	// Reset the internal warning and error flags
	fatalErrorOccurred = false;
	nonfatalErrorOccurred = false;
}

//
// Get functions
//

/**
* Note that this function returns the @e first actuator of the requested name. The layers are
* searched in order from the first layer in the manager's layer list (`BehaviourManager::LList`) to the last.
* @c NULL is returned if no actuator of the requested name was found, or if @p signalName was a null string.
*
* @param signalName The actuator name to search for.
**/
const ActuatorBase* BehaviourManager::findActuator(const std::string& signalName) const
{
	// Dismiss call if requested name is null
	if(signalName.empty()) return NULL;

	// Declare variables
	const ActuatorBase* ABase = NULL;
	index_t i;

	// Ask all the layers whether they own such an actuator, and return immediately if they return affirmative
	for(i = 0;i < LList.size();i++)
	{
		if((ABase = LList[i]->findActuator(signalName)) != NULL)
		{
			return ABase;
		}
	}

	// No actuator of the given name was found
	return NULL;
}

//
// Helper functions
//

// Function: BehaviourManager::toString()
/**
* This function displays the whole object hierarchy of the given manager. A sample output is shown below.
@verbatim
Defined Behaviour Architecture
==============================
The MyM behaviour manager contains:
  (0) RosIL (Interface)
  (1) MyL1
    (0) MyB2
    (1) MyB1
  (2) MyL2
    (0) MyB3
    (1) MyB4

RosIL Layer
===========
RosIL has a sensor manager with the following sensors:
  (0) MyL2/mode
  (1) MyL2/count
  (2) MyL2/target
RosIL has an actuator manager with the following actuators:
  (0) ROS/mode
  (1) ROS/targetX
  (2) ROS/targetY

MyL1 Layer
==========
 MyL1 has a sensor manager with the following sensors:
  (0) ROS/mode
  (1) ROS/targetX
  (2) ROS/targetY
MyL1 has an actuator manager with the following actuators:
  (0) MyL1/mode
  (1) MyL1/xgoal
  (2) MyL1/vgoal

MyL2 Layer
==========
MyL2 has a sensor manager with the following sensors:
  (0) MyL1/mode
  (1) MyL1/xgoal
  (2) MyL1/vgoal
MyL2 has an actuator manager with the following actuators:
  (0) MyL2/mode
  (1) MyL2/count
  (2) MyL2/target
@endverbatim
**/
std::string BehaviourManager::toString(const std::string& linePrefix) const
{
	// Declare variables
	std::ostringstream out;
	BehaviourLayer* Ltmp;
	std::string strtmp;
	index_t i, j;

	// Add a string representation of the layer/behaviour tree
	out << linePrefix << "Defined Behaviour Architecture" << std::endl;
	out << linePrefix << "==============================" << std::endl;
	out << linePrefix << "The " << name << " behaviour manager contains:" << std::endl;
	for(i = 0;i < LList.size();i++)
	{
		Ltmp = LList[i];
		out << linePrefix << "  (" << i << ") " << Ltmp->name << (Ltmp->isInterface ? " (Interface)" : "") << std::endl;
		for(j = 0;j < Ltmp->BList.size();j++)
			out << linePrefix << "    (" << j << ") " << Ltmp->BList[j]->name << std::endl;
	}

	// Add string representations of the sensor and actuator managers of each of the layers in the behaviour manager
	for(i = 0;i < LList.size();i++)
	{
		Ltmp = LList[i];
		out << std::endl;
		out << linePrefix << Ltmp->name << " Layer" << std::endl;
		strtmp.assign(Ltmp->name.length() + 6, '=');
		out << linePrefix << strtmp << std::endl;
		if(Ltmp->hasSM())
		{
			out << linePrefix << Ltmp->name << " has a sensor manager with the following sensors:" << std::endl;
			for(j = 0;j < Ltmp->getSMBase()->SList.size();j++)
				out << linePrefix << "  (" << j << ") " << Ltmp->getSMBase()->SList[j]->signalName << std::endl;
		}
		else
		{
			out << linePrefix << Ltmp->name << " does not have a sensor manager!" << std::endl;
			out << linePrefix << "  (X) ---" << std::endl;
		}
		if(Ltmp->hasAM())
		{
			out << linePrefix << Ltmp->name << " has an actuator manager with the following actuators:" << std::endl;
			for(j = 0;j < Ltmp->getAMBase()->AList.size();j++)
				out << linePrefix << "  (" << j << ") " << Ltmp->getAMBase()->AList[j]->signalName << std::endl;
		}
		else
		{
			out << linePrefix << Ltmp->name << " does not have an actuator manager!" << std::endl;
			out << linePrefix << "  (X) ---" << std::endl;
		}
	}

	// Return the required string
	return out.str();
}
// EOF