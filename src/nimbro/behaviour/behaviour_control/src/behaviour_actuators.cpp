// Behaviour Control Framework - Actuator classes
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <behaviour_control/behaviour_control.h>

// Namespaces
using namespace std;
using namespace behaviourcontrol;

// Constants
const index_t ActuatorManager::DEF_ALIST_CAPACITY = 32;
const std::string ActuatorBase::DEF_ACTUATOR_SIGNAL_NAME = "Actuator";

//
// ActuatorManager class
//

// Default constructor
ActuatorManager::ActuatorManager(BehaviourLayer* LBase)
	: MBase(LBase->MBase)
	, LBase(LBase)
{
	// Initialise actuator list capacity
	AList.reserve(DEF_ALIST_CAPACITY);

	// Automatically set ActuatorManager pointer in the owning BehaviourLayer
	LBase->setActuatorManager(this);
}

// Destructor
ActuatorManager::~ActuatorManager()
{
}

ret_t ActuatorManager::initBase()
{
	// Nothing required for now
	return RET_OK;
}

// Update function
void ActuatorManager::updateActuators()
{
	// Declare to each actuator that a new step is commencing (required for incremental data averaging)
	for(index_t i = 0;i < AList.size();i++)
		AList[i]->update();
}

//
// ActuatorBase class
//

// Default constructor
/**
* Note that although the default value for the @p signalName parameter is @c nullString, this will
* not actually become the actuator's name if the parameter is omitted. This is just used to signal
* to the class's internals that it should generate a default name using the `getUniqueName()` function.
* @p AMBase must never be null or a segmentation fault will result!
*
* @param AMBase A pointer to the actuator manager that the actuator belongs to.
* @param signalName The unique human-friendly name to give the actuator (null means auto-generate).
* This name is used by the sensor classes to look up and bind to the correct actuators. The name
* can be completely arbitrary, with the only restriction being that it can't be the null string, but
* recommended sample names include `"ThisLayer/ActuatorName"` and `"Layer1/TargetX"`, where
* `ThisLayer` and `Layer1` are sample names of layers that own the actuators being constructed,
* and `ActuatorName` and `TargetX` are sample actuator-specific names.
* @see @link Sensor::Sensor Default Sensor constructor @endlink
**/
ActuatorBase::ActuatorBase(ActuatorManager* AMBase, const std::string& signalName)
	: MBase(AMBase->MBase)
	, LBase(AMBase->LBase)
	, AMBase(AMBase)
	, signalName(signalName.empty() ? getUniqueName(AMBase) : signalName)
{
	// Automatically declare the actuator in the containing ActuatorManager
	AMBase->declareActuator(this);
}

// Destructor
ActuatorBase::~ActuatorBase()
{
}

// Function: ActuatorBase::getUniqueName()
/**
* Automatically construct a unique name for the actuator, for situations where none is provided
* by the derived class. The name that results of this function will be unique as long as the
* user does not explicitly use the same name somewhere else, either before or after the construction
* of this actuator. If there are currently seven actuators in the @p AMBase actuator manager, the generated
* name will be the concatenation of @ref ActuatorBase::DEF_ACTUATOR_SIGNAL_NAME "DEF_ACTUATOR_SIGNAL_NAME" and "7".
*
* @param AMBase A non-null pointer to the parent actuator manager.
* @return The required actuator name.
**/
std::string ActuatorBase::getUniqueName(const ActuatorManager* AMBase) const
{
	// Return the required name
	std::ostringstream out;
	out << AMBase->LBase->name << "/" << DEF_ACTUATOR_SIGNAL_NAME << AMBase->AList.size();
	return out.str();
}
// EOF