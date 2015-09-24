// Behaviour Control Framework - Sensor classes
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <behaviour_control/behaviour_control.h>

// Namespaces
using namespace std;
using namespace behaviourcontrol;

// Constants
const index_t SensorManager::DEF_SLIST_CAPACITY = 32;

//
// SensorManager class
//

// Default constructor
SensorManager::SensorManager(BehaviourLayer* LBase)
	: MBase(LBase->MBase)
	, LBase(LBase)
{
	// Initialise sensor list capacity
	SList.reserve(DEF_SLIST_CAPACITY);

	// Automatically set SensorManager pointer in the owning BehaviourLayer
	LBase->setSensorManager(this);
}

// Destructor
SensorManager::~SensorManager()
{
}

// Initialisation functions
ret_t SensorManager::initAllBase()
{
	// Declare variables
	ret_t ret;
	index_t i;

	// Initialise the internals of all the child sensors
	for(i = 0;i < SList.size();i++)
	{
		ret = SList[i]->initBase();
		if(ret != RET_OK) return ret;
	}

	// Success if we got this far
	return RET_OK;
}

// Update function
void SensorManager::updateSensors()
{
	// Go through each sensor and get the latest data
	for(index_t i = 0;i < SList.size();i++)
		SList[i]->getLatestData();
}

//
// SensorBase class
//

// Default constructor
/**
* @param SMBase A pointer to the sensor manager that the sensor belongs to.
* @param signalName The unique human-friendly name of the actuator to bind the sensor to. Lookup of
* the actuator occurs using this name as the key. The name can be completely arbitrary, with the only
* restriction being that it can't be the null string, but recommended sample names include
* `"OtherLayer/ActuatorName"` and `"Layer1/TargetX"`, where `OtherLayer` and `Layer1` are the names of
* the layers in which the `ActuatorName` and `TargetX` actuators are defined respectively.
* @see @link Actuator::Actuator Default Actuator constructor @endlink
**/
SensorBase::SensorBase(SensorManager* SMBase, const std::string& signalName)
	: MBase(SMBase->MBase)
	, LBase(SMBase->LBase)
	, SMBase(SMBase)
	, signalName(signalName)
{
	// Automatically declare the sensor in the containing SensorManager
	SMBase->declareSensor(this);
}

// Destructor
SensorBase::~SensorBase()
{
}

// Initialisation function
ret_t SensorBase::initBase()
{
	// Don't attempt to bind to actuator if signal name is null
	if(signalName.empty())
	{
		REPORT_ERROR(MBase, "Failed to bind to actuator - Actuator signal name cannot be a null string!");
		return RET_BIND_FAILED_NS;
	}

	// Ask the behaviour manager for a pointer to the requested actuator
	const ActuatorBase* ABase = MBase->findActuator(signalName);
	if(ABase == NULL)
	{
		REPORT_ERROR(MBase, "Failed to bind to actuator - Actuator with given signal name ('" + signalName + "') not found!");
		return RET_BIND_FAILED_ANF;
	}

	// Attempt to bind to the actuator (the bindTo() function must check whether this is possible data type-wise)
	ret_t ret;
	if((ret = bindTo(ABase)) != RET_OK)
		return ret;

	// Return success
	return RET_OK;
}
// EOF