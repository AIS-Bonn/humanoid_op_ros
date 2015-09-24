// Behaviour Control Framework - Template function definitions
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file behaviour_template_defns.h
* @brief Defines various template functions for the %Behaviour Control Framework.
*
* This file is necessary because template functions need to be defined in the headers, yet
* instances exist where the template functions need to reference classes that haven't been
* defined yet, leading to 'invalid use of incomplete type' errors. This header is intended
* to be included at the very end of `behaviour_control.h`, at which point all class types
* should be complete and otherwise a-ok!
**/

// Ensure header is only included once
#ifndef BEHAVIOUR_TEMPLATE_DEFNS_H
#define BEHAVIOUR_TEMPLATE_DEFNS_H

// Includes
#include <behaviour_control/classes/behaviour_common.h>

// Behaviour control namespace
namespace behaviourcontrol
{
	//
	// SensorManager class
	//

	// Sensor declaration function
	template <class SClass>
	void SensorManager::declareSensor(SClass* sensor)
	{
		// Ensure at compile time that this function is only called with derived classes of SensorBase
		BOOST_STATIC_ASSERT_MSG((boost::is_base_of<SensorBase, SClass>::value), "SClass template parameter must be a derived class of behaviourcontrol::SensorBase");

		// Error checking
		ASSERT_WARNING(sensor != NULL, MBase, "Failed to declare the requested sensor as a child of the '" + LBase->name + "' layer sensor manager - Provided sensor pointer was null!");
		ASSERT_WARNING(!MBase->isInitialising(), MBase, "Attempted to declare the '" + sensor->signalName + "'sensor as a child of the '" + LBase->name + "' layer sensor manager during initialisation - All sensor declarations should be in the sensor manager constructor!");
		ASSERT_WARNING(!MBase->wasInitialised(), MBase, "Attempted to declare the '" + sensor->signalName + "'sensor as a child of the '" + LBase->name + "' layer sensor manager after initialisation - All sensor declarations should be in the sensor manager constructor!");

		// Add the sensor to the sensor list
		SList.push_back(sensor->getBasePtr());
	}

	//
	// Sensor class
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
	template <class T>
	Sensor<T>::Sensor(SensorManager* SMBase, const std::string& signalName)
		: SensorBase(SMBase, signalName)
		, typeInfo(&typeid(T)) // This is safe because type_info objects don't get destroyed until the program ends
		, sourceAct(NULL)
		, data() // Initialise template data member to zero
		, actWasWrittenTo(false)
	{
	}

	// Destructor
	template <class T>
	Sensor<T>::~Sensor()
	{
	}

	// Actuator compatibility check function
	template <class T>
	bool Sensor<T>::isCompatibleWith(const ActuatorBase* ABase) const
	{
		// Verify that the actuator has the same data type as the sensor
		if(ABase == NULL)
			return false;
		else
			return (*typeInfo == *(ABase->getTypeInfo()));
	}

	// Bind to an actuator of the same data type
	template <class T>
	ret_t Sensor<T>::bindTo(const ActuatorBase* ABase)
	{
		// Check for null pointer
		if(ABase == NULL)
			return RET_NULL_POINTER;

		// Ensure that we are still in the initialisation phase (no re-routing bindings on the fly!)
		if(MBase->wasInitialised())
		{
			REPORT_WARNING(MBase, "Request to bind to actuator '" + ABase->signalName + "' denied - The behaviour manager has already been successfully initialised!");
			return RET_ALREADY_INITIALISED;
		}

		// Verify actuator compatibility
		if(!isCompatibleWith(ABase))
		{
			REPORT_WARNING(MBase, "Request to bind to actuator '" + ABase->signalName + "' denied - This actuator has the wrong data type ('" + std::string(ABase->getTypeInfo()->name()) + "')!");
			return RET_BIND_FAILED_ASI;
		}

		// Retrieve a pointer to the actual Actuator<T> class
		try { sourceAct = dynamic_cast<const Actuator<T>*>(ABase); } // The whole try/catch is overkill here, but I don't want to risk anything (this framework could be used in some safety critical application - no crashing allowed!)
		catch(...)
		{
			sourceAct = NULL;
			REPORT_WARNING(MBase, "Request to bind to actuator '" + ABase->signalName + "' failed - Dynamic cast to type 'const Actuator<T> *' for T = '" + std::string(ABase->getTypeInfo()->name()) + "' failed!");
			return RET_BIND_FAILED_ASI;
		}

		// Successful binding to the actuator
		return RET_OK;
	}

	// Get the latest data from the bound actuator
	template <class T>
	void Sensor<T>::getLatestData()
	{
		if(sourceAct != NULL)
		{
			data = sourceAct->read();
			actWasWrittenTo = sourceAct->wasWrittenTo();
		}
	}

	//
	// ActuatorManager class
	//

	// Actuator declaration function
	template <class AClass>
	void ActuatorManager::declareActuator(AClass* actuator)
	{
		// Ensure at compile time that this function is only called with derived classes of ActuatorBase
		BOOST_STATIC_ASSERT_MSG((boost::is_base_of<ActuatorBase, AClass>::value), "AClass template parameter must be a derived class of behaviourcontrol::ActuatorBase");

		// Error checking
		ASSERT_WARNING(actuator != NULL, MBase, "Failed to declare the requested actuator as a child of the '" + LBase->name + "' layer actuator manager - Provided actuator pointer was null!");
		ASSERT_WARNING(!MBase->isInitialising(), MBase, "Attempted to declare the '" + actuator->signalName + "'actuator as a child of the '" + LBase->name + "' layer actuator manager during initialisation - All actuator declarations should be in the actuator manager constructor!");
		ASSERT_WARNING(!MBase->wasInitialised(), MBase, "Attempted to declare the '" + actuator->signalName + "'actuator as a child of the '" + LBase->name + "' layer actuator manager after initialisation - All actuator declarations should be in the actuator manager constructor!");

		// Add the actuator to the actuator list
		AList.push_back(actuator->getBasePtr());
	}

	//
	// Actuator class
	//

	// Default constructor
	/**
	* Note that although the default value for the @p signalName parameter is @c nullString, this will
	* not actually become the actuator's name if the parameter is omitted. This is just used to signal
	* to the class's internals that it should generate a default name.
	*
	* @param AMBase A pointer to the actuator manager that the actuator belongs to.
	* @param signalName The unique human-friendly name to give the actuator (null means auto-generate).
	* This name is used by the sensor classes to look up and bind to the correct actuators. The name
	* can be completely arbitrary, with the only restriction being that it can't be the null string, but
	* recommended sample names include `"ThisLayer/ActuatorName"` and `"Layer1/TargetX"`, where
	* `ThisLayer` and `Layer1` are sample names of the layer that owns the actuator being constructed,
	* and `ActuatorName` and `TargetX` are sample actuator-specific names.
	* @param aggregatable A boolean flag specifying whether the actuator values are intended to be
	* aggregatable or not. Aggregatable actuators automatically calculate a running weighted average
	* during a step if they are written to multiple times by different behaviours. Non-aggregatable
	* actuators just use the last value that was written to them. Multiple writes to a non-aggregatable
	* actuator during a single step should always be avoided, as it is then not clear which value ends
	* up being used (although this is a deterministic selection of course).
	* @see @link Sensor::Sensor Default Sensor constructor @endlink
	**/
	template <class T>
	Actuator<T>::Actuator(ActuatorManager* AMBase, const std::string& signalName, bool aggregatable)
		: ActuatorBase(AMBase, signalName)
		, aggregatable(aggregatable)
		, typeInfo(&typeid(T)) // This is safe because type_info objects don't get destroyed until the program ends
		, data() // Initialise template data member to zero
		, weightSum(0.0)
		, lastModifier(NULL)
	{
	}

	// Destructor
	template <class T>
	Actuator<T>::~Actuator()
	{
	}

	// Sensor compatibility check function
	template <class T>
	bool Actuator<T>::isCompatibleWith(const SensorBase* SBase) const
	{
		// Verify that the sensor has the same data type as the actuator
		if(SBase == NULL)
			return false;
		else
			return (*typeInfo == *(SBase->getTypeInfo()));
	}

	// Write functions
	template <class T>
	void Actuator<T>::write(const T& newdata, const Behaviour* BBase)
	{
		// Declare variables
		level_t weight = 0.0;
// 		bool firstWrite = (weightSum <= 0.0);

		// Error checking
		ASSERT_WARNING(MBase->wasInitialised(), MBase, "A call to write() was attempted on the actuator '" + signalName + "' (part of the '" + LBase->name + "' layer) before or during initialisation - consider using writeHard() instead!");
		ASSERT_WARNING(BBase != NULL, MBase, "Attempted to write to the actuator '" + signalName + "' with null behaviour pointer!");
		ASSERT_WARNING(BBase != lastModifier, MBase, "The behaviour '" + BBase->name + "' attempted multiple writes to the actuator '" + signalName + "'!");
		ASSERT_WARNING(!LBase->isInterface, MBase, "A call to write() was attempted on an actuator ('" + signalName + "') that belongs to an interface layer ('" + LBase->name + "') - consider using writeHard() instead!");

		// Disabled as these would get annoying for the user if this is intended behaviour (even if it is not recommended)
// 		ASSERT_WARNING(firstWrite, MBase, "More than one write to the actuator '" + signalName + "' was detected within the cycle " + std::string(MBase->cycleID()) + " (part of the '" + LBase->name + "' layer)!");
// 		ASSERT_WARNING(!aggregatable, MBase, "A call to write() was made for the aggregatable actuator '" + signalName + "' (part of the '" + LBase->name + "' layer) - consider using writeAgg() instead!");

		// Retrieve the required activation level (i.e. incremental averaging weight)
		weight = BBase->getA();
		lastModifier = BBase;

		// Ignore function call if weight is zero (or negative for some mysterious reason)
		if(weight <= 0.0) return;

		// Update the weight sum
		weightSum += weight; // We are guaranteed to have weightSum > 0.0 after this => firstWrite will be false in subsequent calls until weightSum is reset (e.g. in update())

		// Update the internal actuator data (a hard write)
		data = newdata;
	}
	template <class T>
	void Actuator<T>::writeAgg(const T& newdata, const Behaviour* BBase)
	{
		// Declare variables
		level_t weight = 0.0;
		bool firstWrite = (weightSum <= 0.0);

		// Error checking
		ASSERT_WARNING(MBase->wasInitialised(), MBase, "A call to write() was attempted on the actuator '" + signalName + "' (part of the '" + LBase->name + "' layer) before or during initialisation - consider using writeHard() instead!");
		ASSERT_WARNING(BBase != NULL, MBase, "Attempted to write to the actuator '" + signalName + "' with null behaviour pointer!");
		ASSERT_WARNING(BBase != lastModifier, MBase, "The behaviour '" + BBase->name + "' attempted multiple writes to the actuator '" + signalName + "'!");
		ASSERT_WARNING(!LBase->isInterface, MBase, "A call to write() was attempted on an actuator ('" + signalName + "') that belongs to an interface layer ('" + LBase->name + "') - consider using writeHard() instead!");
		ASSERT_WARNING(aggregatable, MBase, "A call to writeAgg() was made for the non-aggregatable actuator '" + signalName + "' (part of the '" + LBase->name + "' layer) - consider using write() instead!");

		// Retrieve the required activation level (i.e. incremental averaging weight)
		weight = BBase->getA();
		lastModifier = BBase;

		// Ignore function call if weight is zero (or negative for some mysterious reason)
		if(weight <= 0.0) return;

		// Update the weight sum
		weightSum += weight; // We are guaranteed to have weightSum > 0.0 after this => firstWrite will be false in subsequent calls until weightSum is reset (e.g. in update())

		// Update the internal actuator data
		if(firstWrite)
			data = newdata;
		else
		{
			// Calculate the ratio of how important the new data is
			level_t p = weight/weightSum;

			// Update the internal actuator data
			// Removed due to possible system incompatabilities: if(boost::is_integral<T>::value || !(boost::has_multiplies<T, level_t, T>::value && boost::has_plus<T>::value)) // In summary: A type T is aggregatable if it is integral, or it can't do either T * level_t or T + T
			data = data*(1-p) + newdata*p; // Aggregatable case
		}
	}
	template <class T>
	void Actuator<T>::writeHard(const T& newdata)
	{
		// Determine whether this is the first write
		bool firstWrite = (weightSum <= 0.0);

		// Error checking
		ASSERT_WARNING((LBase->isInterface || !MBase->wasInitialised()), MBase, "A call to writeHard() was attempted on the actuator '" + signalName + "' (part of non-interface layer '" + LBase->name + "') after initialisation - consider using write() instead!");
		ASSERT_WARNING((firstWrite || MBase->wasInitialised()), MBase, "More than one write to the actuator '" + signalName + "' was detected during initialisation (part of the '" + LBase->name + "' layer)!");

		// Disabled because we want to be able to write to the actuator multiple times in for example a ROS callback, and only use the latest value.
//		ASSERT_WARNING(firstWrite, MBase, "More than one write to the actuator '" + signalName + "' was detected in the '" + LBase->name + "' interface layer!");

		// Overwrite the data in the actuator
		data = newdata;
		weightSum = 1.0; // Non-zero so that multiple writes can be detected
		lastModifier = NULL;
	}

	//
	// BehaviourManager class
	//

	// Layer declaration function
	template <class LClass>
	void BehaviourManager::declareLayer(LClass* layer)
	{
		// Ensure at compile time that this function is only called with derived classes of BehaviourLayer
		BOOST_STATIC_ASSERT_MSG((boost::is_base_of<BehaviourLayer, LClass>::value), "LClass template parameter must be a derived class of behaviourcontrol::BehaviourLayer");

		// Error checking
		ASSERT_WARNING(layer != NULL, this, "Failed to declare the requested layer as a child of the '" + name + "' behaviour manager - Provided layer pointer was null!");
		ASSERT_WARNING(!isInitialising(), this, "Attempted to declare the '" + layer->name + "'layer as a child of the '" + name + "' behaviour manager during initialisation - All layer declarations should be in the behaviour manager constructor!");
		ASSERT_WARNING(!wasInitialised(), this, "Attempted to declare the '" + layer->name + "'layer as a child of the '" + name + "' behaviour manager after initialisation - All layer declarations should be in the behaviour manager constructor!");

		// Add the layer to the layer list
		LList.push_back(layer->getBasePtr());
	}

	//
	// BehaviourLayer class
	//

	// Behaviour declaration function
	template <class BClass>
	void BehaviourLayer::declareBehaviour(BClass* behaviour)
	{
		// Ensure at compile time that this function is only called with derived classes of Behaviour
		BOOST_STATIC_ASSERT_MSG((boost::is_base_of<Behaviour, BClass>::value), "BClass template parameter must be a derived class of behaviourcontrol::Behaviour");

		// Error checking
		ASSERT_WARNING(behaviour != NULL, MBase, "Failed to declare the requested behaviour as a child of the '" + name + "' layer - Provided behaviour pointer was null!");
		ASSERT_WARNING(!MBase->isInitialising(), MBase, "Attempted to declare the '" + behaviour->name + "'behaviour as a child of the '" + name + "' layer during initialisation - All behaviour declarations should be in the layer constructor!");
		ASSERT_WARNING(!MBase->wasInitialised(), MBase, "Attempted to declare the '" + behaviour->name + "'behaviour as a child of the '" + name + "' layer after initialisation - All behaviour declarations should be in the layer constructor!");

		// Add the behaviour to the behaviour list
		BList.push_back(behaviour->getBasePtr());
	}
}

#endif /* BEHAVIOUR_TEMPLATE_DEFNS_H */
// EOF