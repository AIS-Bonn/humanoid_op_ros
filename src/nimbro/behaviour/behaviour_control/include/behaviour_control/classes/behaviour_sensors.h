// Behaviour Control Framework - Sensor classes
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file behaviour_sensors.h
* @brief Defines the sensor classes for the %Behaviour Control Framework.
**/

// Ensure header is only included once
#ifndef BEHAVIOUR_SENSORS_H
#define BEHAVIOUR_SENSORS_H

// Includes
#include <behaviour_control/classes/behaviour_common.h>

// Behaviour control namespace
namespace behaviourcontrol
{
	/**
	* @class SensorManager
	*
	* @brief Implements a manager of a particular group of sensors.
	*
	* This class is the base class of all sensor managers that are used in an architecture.
	**/
	class SensorManager
	{
	public:
		// Constants
		static const index_t DEF_SLIST_CAPACITY; //!< @brief Default capacity of the list that stores the child sensors of the sensor manager.

		// Constructors
		explicit SensorManager(BehaviourLayer* LBase); //!< @brief Default constructor
		virtual ~SensorManager(); //!< @brief %Sensor manager object destructor.

		// Constant properties
		BehaviourManager* const MBase; //!< @brief Pointer to the parent behaviour manager.
		BehaviourLayer* const LBase; //!< @brief Pointer to the parent behaviour layer.

		// Initialisation functions
	protected:
		virtual ret_t init() { return RET_OK; } //!< @brief Initialisation callback for the sensor manager. This function is called from `BehaviourLayer::initAll()`, and is intended to be overridden for the purpose of initialising the @e derived sensor manager object.
	private:
		ret_t initAllBase(); //!< @brief Initialisation function for the sensor manager. This function is called from `BehaviourLayer::initAllBase()`, and initialises the @e base sensor manager object, and all child SensorBase objects.

	private:
		// Update function
		void updateSensors(); //!< @brief Internal update function for the sensor manager. This function is called from `BehaviourLayer::updateAll()`, which is in turn called from `BehaviourLayer::step()`.

		// Sensor declaration function
		template <class SClass> void declareSensor(SClass* sensor); //!< @brief Adds @p sensor as a child sensor of the sensor manager.

		// Sensor list
		std::vector<SensorBase*> SList; //!< @brief The list of child sensors of this sensor manager.
		
	public:
		// Write data to an external source (used only in interface layers)
		virtual void writeExternalData() {} //!< @brief Interface layer specific callback that is intended to be overridden by the user to write any available data externally, as required (e.g. to other ROS nodes).

		// Get functions
		SensorManager* getBasePtr() { return this; } //!< @brief Return a pointer to the underlying SensorManager class object in the case of a derived sensor manager class.

		// Friend classes
		friend class BehaviourManager;
		friend class BehaviourLayer;
		friend class SensorBase;
	};

	/**
	* @class SensorBase
	*
	* @brief Implements a single sensor.
	*
	* This class is the base class of all sensors that are used in a SensorManager.
	**/
	class SensorBase
	{
	public:
		// Constructors
		SensorBase(SensorManager* SMBase, const std::string& signalName); //!< @brief Default constructor
		virtual ~SensorBase(); //!< @brief SensorBase object destructor.

		// Constant properties
		BehaviourManager* const MBase; //!< @brief Pointer to the parent behaviour manager.
		BehaviourLayer* const LBase; //!< @brief Pointer to the parent behaviour layer.
		SensorManager* const SMBase; //!< @brief Pointer to the parent sensor manager.
		const std::string signalName; //!< @brief The name of the actuator that this sensor should bind to. This name is used as the lookup key for the `BehaviourManager::findActuator()` function.

		// Initialisation function
	private:
		ret_t initBase(); //!< @brief Initialisation function for the sensor. This function is called from `SensorManager::initAllBase()`, and initialises the @e base sensor object.

	public:
		// Template type information
		virtual const std::type_info* getTypeInfo() const = 0; //!< @brief Abstract callback that should be overridden to return the type information of the sensor (i.e. a `std::type_info` object). Refer to the default override `Sensor::getTypeInfo()`.
		virtual bool isCompatibleWith(const ActuatorBase* ABase) const = 0; //!< @brief Abstract callback that should evaluate whether the current sensor is compatible the actuator @p ABase (can bind to it). Refer to the default override `Sensor::isCompatibleWith()`.

		// Data bind and update functions
	protected:
		virtual ret_t bindTo(const ActuatorBase* ABase) = 0; //!< @brief Abstract function that should be made to bind the sensor to the @p ABase actuator.
		virtual void getLatestData() = 0; //!< @brief Abstract function that should be made to retrieve the data from the bound actuator, and store it locally in the sensor.
	public:
		virtual bool isBound() const = 0; //!< @brief Abstract function that should be made to return whether this sensor has been bound to an actuator.

	public:
		// Get functions
		SensorBase* getBasePtr() { return this; } //!< @brief Return a pointer to the underlying SensorBase object in the case of a derived sensor class.

		// Friend classes
		friend class SensorManager;
	};

	/**
	* @class Sensor
	*
	* @brief Implements a single sensor of a given data type.
	*
	* This class is used to define the sensors of a SensorManager. The template parameter
	* specifies the desired data type of the sensor.
	**/
	template <class T>
	class Sensor : public SensorBase
	{
	public:
		// Constructors
		Sensor(SensorManager* SMBase, const std::string& signalName); //!< @brief Default constructor
		virtual ~Sensor(); //!< @brief Destructor

		// Template type information
		const std::type_info* const typeInfo;
		virtual const std::type_info* getTypeInfo() const { return typeInfo; }
		virtual bool isCompatibleWith(const ActuatorBase* ABase) const;

		// Data bind, update and read functions
	protected:
		virtual ret_t bindTo(const ActuatorBase* ABase); //!< @brief Bind to the actuator @p ABase. This must be called with an @p ABase that is actually pointing to an `Actuator<T>`, or no binding happens.
		virtual void getLatestData(); //!< @brief Get the latest data from the source actuator.
	public:
		virtual bool isBound() const { return (sourceAct != NULL); } //!< @brief Return whether this sensor has successfully been bound to an actuator.
		bool wasWrittenTo() const { return actWasWrittenTo; } //!< @brief Valid after every call to `getLatestData()` (called by `SensorManager::updateSensors()` during the update phase of a global step).
		const T& read() const { return data; } //!< @brief Get the data from the last sensor update (i.e. get the data that was most recently copied from the source actuator).

	private:
		// Internal variables
		const Actuator<T>* sourceAct; //!< @brief An internal pointer to the actuator that the sensor is bound to.
		T data; //!< @brief The value last read from the actuator by the sensor.
		bool actWasWrittenTo; //!< @brief A boolean flag
	};
}

#endif /* BEHAVIOUR_SENSORS_H */
// EOF