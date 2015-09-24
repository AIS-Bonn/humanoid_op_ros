// Behaviour Control Framework - Actuator classes
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file behaviour_actuators.h
* @brief Defines the actuator classes for the %Behaviour Control Framework.
**/

// Ensure header is only included once
#ifndef BEHAVIOUR_ACTUATORS_H
#define BEHAVIOUR_ACTUATORS_H

// Includes
#include <behaviour_control/classes/behaviour_common.h>

// Behaviour control namespace
namespace behaviourcontrol
{
	/**
	* @class ActuatorManager
	*
	* @brief Implements a manager of a particular group of actuators.
	*
	* This class is the base class of all actuator managers that are used in an architecture.
	**/
	class ActuatorManager
	{
	public:
		// Constants
		static const index_t DEF_ALIST_CAPACITY; //!< @brief Default capacity of the list that stores the child actuators of the actuator manager.

		// Constructors
		explicit ActuatorManager(BehaviourLayer* LBase); //!< @brief Default constructor
		virtual ~ActuatorManager(); //!< @brief %Actuator manager object destructor.

		// Constant properties
		BehaviourManager* const MBase; //!< @brief Pointer to the parent behaviour manager.
		BehaviourLayer* const LBase; //!< @brief Pointer to the parent behaviour layer.

		// Initialisation functions
	protected:
		virtual ret_t init() { return RET_OK; } //!< @brief Initialisation callback for the actuator manager. This function is called from `BehaviourLayer::initAll()`, and is intended to be overridden for the purpose of initialising the @e derived actuator manager object.
	private:
		ret_t initBase(); //!< @brief Initialisation function for the actuator manager. This function is called from `BehaviourLayer::initAllBase()`, and initialises the @e base actuator manager object.

	private:
		// Update function
		void updateActuators(); //!< @brief Internal update function for the actuator manager. This function is called from `BehaviourLayer::updateAll()`, which is in turn called from `BehaviourLayer::step()`.

		// Actuator declaration function
		template <class AClass> void declareActuator(AClass* actuator); //!< @brief Adds @p actuator as a child actuator of the actuator manager.

		// Actuator list
		std::vector<ActuatorBase*> AList; //!< @brief The list of child actuators of this actuator manager.

	public:
		// Read data from an external source (used only in interface layers)
		virtual void readExternalData() {} //!< @brief Interface layer specific callback that is intended to be overridden by the user to retrieve any available external data as required (e.g. from ROS).

		// Get functions
		ActuatorManager* getBasePtr() { return this; } //!< @brief Return a pointer to the underlying ActuatorManager class object in the case of a derived actuator manager class.

		// Friend classes
		friend class BehaviourManager;
		friend class BehaviourLayer;
		friend class ActuatorBase;
	};

	/**
	* @class ActuatorBase
	*
	* @brief Implements a single actuator.
	*
	* This class is the base class of all actuators that are used in an ActuatorManager.
	**/
	class ActuatorBase
	{
	public:
		// Constants
		static const std::string DEF_ACTUATOR_SIGNAL_NAME; //!< @brief Default actuator signal name used to generate a unique one, in the case that no name is provided by a derived actuator class.

		// Constructors
		explicit ActuatorBase(ActuatorManager* AMBase, const std::string& signalName = nullString); //!< @brief Default constructor
		virtual ~ActuatorBase(); //!< @brief ActuatorBase object destructor.

		// Constant properties
		BehaviourManager* const MBase; //!< @brief Pointer to the parent behaviour manager.
		BehaviourLayer* const LBase; //!< @brief Pointer to the parent behaviour layer.
		ActuatorManager* const AMBase; //!< @brief Pointer to the parent actuator manager.
		const std::string signalName; //!< @brief The name of the actuator, used as the lookup key for the `BehaviourManager::findActuator()` function.

	private:
		// Update function
		virtual void update() {}; //!< @brief Update callback for the ActuatorBase object. Refer to the default override defined in `Actuator::update()`.

	public:
		// Template type information
		virtual const std::type_info* getTypeInfo() const = 0; //!< @brief Abstract callback that should be overridden to return the type information of the actuator (i.e. a `std::type_info` object). Refer to the default override `Actuator::getTypeInfo()`.
		virtual bool isCompatibleWith(const SensorBase* SBase) const = 0; //!< @brief Abstract callback that should evaluate whether the current actuator is compatible the sensor @p SBase (can be bound to it). Refer to the default override `Actuator::isCompatibleWith()`.

		// Get functions
		std::string getUniqueName(const ActuatorManager* AMBase) const; //!< @brief Returns a unique actuator name.
		ActuatorBase* getBasePtr() { return this; } //!< @brief Return a pointer to the underlying ActuatorBase object in the case of a derived actuator class.

		// Friend classes
		friend class ActuatorManager;
	};

	/**
	* @class Actuator
	*
	* @brief Implements a single actuator of a given data type.
	*
	* This class is used to define the actuators of an ActuatorManager. The template parameter
	* specifies the desired data type of the actuator.
	*
	* The template type T must have a default constructor that initialises the data to an
	* equivalent of 'zero', and it must have a well-defined `operator=` that performs a
	* <i>deep copy</i> of the data. Both of these criteria are satisfied for native types.
	* Note that if an actuator is not written to within a cycle, it retains its value from
	* the previous iteration.
	*
	* In order for an actuator to be able to be aggregatable, the template type T must
	* satisfy two additional properties. Firstly, there must be a multiplication
	* operator defined between it and a #level_t (a floating point type, generally @c float),
	* and secondly there must be an addition operator defined between two objects of type T.
	* To make an actuator aggregatable, simply pass #AGGREGATABLE as the third parameter of
	* the Actuator constructor.
	**/
	template <class T>
	class Actuator : public ActuatorBase
	{
	public:
		// Constructors
		explicit Actuator(ActuatorManager* AMBase, const std::string& signalName = nullString, bool aggregatable = AGGREGATABLE); //!< @brief Default constructor
		virtual ~Actuator(); //!< @brief Actuator object destructor.

		// Template type information
		const bool aggregatable; //!< @brief Flag that specifies whether the actuator is aggregatable.
		const std::type_info* const typeInfo; //!< @brief The type information of the actuator data type.
		virtual const std::type_info* getTypeInfo() const { return typeInfo; }
		virtual bool isCompatibleWith(const SensorBase* SBase) const;

		// Data read and write functions
		const T& read() const { return data; } //!< @brief Get function for the current data contained in the actuator.
		void write(const T& newdata, const Behaviour* BBase); //!< @brief General purpose write function to be used by behaviours to write to non-aggregatable actuators. Do not use this function on aggregatable actuators.
		void writeAgg(const T& newdata, const Behaviour* BBase); //!< @brief General purpose write function to be used by behaviours to write to aggregatable actuators. Do not use this function on non-aggregatable actuators.
		void writeHard(const T& newdata); //!< @brief Special write function that can be used during initialisation to set up the value of an actuator. This is the only write function that can be used by an interface layer, as the other two variants require the passing of a pointer to the source behaviour (and interface layers can't have child behaviours).
		bool wasWrittenTo() const { return weightSum > 0.0; } //!< @brief Returns whether the actuator has been written to since the last update (`update()` is called at the beginning of each step).

	private:
		// Update function
		virtual void update() { weightSum = 0.0; lastModifier = NULL; };

		// Internal variables
		T data; //!< @brief The current value of the actuator.
		level_t weightSum; //!< @brief A running sum of the activation levels of the behaviours that write to the actuator.
		const Behaviour* lastModifier; //!< @brief A pointer to the behaviour that last wrote to the actuator, used to detect multiple writes by the same behaviour (frowned upon).
	};
}

#endif /* BEHAVIOUR_ACTUATORS_H */
// EOF