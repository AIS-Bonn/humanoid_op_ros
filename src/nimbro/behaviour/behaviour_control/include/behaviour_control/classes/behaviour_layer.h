// Behaviour Control Framework - BehaviourLayer class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file behaviour_layer.h
* @brief Defines the BehaviourLayer class for the %Behaviour Control Framework.
**/

// Ensure header is only included once
#ifndef BEHAVIOUR_LAYER_H
#define BEHAVIOUR_LAYER_H

// Includes
#include <behaviour_control/classes/behaviour_common.h>

// Behaviour control namespace
namespace behaviourcontrol
{
	/**
	* @class BehaviourLayer
	*
	* @brief Implements a single behaviour layer.
	*
	* This class is the base class of all behavioural layers that are used in an architecture.
	**/
	class BehaviourLayer
	{
	public:
		// Constants
		static const index_t DEF_BLIST_CAPACITY; //!< @brief Default capacity of the list that stores the child behaviours of the layer.
		static const index_t DEF_ILIST_CAPACITY; //!< @brief Default capacity of the lists that are used to store the chaining and non-chaining inhibition definitions.
		static const std::string DEF_LAYER_NAME; //!< @brief Default behaviour layer name used to generate a unique one, in the case that no name is provided by a derived behaviour layer class.

		// Structures
		struct BBasePair
		{
			BBasePair() : A(NULL), B(NULL) {} //!< Default constructor.
			BBasePair(Behaviour* BBaseA, Behaviour* BBaseB) : A(BBaseA), B(BBaseB) {} //!< Constructor that accepts initial values for @p BBaseA and @p BBaseB.
			Behaviour* A; //!< The inhibiting behaviour.
			Behaviour* B; //!< The behaviour that is being inhibited.
		}; //!< @brief Data structure used to represent a basic inhibition pair, where behaviour A is taken to inhibit behaviour B.

		// Constructors
		explicit BehaviourLayer(BehaviourManager* MBase, const std::string& name = nullString, bool isInterface = false); //!< @brief Default constructor.
		virtual ~BehaviourLayer(); //!< @brief Behaviour layer object destructor.

		// Constant properties
		BehaviourManager* const MBase; //!< @brief Pointer to the parent behaviour manager.
		const std::string Mname; //!< @brief Human-friendly string name of the parent behaviour manager.
		const std::string name; //!< @brief Human-friendly string name of the behaviour layer.
		const bool isInterface; //!< @brief A boolean flag specifying whether the layer is an interface layer or a normal behaviour layer.

	private:
		// Sensor and actuator managers
		void setSensorManager(SensorManager* SMBase); //!< @brief Set function for the child sensor manager.
		void setActuatorManager(ActuatorManager* AMBase); //!< @brief Set function for the child actuator manager.
		SensorManager*   SMBase; //!< @brief Internal pointer to the child sensor manager.
		ActuatorManager* AMBase; //!< @brief Internal pointer to the child actuator manager.
		bool haveSM; //!< @brief Internal boolean flag specifying whether this layer has a sensor manager.
		bool haveAM; //!< @brief Internal boolean flag specifying whether this layer has an actuator manager.

		// Inhibition functions
	protected:
		void addInhibition(Behaviour* BBaseA, Behaviour* BBaseB); //!< @brief Adds a non-chaining inhibition from behaviour @p BBaseA (inhibitor) to behaviour @p BBaseB (inhibitee).
		void addChainInhibition(Behaviour* BBaseA, Behaviour* BBaseB); //!< @brief Adds a chaining inhibition from behaviour @p BBaseA (inhibitor) to behaviour @p BBaseB (inhibitee).
	private:
		std::vector<BBasePair> IList; //!< @brief Internal list of non-chaining inhibitions defined for the layer.
		std::vector<BBasePair> CIList; //!< @brief Internal list of chaining inhibitions defined for the layer.

	protected:
		// Initialisation function
		virtual ret_t init() { return RET_OK; } //!< @brief Initialisation callback for the behaviour layer. This function is called from `initAll()`, and is intended to be overridden for the purpose of initialising the @e derived behaviour layer object.

	private:
		// Internal initialisation functions
		ret_t initAll(); //!< @brief Function that initialises the @e derived classes of the whole behaviour layer, including all child objects. This function is called from `BehaviourManager::initialiseArchitecture()`.
		ret_t initAllBase(); //!< @brief Function that initialises the @e base classes of the whole behaviour layer, including all child objects. This function is called from `BehaviourManager::initialiseArchitecture()`.
		ret_t initBase(); //!< @brief Initialisation function for the behaviour layer. This function is called from `initAllBase()`, and initialises the @e base behaviour layer object.

	public:
		// Get functions
		SensorManager*   getSMBase() { return (haveSM ? SMBase : NULL); } //!< @brief Get function for a pointer to the base class of the child sensor manager.
		ActuatorManager* getAMBase() { return (haveAM ? AMBase : NULL); } //!< @brief Get function for a pointer to the base class of the child actuator manager.
		bool hasSM() const { return haveSM; } //!< @brief Boolean flag specifying whether this layer has a sensor manager.
		bool hasAM() const { return haveAM; } //!< @brief Boolean flag specifying whether this layer has an actuator manager.

		// Step functions
	protected:
		virtual void update() {} //!< @brief Update callback for the behaviour layer object. This function is called from `updateAll()`, which is used in `step()`.
		virtual void postExecuteCallback() {} //!< @brief Callback that is invoked at the end of the `executeAll()` function so that the user can inject code.
	private:
		void step(); //!< @brief Internal step function of the layer.
		void updateAll(); //!< @brief Updates the layer and all of its child objects.
		void executeAll(); //!< @brief Executes the layer and all of its child objects.

	private:
		// Actuator functions
		const ActuatorBase* findActuator(const std::string& signalName) const; //!< @brief Used by the `BehaviourManager::findActuator()` function to search this layer for an actuator of the given name i.e. @p signalName.

		// Behaviour declaration function
		template <class BClass> void declareBehaviour(BClass* behaviour); //!< @brief Adds @p behaviour as a child behaviour of the layer.

		// Behaviour list
		std::vector<Behaviour*> BList; //!< @brief The list of child behaviours of this layer.

	public:
		// Get functions
		std::string getUniqueName(const BehaviourManager* MBase) const; //!< @brief Returns a unique behaviour layer name.
		BehaviourLayer* getBasePtr() { return this; } //!< @brief Return a pointer to the underlying BehaviourLayer class object in the case of a derived behaviour layer class.

		// Friend classes
		friend class BehaviourManager;
		friend class SensorManager;
		friend class ActuatorManager;
		friend class Behaviour;
	};
}

#endif /* BEHAVIOUR_LAYER_H */
// EOF