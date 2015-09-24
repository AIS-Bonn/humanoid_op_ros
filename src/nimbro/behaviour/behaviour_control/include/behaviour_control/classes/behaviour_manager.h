// Behaviour Control Framework - BehaviourManager class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file behaviour_manager.h
* @brief Defines the BehaviourManager class for the %Behaviour Control Framework.
**/

// Ensure header is only included once
#ifndef BEHAVIOUR_MANAGER_H
#define BEHAVIOUR_MANAGER_H

// Includes
#include <behaviour_control/classes/behaviour_common.h>

// Behaviour control namespace
namespace behaviourcontrol
{
	/**
	* @class BehaviourManager
	*
	* @brief Implements a single behaviour manager.
	*
	* This class implements a base class for all behaviour managers that are used in an architecture.
	* In a single process application typically only one behaviour manager is required.
	**/
	class BehaviourManager
	{
	public:
		// Constants
		static const index_t DEF_LLIST_CAPACITY; //!< @brief Default capacity of the list that stores the child behaviour layers of the manager.
		static const std::string DEF_MANAGER_NAME; //!< @brief Default behaviour manager name used to generate a unique one, in the case that no name is provided by a derived behaviour manager class.

		// Constructors
		explicit BehaviourManager(const std::string& name = nullString); //!< @brief Default constructor.
		virtual ~BehaviourManager(); //!< @brief Behaviour manager object destructor.

		// Constant properties
		const std::string name; //!< @brief Human-friendly string name of the behaviour manager.

		// Initialisation functions
	public:
		ret_t initialiseArchitecture(); //!< @brief Function that should be called by the user to initialise the entire behaviour manager architecture, including all child layers, behaviours, sensor managers, actuator managers, and so on.
		bool wasInitialised() const { return initialised; } //!< @brief Boolean flag specifying whether the manager has been initialised yet or not.
		bool isInitialising() const { return initialising; } //!< @brief Boolean flag specifying whether the manager is currently initialising or not.
	protected:
		virtual ret_t init() { return RET_OK; } //!< @brief Initialisation callback for the behaviour manager. This function is called from `initialiseArchitecture()`, and is intended to be overridden for the purpose of initialising the @e derived behaviour manager object.
	private:
		ret_t initBase(); //!< @brief Initialisation function for the behaviour manager. This function is called from `initialiseArchitecture()`, and initialises the @e base behaviour manager object.
		bool initialised; //!< @brief Internal boolean flag specifying whether the manager has been initialised yet or not.
		bool initialising; //!< @brief Internal boolean flag specifying whether the manager is currently initialising or not.

		// Step functions
	public:
		void step(); //!< @brief Function that should be called by the user to execute one step of the entire behaviour manager architecture.
		int cycleID() const { return icycleID; } //!< @brief The numeric ID of the current step cycle of the behaviour manager. Prior to the first step this ID is 0. The ID is incremented by one in each call to `step()`.
	protected:
		virtual void preStepCallback() {} //!< @brief Callback that is invoked at the beginning of the `step()` function so that the user can inject code.
		virtual void postStepCallback() {} //!< @brief Callback that is invoked at the end of the `step()` function so that the user can inject code.
	private:
		int icycleID; //!< @brief The internal numeric ID of the current step cycle of the behaviour manager.

		// Error notification functions
	public:
		void reportError(const std::string& msg, bool fatal, const std::string& funcName, const std::string& fileName, int line); //!< @brief Function used to signal that a warning or an error has occurred. After setting the appropriate error flags, this function invokes the `reportErrorUser()` function. It is recommended to use this function only indirectly via the @ref #REPORT_ERROR "Error Macros".
		virtual void reportErrorUser(const std::string& msg, bool fatal, const std::string& funcName, const std::string& fileName, int line) {} //!< @brief Error handler function that is intended to be overridden by the user to process and/or display warnings and errors raised by the framework. A non-fatal error is a warning.
		void clearErrorStatus(); //!< @brief Reset the error status flags so that functions like `initialiseArchitecture()` and `step()` can be attempted to be called again.
		bool hadError() const { return fatalErrorOccurred; } //!< @brief Boolean flag that specifies whether an error has occurred with the architecture.
		bool hadWarning() const { return nonfatalErrorOccurred; } //!< @brief Boolean flag that specifies whether an error or warning has occurred with the architecture.
	private:
		bool fatalErrorOccurred; //!< @brief Internal boolean flag that specifies whether an error has occurred with the architecture.
		bool nonfatalErrorOccurred; //!< @brief Internal boolean flag that specifies whether an error or warning has occurred with the architecture.

	public:
		// Helper functions
		std::string toString(const std::string& linePrefix = nullString) const; //!< @brief Generates a multiline string representation of the architecture, summarising all of the main components.

		// Get functions
		const ActuatorBase* findActuator(const std::string& signalName) const; //!< @brief Finds an actuator within the architecture, given its string name as a lookup key.
		BehaviourManager* getBasePtr() { return this; } //!< @brief Return a pointer to the underlying BehaviourManager class object in the case of a derived behaviour manager class.

	private:
		// Layer declaration function
		template <class LClass> void declareLayer(LClass* layer); //!< @brief Adds @p layer as a child layer of the manager.

		// Layer list
		std::vector<BehaviourLayer*> LList; //!< @brief The list of child layers of this manager.
		
		// Friend classes
		friend class BehaviourLayer;
		friend class SensorBase;
	};
}

#endif /* BEHAVIOUR_MANAGER_H */
// EOF