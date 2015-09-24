// Behaviour Control Framework - Behaviour class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file behaviour.h
* @brief Defines the Behaviour class for the %Behaviour Control Framework.
**/

// Ensure header is only included once
#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

// Includes
#include <behaviour_control/classes/behaviour_common.h>

// Behaviour control namespace
namespace behaviourcontrol
{
	/**
	* @class Behaviour
	*
	* @brief Implements a single behaviour.
	*
	* This class is the base class of all behaviours that are used in an architecture.
	**/
	class Behaviour
	{
	public:
		// Constants
		static const index_t DEF_ILIST_CAPACITY; //!< @brief Default capacity of the inhibition lists that are used to store which behaviours are inhibitors and inhibitees of this behaviour.
		static const std::string DEF_BEHAVIOUR_NAME; //!< @brief Default behaviour name used to generate a unique one, in the case that no name is provided by a derived behaviour class.

		// Constructors
		explicit Behaviour(BehaviourLayer* LBase, const std::string& name = nullString); //!< @brief Default constructor.
		virtual ~Behaviour(); //!< @brief %Behaviour object destructor.

		// Constant properties
		BehaviourManager* const MBase; //!< @brief Pointer to the parent behaviour manager.
		BehaviourLayer* const LBase; //!< @brief Pointer to the parent behaviour layer.
		const std::string Mname; //!< @brief Human-friendly string name of the parent behaviour manager.
		const std::string Lname; //!< @brief Human-friendly string name of the parent behaviour layer.
		const std::string name; //!< @brief Human-friendly string name of the behaviour.

	private:
		// Inhibition lists
		static void addInhibition(Behaviour* BBaseA, Behaviour* BBaseB); //!< @brief Updates the respective internal behaviour inhibitor and inhibitee lists to reflect that @p BBaseA inhibits @p BBaseB.
		std::vector<Behaviour*> inhibits; //!< @brief A list of the behaviours that this behaviour inhibits (inhibitees).
		std::vector<Behaviour*> isInhibitedBy; //!< @brief A list of the behaviours that inhibit this behaviour (inhibitors).

		// Initialisation functions
	protected:
		virtual ret_t init() { return RET_OK; } //!< @brief Initialisation callback for the behaviour. This function is called from `BehaviourLayer::initAll()`, and is intended to be overridden for the purpose of initialising the @e derived behaviour object.

		// Update and execute functions
	protected:
		virtual void update() {} //!< @brief Update callback for the behaviour object. This function is called from `BehaviourLayer::updateAll()`, which is used in `BehaviourLayer::step()`.
		virtual void execute() {} //!< @brief Execute callback for the behaviour object. This function is called from `BehaviourLayer::executeAll()`, which is used in `BehaviourLayer::step()`.
		virtual void inhibited() {} //!< @brief A callback for the behaviour object that is invoked instead of `execute()` when the behaviour is deactivated. This function is called from `BehaviourLayer::executeAll()`, which is used in `BehaviourLayer::step()`.
		bool wasJustActivated() const { return justActivated; } //!< @brief Returns true when called from the `execute()` callback, and the behaviour was just activated. The return value is always true when called from `inhibited()`.
		bool wasJustDeactivated() const { return justDeactivated; } //!< @brief Returns true when called from the `inhibited()` callback, and the behaviour was just inhibited. The return value is always true when called from `execute()`.
	private:
		bool justActivated; //!< @brief Internal variable for the `wasJustActivated()` function.
		bool justDeactivated; //!< @brief Internal variable for the `wasJustDeactivated()` function.

		// Activation levels
	public:
		level_t getA() const { return trueA; } //!< @brief Returns the current @e true activation level of the behaviour. The return value is valid only when called from the `execute()`, `inhibited()` or `BehaviourLayer::postExecuteCallback()` function.
	protected:
		virtual level_t computeActivationLevel() { return 0.0; } //!< @brief Callback for the computation of the raw behaviour activation level. Valid return values are @c false, @c true, @ref behaviourcontrol::LVL_INACTIVE "LVL_INACTIVE", @ref behaviourcontrol::LVL_ACTIVE "LVL_ACTIVE" and floating point numbers between 0 and 1 inclusive (values outside this range are coerced).
	private:
		level_t rawA; //!< @brief Internal variable used to store the raw activation level.
		level_t trueA; //!< @brief Internal variable used to store the true activation level.

	public:
		// Get functions
		std::string getUniqueName(const BehaviourLayer* LBase) const; //!< @brief Returns a unique behaviour name.
		Behaviour* getBasePtr() { return this; } //!< @brief Return a pointer to the underlying Behaviour class object in the case of a derived behaviour class.

		// Friend classes
		friend class BehaviourLayer;
	};
}

#endif /* BEHAVIOUR_H */
// EOF