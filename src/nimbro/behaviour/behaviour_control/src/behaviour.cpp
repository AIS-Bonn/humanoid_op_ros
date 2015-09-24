// Behaviour Control Framework - Behaviour class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <behaviour_control/behaviour_control.h>

// Namespaces
using namespace std;
using namespace behaviourcontrol;

// Constants
const index_t Behaviour::DEF_ILIST_CAPACITY = 16;
const std::string Behaviour::DEF_BEHAVIOUR_NAME = "Behaviour";

//
// Constructors
//

// Default constructor
/**
* Note that although the default value for the @p name parameter is @c nullString, this will not
* actually become the behaviour's name if the parameter is omitted. This is just used to signal
* to the class's internals that it should generate a default name using the `getUniqueName()`
* function. @p LBase must never be null or a segmentation fault will result!
*
* @param LBase A pointer to the behaviour layer that the behaviour belongs to.
* @param name The human-friendly name to give the behaviour (null means auto-generate).
**/
Behaviour::Behaviour(BehaviourLayer* LBase, const std::string& name)
	: MBase(LBase->MBase)
	, LBase(LBase)
	, Mname(LBase->Mname)
	, Lname(LBase->name)
	, name(name.empty() ? getUniqueName(LBase) : name)
	, justActivated(true)
	, justDeactivated(true)
	, rawA(0.0)
	, trueA(0.0)
{
	// Initialise list capacities
	inhibits.reserve(DEF_ILIST_CAPACITY);
	isInhibitedBy.reserve(DEF_ILIST_CAPACITY);

	// Ensure that the parent is not an interface layer
	ASSERT_ERROR(!LBase->isInterface, MBase, "Failed to declare behaviour '" + this->name + "' as a child of layer '" + LBase->name + "' - Interface layers cannot contain child behaviours!");

	// Automatically declare the behaviour in the containing BehaviourLayer
	LBase->declareBehaviour(this);
}

// Destructor
Behaviour::~Behaviour()
{
}

//
// Inhibition lists
//

// Function: Behaviour::addInhibition()
/**
* Mutually add an inhibition between two behaviours (A inhibits B).
* Note that you need to be super sure that no null pointer creeps into this function (the null
* checking in `BehaviourLayer::addInhibition()` and `BehaviourLayer::addChainInhibition()`
* *should* suffice).
*
* @param BBaseA A non-null pointer to the behaviour that is inhibiting.
* @param BBaseB A non-null pointer to the behaviour that is being inhibited.
**/
void Behaviour::addInhibition(Behaviour* BBaseA, Behaviour* BBaseB) // 
{
	// Add BBaseB to the BBaseA->inhibits list
	addIfUnique(BBaseA->inhibits, BBaseB);

	// Add BBaseA to the BBaseB->isInhibitedBy list
	addIfUnique(BBaseB->isInhibitedBy, BBaseA);
}

//
// Get functions
//

// Function: Behaviour::getUniqueName()
/**
* Automatically construct a unique name for the behaviour, for situations where none is provided
* by the derived class. The name that results of this function will be unique as long as the
* user does not explicitly use the same name somewhere else, either before or after the construction
* of this behaviour. If there are currently four behaviours in the @p LBase layer, the generated
* name will be the concatenation of @ref Behaviour::DEF_BEHAVIOUR_NAME "DEF_BEHAVIOUR_NAME" and "4".
*
* @param LBase A non-null pointer to the parent behaviour layer.
* @return The required behaviour name.
**/
std::string Behaviour::getUniqueName(const BehaviourLayer* LBase) const
{
	// Return the required name
	std::ostringstream out;
	out << LBase->name << "/" << DEF_BEHAVIOUR_NAME << LBase->BList.size();
	return out.str();
}
// EOF