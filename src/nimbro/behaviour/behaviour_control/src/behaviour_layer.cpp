// Behaviour Control Framework - BehaviourLayer class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <behaviour_control/behaviour_control.h>
#include <signal.h>

// Namespaces
using namespace std;
using namespace behaviourcontrol;

// Constants
const index_t BehaviourLayer::DEF_BLIST_CAPACITY = 16;
const index_t BehaviourLayer::DEF_ILIST_CAPACITY = 32;
const std::string BehaviourLayer::DEF_LAYER_NAME = "BehaviourLayer";

//
// Constructors
//

// Default constructor
/**
* Note that although the default value for the @p name parameter is @c nullString, this will not
* actually become the behaviour layer's name if the parameter is omitted. This is just used to signal
* to the class's internals that it should generate a default name using the `getUniqueName()` function.
* @p MBase must never be null or a segmentation fault will result!
*
* @param MBase A pointer to the behaviour manager that the behaviour layer belongs to.
* @param name The human-friendly name to give the behaviour layer (null means auto-generate).
* @param isInterface A boolean flag specifying whether the layer is an interface layer. This changes
* what you are allowed to do with the class in some situations, and ensures that the execution order
* is right for an external interface.
**/
BehaviourLayer::BehaviourLayer(BehaviourManager* MBase, const std::string& name, bool isInterface)
	: MBase(MBase)
	, Mname(MBase->name)
	, name(name.empty() ? getUniqueName(MBase) : name)
	, isInterface(isInterface)
	, SMBase(NULL)
	, AMBase(NULL)
	, haveSM(false)
	, haveAM(false)
{
	// Initialise list capacities
	BList.reserve(DEF_BLIST_CAPACITY);
	IList.reserve(DEF_ILIST_CAPACITY);
	CIList.reserve(DEF_ILIST_CAPACITY);

	// Automatically declare the behaviour layer in the containing BehaviourManager
	MBase->declareLayer(this);
}

// Destructor
BehaviourLayer::~BehaviourLayer()
{
}

//
// Sensor and actuator managers
//

// Sensor manager set-function
void BehaviourLayer::setSensorManager(SensorManager* SMBase)
{
	// Update the internal SensorManager pointer
	if(!haveSM && (SMBase != NULL) && !MBase->wasInitialised())
	{
		this->SMBase = SMBase;
		haveSM = true;
	}
}

// Actuator manager set-function
void BehaviourLayer::setActuatorManager(ActuatorManager* AMBase)
{
	// Update the internal ActuatorManager pointer
	if(!haveAM && (AMBase != NULL) && !MBase->wasInitialised())
	{
		this->AMBase = AMBase;
		haveAM = true;
	}
}

//
// Inhibition functions
//

// Function: BehaviourLayer::addInhibition()
/**
* Add a non-chaining (non-transitive) inhibition from behaviour A to behaviour B. Neither
* behaviour pointer should be null, the behaviours pointers can't be equal, and both
* behaviours should belong to the behaviour layer for which this function is called.
* This function can only be used in the constructor of the derived layer class.
*
* @param BBaseA A non-null pointer to the behaviour that is inhibiting.
* @param BBaseB A non-null pointer to the behaviour that is being inhibited.
**/
void BehaviourLayer::addInhibition(Behaviour* BBaseA, Behaviour* BBaseB)
{
	// Error checking
	ASSERT_ERROR(((BBaseA != NULL) && (BBaseB != NULL)), MBase, "Failed to add requested inhibition for layer '" + name + "' - One or more of the passed behaviour pointers was null!");
	ASSERT_ERROR(BBaseA != BBaseB, MBase, "Failed to add requested inhibition for layer '" + name + "' - Behaviour '" + BBaseA->name + "' cannot inhibit itself!");
	ASSERT_WARNING(!MBase->isInitialising(), MBase, "Attempted to add an inhibition ('" + BBaseA->name + " => " + BBaseB->name + "' in layer '" + name + "') during initialisation - Inhibitions must be added in the layer class constructor!");
	ASSERT_WARNING(!MBase->wasInitialised(), MBase, "Attempted to add an inhibition ('" + BBaseA->name + " => " + BBaseB->name + "' in layer '" + name + "') after initialisation - Inhibitions must be added in the layer class constructor!");
	ASSERT_WARNING(((BBaseA->LBase == this) && (BBaseB->LBase == this)), MBase, "Attempted to add an inhibition ('" + BBaseA->name + " => " + BBaseB->name + "' in layer '" + name + "') involving behaviour(s) that belong to a different layer!");

	// Add the requested inhibition to the layer's (non-transitive) inhibition list
	BBasePair tmp(BBaseA, BBaseB);
	IList.push_back(tmp); // Invoke default copy constructor (shallow copy)
}

// Function: BehaviourLayer::addChainInhibition()
/**
* Add a chaining (transitive) inhibition from behaviour A to behaviour B. Neither
* behaviour pointer should be null, the behaviours pointers can't be equal, and both
* behaviours should belong to the behaviour layer for which this function is called.
* This function can only be used in the constructor of the derived layer class.
*
* @param BBaseA A non-null pointer to the behaviour that is inhibiting.
* @param BBaseB A non-null pointer to the behaviour that is being inhibited.
**/
void BehaviourLayer::addChainInhibition(Behaviour* BBaseA, Behaviour* BBaseB)
{
	// Error checking
	ASSERT_ERROR(((BBaseA != NULL) && (BBaseB != NULL)), MBase, "Failed to add requested chain inhibition for layer '" + name + "' - One or more of the passed behaviour pointers was null!");
	ASSERT_ERROR(BBaseA != BBaseB, MBase, "Failed to add requested chain inhibition for layer '" + name + "' - Behaviour '" + BBaseA->name + "' cannot inhibit itself!");
	ASSERT_WARNING(!MBase->isInitialising(), MBase, "Attempted to add a chain inhibition ('" + BBaseA->name + " => " + BBaseB->name + "' in layer '" + name + "') during initialisation - Inhibitions must be added in the layer class constructor!");
	ASSERT_WARNING(!MBase->wasInitialised(), MBase, "Attempted to add a chain inhibition ('" + BBaseA->name + " => " + BBaseB->name + "' in layer '" + name + "') after initialisation - Inhibitions must be added in the layer class constructor!");
	ASSERT_WARNING(((BBaseA->LBase == this) && (BBaseB->LBase == this)), MBase, "Attempted to add a chain inhibition ('" + BBaseA->name + " => " + BBaseB->name + "' in layer '" + name + "') involving behaviour(s) that belong to a different layer!");

	// Add the requested inhibition to the layer's chain inhibition list
	BBasePair tmp(BBaseA, BBaseB);
	CIList.push_back(tmp); // Invoke default copy constructor (shallow copy)
}

//
// Initialisation functions
//

/**
* This function calls the `init()` functions of itself and its children to perform the initialisation.
* The function first initialises itself (the layer object), then the child sensor manager (if it exists),
* then the child actuator manager (if it exists), and finally the child behaviours, in the order that
* they are stored in the internal behaviours list.
* 
* @return A @c ret_t type that specifies whether all of the initialisation functions were successful.
**/
ret_t BehaviourLayer::initAll()
{
	// Declare variables
	ret_t ret;
	index_t i;

	// Initialise the user's derived behaviour layer
	ret = init();
	if(ret != RET_OK) return ret;

	// Initialise the user's derived sensor manager
	if(haveSM)
	{
		ret = SMBase->init();
		if(ret != RET_OK) return ret;
	}

	// Initialise the user's derived actuator manager
	if(haveAM)
	{
		ret = AMBase->init();
		if(ret != RET_OK) return ret;
	}

	// Initialise the user's derived child behaviours
	for(i = 0;i < BList.size();i++)
	{
		ret = BList[i]->init();
		if(ret != RET_OK) return ret;
	}

	// Success if we got this far
	return RET_OK;
}

// Initialise the behaviour layer internals, and the internals of all the child behaviours
ret_t BehaviourLayer::initAllBase()
{
	// Declare variables
	ret_t ret;

	// Initialise the internals of the behaviour layer
	ret = initBase();
	if(ret != RET_OK) return ret;

	// Initialise the internals of the sensor manager
	if(haveSM)
	{
		ret = SMBase->initAllBase();
		if(ret != RET_OK) return ret;
	}

	// Initialise the internals of the actuator manager
	if(haveAM)
	{
		ret = AMBase->initBase();
		if(ret != RET_OK) return ret;
	}

	// Success if we got this far
	return RET_OK;
}

/**
* Initialises the internals of the behaviour layer. The main task of this function is to process the
* inhibition definitions that were made in the derived class constructor using the `addInhibition()`
* and `addChainInhibition()` functions. These definitions are retrieved from the `IList` and `CIList`
* class members, and used to populate the inhibitor and inhibitee lists inside the individual
* behaviour objects. The chaining inhibitions are then resolved into their equivalent sets of
* non-chaining inhibitions, and are combined with the non-chaining inhibitions into one long list.
* A topological sort is then performed on the behaviours list with respect to the directed graph
* on the behaviours given by the long list of non-chaining inhibitions. The `BList` member is modified
* to match this topological ordering, as this becomes the desired order to evaluate the behaviours in
* during stepping of the layer. If a cycle is detected in the inhibition definitions during the
* topological sort, an error is reported, a cycle is generated and displayed, and this function returns
* `RET_INHIBITION_CYCLE`.
*
* @return A @c ret_t type that specifies whether the function was successful.
**/
ret_t BehaviourLayer::initBase()
{
	// Declare variables
	std::vector<BBasePair> E;
	std::vector<Behaviour*> V;
	std::vector<Behaviour*> L;
	Behaviour* Btmp;
	index_t i, j, k;
	BBasePair tmp;
	bool found;

	// Clear all the inhibition lists inside the behaviours
	for(i = 0;i < BList.size();i++)
	{
		BList[i]->inhibits.clear();
		BList[i]->isInhibitedBy.clear();
	}

	// Add the non-transitive inhibitions from IList directly to the E vector and the behaviour classes
	for(i = 0;i < IList.size();i++)
	{
		E.push_back(IList[i]);
		Behaviour::addInhibition(IList[i].A, IList[i].B);
	}

	// Elaborate the chain inhibitions to get the required set of equivalent non-transitive inhibitions
	for(i = 0;i < BList.size();i++) // For each behaviour in the layer...
	{
		// Reset vector V to just contain the nominated root behaviour
		V.assign(1, BList[i]);

		// Loop until no more behaviours belonging to the chain are found (note that V grows in the loop)
		for(j = 0;j < V.size();j++)
		{
			for(k = 0;k < CIList.size();k++) // For all defined chain inhibitions...
			{
				if(CIList[k].A == V[j]) // If we have a chain inhibition child of the behaviour we are currently looking at in V...
					addIfUnique(V, CIList[k].B);
			}
		}

		// Add non-transitive inhibitions between the root behaviour and each of the discovered (direct or indirect) chain children
		tmp.A = BList[i];
		for(j = 1;j < V.size();j++)
		{
			tmp.B = V[j];
			E.push_back(tmp);
			Behaviour::addInhibition(BList[i], V[j]); // BList[i] == V[0] so this cannot be adding a self-referencing inhibition
		}
	}

	// Note: At this point we have reduced all specified inhibitions into a collection of basic non-transitive
	//       inhibitions, stored as BBasePair's in vector E. These define a directed graph on the behaviours in
	//       the layer. The vector is called E because these define the edges of the directed graph. The vertices
	//       of the graph are the individual behaviours in the layer, stored in the BList vector. The inhibition
	//       information is at this point also stored internally in the individual behaviours, in the 'inhibits'
	//       and 'isInhibitedBy' vectors. These are now trusted sources of information on what inhibits what.
	//       Note that these lists don't contain any duplicates and are internally consistent due to the exclusive
	//       use of the addIfUnique() and Behaviour::addInhibition() functions above. We now perform a topological
	//       sort on the directed behaviour inhibition graph to get an ordering of the behaviours in the layer that
	//       will result in the correct inhibition behaviour.

	// We are re-using the variable V for a different purpose now...
	V.clear();

	// Construct a list of all the nodes that are not inhibited by any other behaviours
	for(i = BList.size();i > 0;i--) // Reversed as the topological sort naturally tends to reverse the order of unconnected vertices
	{
		if(BList[i-1]->isInhibitedBy.empty())
			V.push_back(BList[i-1]);
	}

	// Perform the required topological sort and store the output in the L vector
	while(!V.empty())
	{
		// Pick and remove a behaviour from V, and move it into L (ok to do as it is not inhibited by anything that isn't already in L)
		Btmp = V.back();
		V.pop_back();
		L.push_back(Btmp);

		// Remove all edges where Btmp is the inhibitor from E
		for(i = 0;i < E.size();i++)
		{
			if(E[i].A == Btmp)
			{
				found = false;
				for(j = 0;j < E.size();j++)
				{
					if((E[i].B == E[j].B) && (i != j)) // If still inhibited by something other than the edge we're about to remove...
						found = true;
				}
				if(!found) V.push_back(E[i].B);
				E.erase(E.begin() + i--); // Need to post-decrement i to counteract the i++ in the for-loop as we just erased the element we were looking at, so the next element is now in the same position as we were just looking
			}
		}
	}

	// If there are still edges in E then we have ourselves a cycle!
	if(!E.empty())
	{
		// Note: Often the number of inhibitions left in E can still be pretty large, so just displaying all of them just helps a little.
		//       Put in a little bit more work and a few heuristics and we can in many circumstances get a much simpler circuit.
		//       Note that a lot of this is unnecessarily ugly because we don't have access to something like 'inhibits' and 'isInhibitedBy'.
		//       Regenerating that and then keeping it up to date with pruning etc would be far more complication than it's worth.

		// Declare variables
		std::vector<Behaviour*> C;
		std::vector<int> numOut;
		std::vector<int> numIn;
		Behaviour* startBeh;
		Behaviour* maxBeh;
		int maxval;

		// Build a list of the behaviours that still have edges floating around
		V.clear(); // Once again being used for a new purpose
		for(i = 0;i < E.size();i++)
		{
			addIfUnique(V, E[i].A);
			addIfUnique(V, E[i].B);
		}

		// For each behaviour count how many inhibitions are going out and in (think: inhibition arrows)
		numOut.assign(V.size(), 0); // The indices of these two vectors match up with the indices of V
		numIn.assign(V.size(), 0);
		for(i = 0;i < E.size();i++)
		{
			for(j = 0;j < V.size();j++)
			{
				if(E[i].A == V[j]) (numOut[j])++;
				if(E[i].B == V[j]) (numIn[j])++;
			}
		}

		// Prune out behaviours that don't inhibit any other behaviours (by definition these can't be part of a cycle)
		found = true;
		while(found)
		{
			found = false;
			for(i = 0;i < V.size();i++)
			{
				if(numOut[i] == 0)
				{
					for(j = 0;j < E.size();j++)
					{
						if(E[j].B == V[i])
						{
							for(k = 0;k < V.size();k++)
							{
								if(E[j].A == V[k])
									(numOut[k])--; // We're about to prune an edge that started here!
							}
							E.erase(E.begin() + j--); // Remove the edges that end in the 'cul-de-sac' behaviour V[i]
						}
					}
					V.erase(V.begin() + i); // Remove the 'cul-de-sac' behaviour V[i]
					numOut.erase(numOut.begin() + i);
					numIn.erase(numIn.begin() + i);
					i--;
					found = true;
				}
			}
		}

		// Find the behaviour with the most behaviours that inhibit it (i.e. the most 'unpopular' one)
		maxval = -1;
		startBeh = NULL; // Should never ever come into play...
		for(i = 0;i < V.size();i++)
		{
			if(numIn[i] > maxval)
			{
				maxval = numIn[i];
				startBeh = V[i];
			}
		}

		// Take a walk around the behaviour vertices starting from the most unpopular behaviour and try to close a loop
		found = false;
		C.assign(1, startBeh);
		while(!C.empty()) // If the while-loop exits with empty C then no cycle was found, else the cycle is given (in order) in C
		{
			// Try to find an edge that would close a loop
			for(i = 0;i < C.size();i++)
			{
				for(j = 0;j < E.size();j++)
				{
					if((E[j].A == C.back()) && (E[j].B == C[i])) // Found loop that goes from C[i] -> ... -> C[end] -> C[i]
					{
						if(i > 0) C.erase(C.begin(), C.begin() + i - 1); // Erase all the elements that aren't part of the cycle
						found = true;
						break;
					}
				}
				if(found) break;
			}
			if(found) break;

			// An immediate loop not being possible, find the behaviour with the most outward inhibitions
			maxval = -1;
			maxBeh = NULL; // Should never come into play as we supposedly pruned all the 'cul-de-sac' behaviours...
			for(i = 0;i < V.size();i++)
			{
				for(j = 0;j < E.size();j++)
				{
					if((E[j].A == C.back()) && (E[j].B == V[i])) // If we have found an edge that takes us away from the current behaviour...
					{
						if(numOut[i] > maxval)
						{
							maxval = numOut[i];
							maxBeh = V[i];
						}
					}
				}
			}

			// Move to the behaviour of maximum promise
			if(maxBeh == NULL) C.clear(); // Something went wrong, so quit the loop and have an empty C to signal failure
			else C.push_back(maxBeh);
		}

		// Report the error to the user
		std::ostringstream out;
		if(C.empty()) // If no loop was found after all that effort then just report all the involved inhibitions and let the user work it out
		{
			out << "A behaviour inhibition cycle was detected in the '" << name << "' layer! The following inhibitions are involved:" << endl;
			for(i = 0;i < E.size();i++) out << "Inhibition: '" << E[i].A->name << "' ==> '" << E[i].B->name << "'" << endl;
			out << "Note that some of these inhibitions may only be implicitly defined in the code via inhibition chains (if you used the addChainInhibition() function).";
			REPORT_ERROR(MBase, out.str());
		}
		else
		{
			out << "A behaviour inhibition cycle was detected in the '" << name << "' layer! Although there may also be others, the detected cycle was:" << endl << ">>>  ";
			for(i = 0;i < C.size();i++) out << C[i]->name << " ==> ";
			out << C[0]->name << "  <<<" << endl;
			out << "Note that some of these inhibitions may only be implicitly defined in the code via inhibition chains (if you used the addChainInhibition() function).";
			REPORT_ERROR(MBase, out.str());
		}

		// Return that a cycle was found
		return RET_INHIBITION_CYCLE;
	}

	// Update the layer's behaviour list to reflect the topological ordering
	BList.assign(L.begin(), L.end());

	// Success if we got this far
	return RET_OK;
}

//
// Step functions
//

// Step function
void BehaviourLayer::step()
{
	// Update the layer
	updateAll();

	// Execute the layer
	executeAll();
}

// Update all-in-layer function
void BehaviourLayer::updateAll()
{
	// Update the values of all the sensors in the layer, and let the actuators know that a new step is now underway
	if(haveSM) SMBase->updateSensors();
	if(haveAM) AMBase->updateActuators();

	// Call the user's layer update function
	update();

	// Call the user's update functions for each of the child behaviours
	for(index_t i = 0;i < BList.size();i++)
		BList[i]->update();
}

// Execute all-in-layer function
void BehaviourLayer::executeAll()
{
	// Declare variables
	index_t i, j, num;
	level_t sum, level;

	// Retrieve how many child behaviours there are
	num = BList.size();

	// Calculate the desired activation levels of each of the behaviours
	for(i = 0;i < num;i++)
	{
		level = BList[i]->computeActivationLevel();
		BList[i]->rawA = (level_t) ((level < 0.0) ? 0.0 : ((level > 1.0 ? 1.0 : level))); // Coerce to 0->1 range... Important!
		BList[i]->trueA = BList[i]->rawA; // Initialise trueA for the loop below
	}

	// Apply the required inhibitions to get the true activation levels
	for(i = 0;i < num;i++) // BList[i] is assumed to be in a correct topological order due to the sorting in initBase()...
	{
		// Topological ordering: The implicit assumption here is that trueA is finalised for BList[i] (by behaviours that inhibit it)
		// before this j-for-loop is called for any behaviour that it inhibits. That is to say that BList[i] must come before all
		// of the behaviours it inhibits in the list, and after all of the behaviours that inhibit it.
		for(j = 0;j < BList[i]->inhibits.size();j++)
			BList[i]->inhibits[j]->trueA *= (level_t) (1.0 - BList[i]->trueA);
	}

	// Normalise to get the true activation levels
	sum = 0.0;
	for(i = 0;i < num;i++)
		sum += BList[i]->trueA;
	if(sum != 0.0)
	{
		for(i = 0;i < num;i++)
			BList[i]->trueA /= sum;
	}

	// Call the activated behaviours in the current layer
	for(i = 0;i < num;i++)
	{
		if(BList[i]->trueA > 0.0)
		{
			BList[i]->justDeactivated = true;
			BList[i]->execute();
			BList[i]->justActivated = false;
		}
		else
		{
			BList[i]->justActivated = true;
			BList[i]->inhibited();
			BList[i]->justDeactivated = false;
		}
	}
	// Call the post-execution callback
	postExecuteCallback();
}

//
// Get functions
//

// Find an actuator by name inside this layer (or report that you don't have one of that name)
// This is a worker function for the findActuator() function inside the BehaviourManager class
const ActuatorBase* BehaviourLayer::findActuator(const std::string& signalName) const
{
	// Dismiss call if requested name is null or we don't have an actuator manager
	if(signalName.empty() || !haveAM) return NULL;

	// Check all the actuators in our actuator manager to see whether they have the requested name
	for(index_t i = 0;i < AMBase->AList.size();i++)
	{
		if(AMBase->AList[i]->signalName == signalName)
		{
			return AMBase->AList[i];
		}
	}

	// No actuator of the given name was found
	return NULL;
}

// Function: BehaviourLayer::getUniqueName()
/**
* Automatically construct a unique name for the behaviour layer, for situations where none is provided
* by the derived class. The name that results of this function will be unique as long as the
* user does not explicitly use the same name somewhere else, either before or after the construction
* of this layer. If there are currently two layers in the @p MBase manager, the generated
* name will be the concatenation of @ref BehaviourLayer::DEF_LAYER_NAME "DEF_LAYER_NAME" and "2".
*
* @param MBase A non-null pointer to the parent behaviour manager.
* @return The required layer name.
**/
std::string BehaviourLayer::getUniqueName(const BehaviourManager* MBase) const
{
	// Return the required name
	std::ostringstream out;
	out << MBase->name << "/" << DEF_LAYER_NAME << MBase->LList.size();
	return out.str();
}
// EOF