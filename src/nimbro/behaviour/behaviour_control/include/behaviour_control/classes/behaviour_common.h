// Behaviour Control Framework - Common definitions
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file behaviour_common.h
* @brief Common definitions include file for the internal %Behaviour Control Framework source code.
**/

// Ensure header is only included once
#ifndef BEHAVIOUR_COMMON_H
#define BEHAVIOUR_COMMON_H

// Includes
#include <cstddef> // For std::size_t
#include <vector>  // For std::vector
#include <string>  // For std::string
#include <sstream> // For std::ostringstream
#include <typeinfo> // For std::type_info
#include <boost/static_assert.hpp> // For BOOST_STATIC_ASSERT_MSG()
#include <boost/type_traits.hpp> // For checking template types
#include <boost/utility/enable_if.hpp> // For boost::enable_if

// Define a macro to retrieve the executing function name
#if defined(_MSC_VER) || (defined(__INTEL_COMPILER) && (__INTEL_COMPILER >= 600)) || (defined(__IBMCPP__) && (__IBMCPP__ >= 500))
#define CURRENT_FUNC __FUNCTION__
#elif defined(__GNUC__) || defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901) || (defined(__ICC) && (__ICC >= 600))
#define CURRENT_FUNC __func__
#else
#define CURRENT_FUNC "<unknown>"
#endif

/**
* @name Error Macros
* A collection of macros that wrap the @ref behaviourcontrol::BehaviourManager::reportError "reportError()" function
* to make shortcuts for reporting warnings, errors and making assertions.
**/
///@{
#define REPORT_ERROR(MBasePtr, msg)          (MBasePtr)->reportError((msg),true,CURRENT_FUNC,__FILE__,__LINE__)  //!< @brief Reports a `std::string` error message @p msg to the behaviour manager pointed to by @p MBasePtr (`fatal = true`).
#define REPORT_ERROR_(MBase, msg)            (MBase).reportError((msg),true,CURRENT_FUNC,__FILE__,__LINE__)      //!< @brief Reports a `std::string` error message @p msg to the @p MBase behaviour manager (`fatal = true`).
#define REPORT_WARNING(MBasePtr, msg)        (MBasePtr)->reportError((msg),false,CURRENT_FUNC,__FILE__,__LINE__) //!< @brief Reports a `std::string` warning message @p msg to the behaviour manager pointed to by @p MBasePtr (`fatal = false`).
#define REPORT_WARNING_(MBase, msg)          (MBase).reportError((msg),false,CURRENT_FUNC,__FILE__,__LINE__)     //!< @brief Reports a `std::string` warning message @p msg to the @p MBase behaviour manager (`fatal = false`).
#define ASSERT_ERROR(cond, MBasePtr, msg)    if(!(cond)) { REPORT_ERROR((MBasePtr),msg); return; }               //!< @brief Reports a `std::string` error message @p msg to the behaviour manager pointed to by @p MBasePtr if @p cond evaluates to false (`fatal = true`).
#define ASSERT_ERROR_(cond, MBase, msg)      if(!(cond)) { REPORT_ERROR_((MBase),msg); return; }                 //!< @brief Reports a `std::string` error message @p msg to the @p MBase behaviour manager if @p cond evaluates to false (`fatal = true`).
#define ASSERT_WARNING(cond, MBasePtr, msg)  if(!(cond)) { REPORT_WARNING((MBasePtr),msg); return; }             //!< @brief Reports a `std::string` warning message @p msg to the behaviour manager pointed to by @p MBasePtr if @p cond evaluates to false (`fatal = false`).
#define ASSERT_WARNING_(cond, MBase, msg)    if(!(cond)) { REPORT_WARNING_((MBase),msg); return; }               //!< @brief Reports a `std::string` warning message @p msg to the @p MBase behaviour manager if @p cond evaluates to false (`fatal = false`).
///@}

// Behaviour control namespace
namespace behaviourcontrol
{
	//
	// Class declarations
	//
	class SensorManager;
	class SensorBase;
	template <class T> class Sensor;
	class ActuatorManager;
	class ActuatorBase;
	template <class T> class Actuator;
	class BehaviourManager;
	class BehaviourLayer;
	class Behaviour;

	//
	// Enumerations
	//
	enum FuncReturnID              //!  Used to specify return values and error codes of certain behaviour control framework functions
	{
		RET_OK = 0,                //!< Signals successful execution of a function, with no error conditions encountered
		RET_MISC_FAILURE,          //!< Signals an unknown or unspecified error
		RET_NULL_POINTER,          //!< Signals that an unexpected null pointer was encountered (usually as a function parameter)
		RET_PREVIOUS_FATAL_ERROR,  //!< Signals that an action was attempted despite a pre-existing fatal error condition
		RET_ALREADY_INITIALISED,   //!< Signals that a duplicate call of a once-only initialisation function was attempted
		RET_BIND_FAILED_NS,        //!< Signals that an attempt to bind a sensor to an actuator failed because of a null sensor signal name
		RET_BIND_FAILED_ANF,       //!< Signals that an attempt to bind a sensor to an actuator failed because no actuator of a suitable name was found
		RET_BIND_FAILED_ASI,       //!< Signals that an attempt to bind a sensor to an actuator failed because of an actuator/sensor data type incompatibility
		RET_INHIBITION_CYCLE       //!< Signals that a cycle was detected in the behaviour inhibition definitions of a layer
	};

	//
	// Type definitions
	//

	/**
	* @name Sensor Typedefs
	* More synonyms have been included than you will normally ever need. Additional commonly-used
	* typedefs (that have extra dependencies) can be added in the user's code. For example:
	* @code
	* typedef Sensor<std::int64_t>     SensorInt64   // Needs <cstdint>
	* typedef Sensor<std::uint64_t>    SensorUInt64  // Needs <cstdint>
	* typedef Sensor<Eigen::Vector2f>  SensorVec2f   // Needs <Eigen/Core>
	* typedef Sensor<Eigen::Vector3f>  SensorVec3f   // Needs <Eigen/Core>
	* typedef Sensor<Eigen::Vector2d>  SensorVec2d   // Needs <Eigen/Core>
	* typedef Sensor<Eigen::Vector3d>  SensorVec3d   // Needs <Eigen/Core>
	* @endcode
	**/
	///@{
	typedef Sensor<bool>                 SensorBool;      //!< @brief An sensor of data type `bool`
	typedef Sensor<int>                  SensorInt;       //!< @brief An sensor of data type `int`
	typedef Sensor<unsigned int>         SensorUInt;      //!< @brief An sensor of data type `unsigned int`
	typedef Sensor<long>                 SensorLong;      //!< @brief An sensor of data type `long`
	typedef Sensor<unsigned long>        SensorULong;     //!< @brief An sensor of data type `unsigned long`
	typedef Sensor<long long>            SensorLLong;     //!< @brief An sensor of data type `long long`
	typedef Sensor<unsigned long long>   SensorULLong;    //!< @brief An sensor of data type `unsigned long long`
	typedef Sensor<float>                SensorFloat;     //!< @brief An sensor of data type `float`
	typedef Sensor<double>               SensorDouble;    //!< @brief An sensor of data type `double`
	typedef Sensor<long double>          SensorLDouble;   //!< @brief An sensor of data type `long double`
	///@}

	/**
	* @name Actuator Typedefs
	* More synonyms have been included than you will normally ever need. Additional commonly-used
	* typedefs (that have extra dependencies) can be added in the user's code. For example:
	* @code
	* typedef Actuator<std::int64_t>     ActuatorInt64   // Needs <cstdint>
	* typedef Actuator<std::uint64_t>    ActuatorUInt64  // Needs <cstdint>
	* typedef Actuator<Eigen::Vector2f>  ActuatorVec2f   // Needs <Eigen/Core>
	* typedef Actuator<Eigen::Vector3f>  ActuatorVec3f   // Needs <Eigen/Core>
	* typedef Actuator<Eigen::Vector2d>  ActuatorVec2d   // Needs <Eigen/Core>
	* typedef Actuator<Eigen::Vector3d>  ActuatorVec3d   // Needs <Eigen/Core>
	* @endcode
	**/
	///@{
	typedef Actuator<bool>               ActuatorBool;    //!< @brief An actuator of data type `bool`
	typedef Actuator<int>                ActuatorInt;     //!< @brief An actuator of data type `int`
	typedef Actuator<unsigned int>       ActuatorUInt;    //!< @brief An actuator of data type `unsigned int`
	typedef Actuator<long>               ActuatorLong;    //!< @brief An actuator of data type `long`
	typedef Actuator<unsigned long>      ActuatorULong;   //!< @brief An actuator of data type `unsigned long`
	typedef Actuator<long long>          ActuatorLLong;   //!< @brief An actuator of data type `long long`
	typedef Actuator<unsigned long long> ActuatorULLong;  //!< @brief An actuator of data type `unsigned long long`
	typedef Actuator<float>              ActuatorFloat;   //!< @brief An actuator of data type `float`
	typedef Actuator<double>             ActuatorDouble;  //!< @brief An actuator of data type `double`
	typedef Actuator<long double>        ActuatorLDouble; //!< @brief An actuator of data type `long double`
	///@}

	/**
	* @name Miscellaneous Typedefs
	**/
	///@{
	typedef std::size_t index_t; //!< @brief Used to represent an array or vector index (must be an unsigned type)
	typedef float level_t;       //!< @brief Used to represent an activation level (raw activation levels should always be in the range [0,1])
	typedef int ret_t;           //!< @brief Used to represent an error code/function return value
	///@}

	//
	// Constants
	//
	const std::string nullString = "";    //!< @brief Used to avoid the need for null string literals throughout the code
	const level_t LVL_ACTIVE = 1.0;       //!< @brief Used to signal that a behaviour wishes to be fully activated
	const level_t LVL_INACTIVE = 0.0;     //!< @brief Used to signal that a behaviour wishes to be fully deactivated
	const bool AGGREGATABLE = true;       //!< @brief Used in the `Actuator` constructor to explicitly specify an aggregatable actuator
	const bool NOT_AGGREGATABLE = false;  //!< @brief Used in the `Actuator` constructor to specify a non-aggregatable actuator

	//
	// Functions
	//

	// Function: addIfUnique()
	/**
	* @brief Add an element to a `std::vector` if an equivalent element (A == B) is not already in there
	*
	* This helper function is used internally by the BehaviourLayer class while resolving the inhibition
	* structure.
	*
	* @param V The `std::vector` to append a value to.
	* @param value The value to append.
	**/
	template <class T>
	void addIfUnique(std::vector<T>& V, T& value)
	{
		// Declare variables
		bool duplicate;
		index_t i;

		// Check whether an equivalent element already exists in the vector
		duplicate = false;
		for(i = 0;i < V.size();i++)
		{
			if(V[i] == value)
			{
				duplicate = true;
				break;
			}
		}

		// Add the element to the vector if no duplicate was found
		if(!duplicate)
			V.push_back(value);
	}
}

#endif /* BEHAVIOUR_COMMON_H */
// EOF