// Utilities for timing events and actions using the ROS time
// File: ros_timing.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef ROS_TIMING_H
#define ROS_TIMING_H

// Includes
#include <cstddef>
#include <ros/time.h>
#include <ros/service_client.h>

// Nimbro utilities namespace
namespace nimbro_utils
{
	/**
	* @class RosTimeMarker
	*
	* @brief Class that facilitates the timing of durations using ROS-time
	*
	* To use a `RosTimeMarker` you simply need to instantiate one with its default constructor
	* (e.g. simply add it as a member of your class). No initialisation is necessary.
	* Then you can go straight to checking `hasElapsed(T)` for example, which until you
	* actually set the marker will always return true and allow your action to happen.
	**/
	class RosTimeMarker
	{
	public:
		// Constructors
		RosTimeMarker() : markerTime(0), iHaveMarker(false) {} //!< @brief Default constructor

		// Timing functions
		void unsetMarker() { iHaveMarker = false; } //!< @brief Forget any time marker that may have been set previously
		void setMarker()
		{
			// Record the current ROS time
			markerTime = ros::Time::now();
			iHaveMarker = true;
		} //!< @brief Set the time marker to the current ROS time (future calls to `getElapsed()` and `hasElapsed()` will be evaluated relative to this marker)
		bool haveMarker() const { return iHaveMarker; } //!< @brief Returns whether a marker is currently set
		double getElapsed() const { return (iHaveMarker ? (ros::Time::now() - markerTime).toSec() : -1.0); } //!< @brief Returns the current elapsed time since the marker was set (returns -1.0 if no marker has been set - check this if you must as `getElapsed() < 0.0`)
		bool hasElapsed(double duration) const { return !iHaveMarker || (iHaveMarker && ((ros::Time::now() - markerTime).toSec() >= duration)); } //!< @brief Returns whether a certain time duration has elapsed since the time marker was set (returns true if no marker has been set)

	private:
		// Internal variables
		ros::Time markerTime;
		bool iHaveMarker;
	};

	/**
	* @class RosTimeTracker
	*
	* @brief Class that facilitates the timing of multiple durations using ROS-time
	*
	* If the value @c N is passed to the constructor, then the valid index range for the time markers
	* (i.e. argument @c m in all the functions) is `0` to `N-1`.
	* To use a `RosTimeTracker` you simply need to instantiate one with its default constructor
	* (e.g. simply add it as a member of your class and provide `N` in the class initialiser list).
	* Then you can go straight to checking `hasElapsed(m,T)` for example, which until you
	* actually set the marker will always return true and allow your action(s) to happen.
	**/
	class RosTimeTracker
	{
	public:
		//! @brief Default constructor
		explicit RosTimeTracker(std::size_t N) : N(N)
		{
			// Make sure we have room for at least one marker
			if(N < 1) N = 1;

			// Allocate memory for the marker and flag arrays
			markerTime = new ros::Time[N];
			iHaveMarker = new bool[N];

			// Initialise the arrays explicitly
			for(std::size_t i = 0;i < N;i++)
			{
				markerTime[i].fromSec(0);
				iHaveMarker[i] = false;
			}
		}

		//! @brief Object destructor
		virtual ~RosTimeTracker() 
		{
			// Delete the allocated memory
			delete[] markerTime;
			delete[] iHaveMarker;
		}

		// Timing functions (in all these functions m is the index of the marker, valid values are 0 -> N-1)
		//! @brief Forget the `m`-th time marker if one is currently set
		void unsetMarker(std::size_t m)
		{
			// Forget the required marker
			if(m < N) iHaveMarker[m] = false;
		}
		//! @brief Set the `m`-th time marker to the current ROS time (future calls to `getElapsed(m)` and `hasElapsed(m,T)` will be evaluated relative to this marker)
		void setMarker(std::size_t m)
		{
			// Record the current ROS time in the appropriate marker
			if(m < N)
			{
				markerTime[m] = ros::Time::now();
				iHaveMarker[m] = true;
			}
		}
		//! @brief Returns whether the `m`-th marker is currently set
		bool haveMarker(std::size_t m) const { return (m < N ? iHaveMarker[m] : false); }
		//! @brief Returns the current elapsed time since the `m`-th marker was set (returns -1.0 if the `m`-th marker is not set - check this if you must as `getElapsed(m) < 0.0`)
		double getElapsed(std::size_t m) const
		{
			// Return the elapsed time since marker m was set
			if(m < N) return (iHaveMarker[m] ? (ros::Time::now() - markerTime[m]).toSec() : -1.0);
			else return -1.0;
		}
		//! @brief Returns whether a certain time duration has elapsed since the `m`-th time marker was set (returns true if no `m`-th marker has been set)
		bool hasElapsed(std::size_t m, double duration) const
		{
			// Return whether the given duration has elapsed since marker m was set
			if(m < N) return !iHaveMarker[m] || (iHaveMarker[m] && ((ros::Time::now() - markerTime[m]).toSec() >= duration));
			else return false;
		}

	private:
		// Internal variables
		ros::Time* markerTime;
		bool* iHaveMarker;
		std::size_t N;
	};

	/**
	* @class RosServiceCaller
	*
	* @brief Provides basic functionality for controlling calls to ROS services
	*
	* To use a `RosServiceCaller` in your class, simply add an instance of it as a class member and
	* provide the two delay arguments to the `RosServiceCaller` constructor in the class initialiser list.
	* From then on you can simply directly invoke `callService()` whenver you need to.
	* So in the end you'll probably have something like this:
	* @code
	* RosServiceCaller<srv_node::MySrv> m_rsc_mySrv(2.0, 0.3);
	* ...
	* if(I_Want_To_Call_My_Service)
	* {
	* 	srv_node::MySrv data;
	* 	data.request.XXX = ...;
	* 	data.request.YYY = ...;
	* 	if(m_rsc_mySrv.callService(data))
	* 	{
	* 		// Process data.response!
	* 	}
	* }
	* ...
	* OR
	* ...
	* if(I_Want_To_Call_My_Service)
	* {
	* 	m_rsc_mySrv.data.request.XXX = ...;
	* 	m_rsc_mySrv.data.request.YYY = ...;
	* 	if(m_rsc_mySrv.callService())
	* 	{
	* 		// Process m_rsc_mySrv.data.response!
	* 	}
	* }
	* @endcode
	**/
	template <class T>
	class RosServiceCaller
	{
	public:
		/**
		* @brief Default constructor
		*
		* @param reissueDelay The minimum time to wait after a successful service call before allowing attempts to call the service again.
		* @param failRetryDelay The minimum time to wait after an unsuccessful service call before allowing attempts to call the service again.
		**/
		RosServiceCaller(double reissueDelay, double failRetryDelay)
			: failRetryDelay(failRetryDelay)
			, reissueDelay(reissueDelay)
		{}

		// Service client set function
		void setServiceClient(const ros::ServiceClient& SC) { m_srv = SC; } //!< @brief Set function for the ROS `ServiceClient` used by this `RosServiceCaller`

		// Service call functions
		//! @brief Call the required service with the request data stored in the @c data class member
		bool callService() { return callService(data); }
		//! @brief Call the required service with the request data stored in @p srv_data
		bool callService(T& srv_data)
		{
			// Declare variables
			bool ret = false;

			// Decide whether to call the service or not (don't want to spam the service by calling it every time!)
			if(lastGoodCall.hasElapsed(reissueDelay) && lastBadCall.hasElapsed(failRetryDelay))
			{
				ret = m_srv.call<T>(srv_data); // If setServiceClient() hasn't been called (i.e. m_srv is invalid) then this returns false
				if(ret) lastGoodCall.setMarker();
				else lastBadCall.setMarker();
			}

			// Return whether the service call was carried out AND was successful
			return ret;
		}

	public:
		// Variables
		T data; //!< @brief Request variable (can choose to use this instead of providing your own in the calling scope of `callService()`)

	private:
		// Internal variables
		const double failRetryDelay;
		const double reissueDelay;
		ros::ServiceClient m_srv;
		RosTimeMarker lastGoodCall;
		RosTimeMarker lastBadCall;
	};
}

#endif /* ROS_TIMING_H */
// EOF