// Utilities for publishing markers
// File: marker_manager.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef MARKER_MANAGER_H
#define MARKER_MANAGER_H

// Includes
#include <string>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/this_node.h> // Note: ros::this_node::getName() returns the node name with a '/' at the front!
#include <visualization_msgs/MarkerArray.h>

// Visualisation utilities namespace
namespace vis_utils
{
	// Classes
	class MarkerManager;
	class GenMarker;
	class SphereMarker;
	class CubeMarker;
	class BoxMarker;
	class ArrowMarker;

	/**
	* @class GenMarker
	*
	* @brief The base class for all markers that can be used with a MarkerManager.
	*
	* For a general marker you can directly instantiate and use this class, and use the provided
	* `set*()` functions to customise the marker. The `update()` function then needs
	* to be called to notify the parent MarkerManager that the given marker should be added to
	* the marker array for this cycle. For more complicated markers you can also manually modify
	* the embedded `visualization_msgs::Marker` object (the member called '`marker`') and the `update()`
	* function will still work the same. Note however that it is *not* recommended that you manually
	* modify the @c header, @c ns and/or @c id fields of @c marker. Everything else is fair game as
	* long as you know what you're doing. Refer to the <a href="http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html">visualization_msgs::Marker message class</a>
	* for more information.
	*
	* If there is a particular class or type of marker that you need multiple
	* instances of (even if they're slightly customised between the instances), then it is recommended
	* you subclass GenMarker and provide a constructor that initialises the special marker object,
	* and overload the `update()` function to be specific to your case. Refer to the `SphereMarker`
	* class for an example of how to approach this. A snapshot of the SphereMarker class is given
	* here to make things easy, but it is recommended you refer to the `markers.h` source file.
	* @code
	* class SphereMarker : public GenMarker
	* {
	* public:
	* 	// Constructor
	* 	explicit SphereMarker(MarkerManager* MM, const std::string& frameID = "", const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE) : GenMarker(MM, frameID, markerNamespace)
	* 	{
	* 		// Set marker properties
	* 		setType(visualization_msgs::Marker::SPHERE);
	* 		setScale(0.025);
	* 	}
	* 	
	* 	// Update function
	* 	void update(double x, double y, double z)
	* 	{
	* 		// Update the sphere marker
	* 		if(MM->willPublish())
	* 		{
	* 			setPosition(x, y, z); // <-- In update() overloads, set any additional marker properties here
	* 			MM->add(this);
	* 		}
	* 	}
	* };
	* @endcode
	* Refer to the documentation of the MarkerManager class for an example of how to use the GenMarker
	* class.
	**/
	class GenMarker
	{
	public:
		// Constants
		static const std::string DEFAULT_MARKER_NAMESPACE; //!< @brief Default value of the namespace field for markers (expands to the node name).

		/**
		* @brief Default constructor.
		*
		* @param MM A pointer to the MarkerManager that should own this marker. If this parameter is null
		* then a segmentation fault will be of likely consequence.
		* @param frameID The name of the coordinate frame in which the marker should be published (e.g. "/odom").
		* If it is desired for the marker to be continually retransformed into this coordinate frame for the
		* duration of its lifetime, then use the `setFrameLocked()` function with argument `true`.
		* @param markerNamespace The namespace to use for the marker (`ns` field). A leading '~' is automatically
		* replaced by the node name (e.g. '~' turns into '/node_name', and '~foo' turns into '/node_name/foo').
		**/
		explicit GenMarker(MarkerManager* MM, const std::string& frameID = "", const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE);

		/**
		* @name Set Functions for Marker Properties
		**/
		///@{
		//! @brief Sets the name of the coordinate frame in which the marker should be published (e.g. "/odom"). Refer also to `setFrameLocked()`.
		void setFrameID(const std::string& frameID)
		{
			marker.header.frame_id = frameID;
		}
		//! @brief Sets the marker type. Valid values are defined in the `visualization_msgs::Marker` class (e.g. `visualization_msgs::Marker::ARROW`).
		void setType(int type)
		{
			marker.type = type;
		}
		//! @brief Sets the marker's desired position relative to the coordinate frame that is specified.
		void setPosition(double x, double y, double z)
		{
			marker.pose.position.x = x;
			marker.pose.position.y = y;
			marker.pose.position.z = z;
		}
		//! @brief Sets the marker's desired orientation relative to the coordinate frame that is specified (expressed as a quarternion).
		void setOrientation(double w, double x, double y, double z)
		{
			marker.pose.orientation.w = w;
			marker.pose.orientation.x = x;
			marker.pose.orientation.y = y;
			marker.pose.orientation.z = z;
		}
		//! @brief Sets the scale (i.e. size) of the marker uniformly in all axes
		void setScale(double scale)
		{
			marker.scale.x = scale;
			marker.scale.y = scale;
			marker.scale.z = scale;
		}
		//! @brief Sets the scale (i.e. size) of the marker individually along each axis
		void setScale(double scaleX, double scaleY, double scaleZ)
		{
			marker.scale.x = scaleX;
			marker.scale.y = scaleY;
			marker.scale.z = scaleZ;
		}
		//! @brief Sets the color of the marker in RGB using floating point numbers on the unit interval
		void setColor(float R, float G, float B, float A = 1.0)
		{
			marker.color.r = R;
			marker.color.g = G;
			marker.color.b = B;
			marker.color.a = A;
		}
		//! @brief Sets the lifetime of the marker, that is, how long the marker is displayed before it automatically disappears.
		void setLifetime(double lifetime)
		{
			marker.lifetime = ros::Duration(lifetime);
		}
		//! @brief Specifies whether the marker should continually be retransformed into the reference coordinate frame for the duration of its lifetime (continually retransform &rarr; set `locked = true`).
		void setFrameLocked(bool locked)
		{
			marker.frame_locked = locked;
		}
		///@}

		// Update function
		void update(); //!< @brief Signals to the owning MarkerManager that this marker wishes to be published in this step. This function adds the marker to the marker array embedded inside the MarkerManager.

		// Pointer to the parent MarkerManager
		MarkerManager* const MM; //!< @brief A pointer to the owning MarkerManager object of the marker.

		// Visualisation marker object
		visualization_msgs::Marker marker; //!< @brief The internal `visualization_msgs::Marker` object that is used for publishing and storing of the marker properties.
	};
	
	/**
	* @class MarkerManager
	*
	* @brief Class that facilitates the display of markers in the RViz widget.
	*
	* In general only one MarkerManager instance should be used per @b node, and all parts of the
	* node should publish markers using the functions of this MarkerManager.
	*
	* The MarkerManager class is used as follows:
	* @code
	* //
	* // Definition of a MarkerManager subclass...
	* //
	* class MarkerMan : public MarkerManager
	* {
	* public:
	* 	// Constructor
	* 	MarkerMan() : MarkerManager("~vis_marker_array", 10) // Publish to "/node_name/vis_marker_array" topic in every 10th call to publish()...
	* 		, Ball(this, "/odom", 0.2)
	* 		, Obstacle(this, "/odom")
	* 		, ZMP(this, "/ego_floor", 0.02)
	* 	{
	* 		// Initialise the ball marker
	* 		Ball.setColor(0.882, 0.196, 0.000); // Note: This overrides the default color specified in the SphereMarker constructor for example
	* 		
	* 		// Initialise the obstacle marker
	* 		Obstacle.setType(visualization_msgs::Marker::CYLINDER);
	* 		Obstacle.setScale(0.35, 0.35, 1.00);
	* 		Obstacle.setColor(0.10, 0.10, 0.10);
	* 		
	* 		// Initialise the ZMP marker
	* 		ZMP.setColor(0.0, 0.0, 1.0);
	* 	}
	*
	* 	// Markers
	* 	SphereMarker Ball;
	* 	GenMarker Obstacle;
	* 	CubeMarker ZMP;
	* };
	*
	* //
	* // In the main function/class...
	* //
	* MarkerMan Markers;
	*
	* //
	* // In the step function/main loop...
	* //
	* Markers.clear(); // <-- This sets the ROS timestamp of the markers for the next call to publish() (a ros::Time can be passed as an argument if the internal ros::Time::now() doesn't suffice)
	* ...
	* myvec = where_is_ball();
	* Markers.Ball.update(myvec.x, myvec.y, myvec.z);
	* ...
	* myvec2 = where_is_obstacle();
	* Markers.Obstacle.setPosition(myvec2.x, myvec2.y, 0.50);
	* Markers.Obstacle.update();
	* ...
	* if(Markers.willPublish())
	* {
	* 	zmpvec = expensive_calc_only_for_marker();
	* 	Markers.ZMP.update(zmpvec.x, zmpvec.y, zmpvec.z);
	* }
	* ...
	* Markers.publish();
	* @endcode
	* There should be no sleeping/ROS spinning between `Markers.clear()` and `Markers.publish()` or the
	* marker timestamps get less accurate.
	**/
	class MarkerManager
	{
	public:
		// Constants
		static const std::string DEFAULT_TOPICNAME; //!< @brief Default topic name to use for the published markers array

		/**
		* @brief Default constructor
		*
		* @param topicName A `std::string` specifying the topic to publish the markers to (Default:
		* `DEFAULT_TOPICNAME`). If the first character is @c ~ then it is explicitly replaced by the
		* node name (e.g. @c ~foo becomes @c /node_name/foo) so as to ensure conformance.
		* @param publishInterval Specifies the rate at which to publish the markers (Default: `1`).
		* If this parameter is @c n then the markers are only published every `n`-th time.
		* @param enabled This controls the intial enabled state of the MarkerManager (Default: `true)`.
		* The enabled state can later be changed using the enable() and disable() functions, and
		* controls whether the MarkerManager actually publishes anything.
		**/
		explicit MarkerManager(const std::string& topicName = DEFAULT_TOPICNAME, int publishInterval = 1, bool enabled = true)
			: publishInterval(publishInterval)
			, publishCount(1)
			, m_stamp()
			, enabled(enabled)
			, IDCount(0)
		{
			// Save the topic name
			itopicName = topicName;
			
			// Verify that the topic name is valid
			std::string errStr;
			if(!ros::names::validate(itopicName, errStr))
			{
				ROS_WARN_STREAM("Invalid topic name '" << itopicName << "', using '" << DEFAULT_TOPICNAME << "' instead!");
				ROS_WARN_STREAM("Details: " << errStr);
				itopicName = DEFAULT_TOPICNAME;
			}

			// Replace the ~ character explicitly to ensure naming convention is consistent across utilities
			if(itopicName.empty() || itopicName == "~" || itopicName == "/") itopicName = DEFAULT_TOPICNAME;
			if(itopicName.at(0) == '~') itopicName.replace(0, 1, ros::this_node::getName() + "/");

			// Advertise on the plot topic
			ros::NodeHandle nh("~");
			m_pub_markers = nh.advertise<visualization_msgs::MarkerArray>(itopicName, 1);

			// Error checking on the downsample rate
			if(publishInterval < 1)
			{
				ROS_WARN_STREAM("Publish interval of MarkerManager class cannot be less than 1 ('" << publishInterval << "'), using 1 by default!");
				publishInterval = 1;
			}
		}

		//! @brief Reset function. Invokes `clear()` and forces the next call to `publish()` to actually publish the required markers.
		void reset()
		{
			// Reset internal variables
			clear();
			forcePublish();
		}

		//! @brief Function that clears the internal marker array and sets the current timestamp to the current ROS time. Call one clear() overload at the start of every main loop cycle, ensuring that there is no sleeping between this and the later call to `publish()`.
		void clear()
		{
			// Clear the markers list and set the timestamp
			clear(ros::Time::now());
		}

		//! @brief Function that clears the internal marker array and sets the current timestamp to @p stamp. Call one clear() overload at the start of every main loop cycle, ensuring that there is no sleeping between this and the later call to `publish()`.
		void clear(ros::Time stamp)
		{
			// Clear the markers list and set the timestamp
			m_stamp = stamp;
			m_markers.markers.clear();
		}

		//! @brief Returns whether the MarkerManager will actually publish the markers in this cycle
		bool willPublish() const
		{
			// Return whether the next call to publish() will actually publish to the ROS topic or not
			// This function obviously can't predict whether reset() is called before the next publish though, which would change whether the publish happens or not
			return enabled && (publishCount <= 1);
		}
		//! @brief Updates the marker timestamp with the stamp from the last call to `clear()`, and adds the marker to the internal marker array
		void add(GenMarker *MB)
		{
			// Stamp the marker with the time of the last call to clear()
			MB->marker.header.stamp = m_stamp;
			
			// Add the required marker to the marker array (ready for publishing)
			m_markers.markers.push_back(MB->marker);
		}
		//! @brief Forces the next call to `publish()` to actually publish the collected markers. Note however that if `willPublish()` was false before the call to `forcePublish()` then only markers added after this call will be published.
		void forcePublish()
		{
			// Reset the publish count artificially to force a publish in this step
			publishCount = 1;
		}
		//! @brief Publish the markers stored in the internal marker array to the required ROS topic. The MarkerManager must be `enabled` and the `publishInterval` must have just expired for any publishing to actually occur.
		void publish()
		{
			// Publish the required markers at the desired downsampled rate
			if(enabled)
			{
				if(publishCount <= 1)
				{
					m_pub_markers.publish(m_markers);
					publishCount = publishInterval;
				}
				else publishCount--;
			}
		}

		// Get functions
		const std::string& getTopicName() const { return itopicName; } //!< @brief Returns the topic name in use by the MarkerManager for publishing of the visualization markers
		int getPublishInterval() const { return publishInterval; } //!< @brief Returns the publishing interval in use by the MarkerManager. If this is 'n' then the markers are only published every `n`-th time.
		int getUniqueID() { return ++IDCount; } //!< @brief Returns a unique ID for the purpose of uniquely identifying markers belonging to this MarkerManager (for the `id` field of the `visualization_msgs::Marker` class)
		bool getEnabled() const { return enabled; } //!< @brief Returns whether the MarkerManager is currently enabled
		int getNumMarkers() const { return m_markers.markers.size(); } //!< @brief Returns how many markers are currently located in the markers array

		// Properties
		void enable()  { enabled =  true; forcePublish(); } //!< @brief Enables the MarkerManager (see `publish()`)
		void disable() { enabled = false; forcePublish(); } //!< @brief Disables the MarkerManager (see `publish()`)

	private:
		// Internal variables
		ros::Publisher m_pub_markers;
		visualization_msgs::MarkerArray m_markers;
		int publishInterval, publishCount;
		std::string itopicName;
		ros::Time m_stamp;
		bool enabled;
		int IDCount;

		// Friend classes
		friend class GenMarker;
	};

	/**
	* @class SphereMarker
	*
	* @brief Encapsulates a spherical marker object. By default sets up a spherical marker of diameter 0.025.
	*
	* This class demonstrates for a simple case how you can subclass the GenMarker class to create a
	* specialised marker class. Any special marker properties can be set in the constructor, and
	* the update function can be overloaded to accept whatever values need to change in every cycle.
	* In this case it's just the position of the sphere, but it could be something more abstract,
	* such as a vector start and end position. The overloaded update function would then calculate
	* the required arrow marker positions and orientations and set them (before calling `MM->add()`).
	**/
	class SphereMarker : public GenMarker
	{
	public:
		/**
		* @brief Default constructor.
		*
		* @param MM A pointer to the MarkerManager that should own this marker. If this parameter is null
		* then a segmentation fault will be of likely consequence.
		* @param frameID The name of the coordinate frame in which the marker should be published (e.g. "/odom").
		* If it is desired for the marker to be continually retransformed into this coordinate frame for the
		* duration of its lifetime, then use the `setFrameLocked()` function with argument `true`.
		* @param diameter The required diameter of the spherical marker (Default: 0.025).
		* @param markerNamespace The namespace to use for the marker (`ns` field). A leading '~' is automatically
		* replaced by the node name (e.g. '~' turns into '/node_name', and '~foo' turns into '/node_name/foo').
		**/
		explicit SphereMarker(MarkerManager* MM, const std::string& frameID = "", double diameter = 0.025, const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE) : GenMarker(MM, frameID, markerNamespace)
		{
			// Set marker properties
			setType(visualization_msgs::Marker::SPHERE);
			setScale(diameter);
		}

		//! @brief Update function overload: Sets the position of the marker
		void update(double x, double y, double z)
		{
			// Update the marker
			if(MM->willPublish())
			{
				setPosition(x, y, z); // <-- In update() overloads, set any additional marker properties here
				MM->add(this);
			}
		}
	};

	/**
	* @class CubeMarker
	*
	* @brief Encapsulates a cube marker object. By default sets up a cube marker of edge length 0.025.
	**/
	class CubeMarker : public GenMarker
	{
	public:
		/**
		* @brief Default constructor.
		*
		* @param MM A pointer to the MarkerManager that should own this marker. If this parameter is null
		* then a segmentation fault will be of likely consequence.
		* @param frameID The name of the coordinate frame in which the marker should be published (e.g. "/odom").
		* If it is desired for the marker to be continually retransformed into this coordinate frame for the
		* duration of its lifetime, then use the `setFrameLocked()` function with argument `true`.
		* @param size The required edge length of the cube marker (Default: 0.025).
		* @param markerNamespace The namespace to use for the marker (`ns` field). A leading '~' is automatically
		* replaced by the node name (e.g. '~' turns into '/node_name', and '~foo' turns into '/node_name/foo').
		**/
		explicit CubeMarker(MarkerManager* MM, const std::string& frameID = "", double size = 0.025, const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE) : GenMarker(MM, frameID, markerNamespace)
		{
			// Set marker properties
			setType(visualization_msgs::Marker::CUBE);
			setScale(size);
		}

		//! @brief Update function overload: Sets the position of the marker
		void update(double x, double y, double z)
		{
			// Update the marker
			if(MM->willPublish())
			{
				setPosition(x, y, z);
				MM->add(this);
			}
		}
	};

	/**
	* @class BoxMarker
	*
	* @brief Encapsulates a rectangular box marker object. By default sets up a box marker where each dimension is 0.20.
	**/
	class BoxMarker : public GenMarker
	{
	public:
		/**
		* @brief Default constructor.
		*
		* Avoid mixing use of the two `update()` overloads or the orientation of the BoxMarker may not end up as expected.
		*
		* @param MM A pointer to the MarkerManager that should own this marker. If this parameter is null
		* then a segmentation fault will be of likely consequence.
		* @param frameID The name of the coordinate frame in which the marker should be published (e.g. "/odom").
		* If it is desired for the marker to be continually retransformed into this coordinate frame for the
		* duration of its lifetime, then use the `setFrameLocked()` function with argument `true`.
		* @param sizeX The required size of the box in the x direction (Default: 0.20).
		* @param sizeY The required size of the box in the y direction (Default: 0.20).
		* @param sizeZ The required size of the box in the z direction (Default: 0.20).
		* @param markerNamespace The namespace to use for the marker (`ns` field). A leading '~' is automatically
		* replaced by the node name (e.g. '~' turns into '/node_name', and '~foo' turns into '/node_name/foo').
		**/
		explicit BoxMarker(MarkerManager* MM, const std::string& frameID = "", double sizeX = 0.20, double sizeY = 0.20, double sizeZ = 0.20, const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE) : GenMarker(MM, frameID, markerNamespace)
		{
			// Set marker properties
			setType(visualization_msgs::Marker::CUBE);
			setScale(sizeX, sizeY, sizeZ);
		}

		//! @brief Update function overload: Sets the position of the marker only (the orientation should have been set at some point using `setOrientation`, e.g. during construction)
		void update(double x, double y, double z)
		{
			// Update the marker
			if(MM->willPublish())
			{
				setPosition(x, y, z);
				MM->add(this);
			}
		}
		//! @brief Update function overload: Sets the position and orientation of the marker
		void update(double x, double y, double z, double rw, double rx, double ry, double rz)
		{
			// Update the marker
			if(MM->willPublish())
			{
				setPosition(x, y, z);
				setOrientation(rw, rx, ry, rz);
				MM->add(this);
			}
		}
	};

	/**
	* @class ArrowMarker
	*
	* @brief Encapsulates an arrow marker object. By default sets up an arrow marker with shaft diameter 0.025 and head diameter 0.050.
	**/
	class ArrowMarker : public GenMarker
	{
	public:
		/**
		* @brief Default constructor.
		*
		* @param MM A pointer to the MarkerManager that should own this marker. If this parameter is null
		* then a segmentation fault will be of likely consequence.
		* @param frameID The name of the coordinate frame in which the marker should be published (e.g. "/odom").
		* If it is desired for the marker to be continually retransformed into this coordinate frame for the
		* duration of its lifetime, then use the `setFrameLocked()` function with argument `true`.
		* @param shaft_diam The required shaft diameter of the arrow (Default: 0.025).
		* @param head_diam The required arrow head diameter (Default: 0.050).
		* @param head_length The required arrow head length, where a value 0.0 means 'automatically select' (Default: 0.0).
		* @param markerNamespace The namespace to use for the marker (`ns` field). A leading '~' is automatically
		* replaced by the node name (e.g. '~' turns into '/node_name', and '~foo' turns into '/node_name/foo').
		**/
		explicit ArrowMarker(MarkerManager* MM, const std::string& frameID = "", double shaft_diam = 0.025, double head_diam = 0.050, double head_length = 0.0, const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE) : GenMarker(MM, frameID, markerNamespace)
		{
			// Set marker properties
			setType(visualization_msgs::Marker::ARROW);
			setScale(shaft_diam, head_diam, head_length);

			// Initialise the points array from/to pair
			marker.points.resize(2);
			marker.points[0].x = 0.0;
			marker.points[0].y = 0.0;
			marker.points[0].z = 0.0;
			marker.points[1].x = 1.0;
			marker.points[1].y = 0.0;
			marker.points[1].z = 0.0;
		}

		//! @brief Update function overload: Sets the coordinates of the arrow, assuming that the base is located at the origin
		void update(double x, double y, double z)
		{
			// Update the marker
			if(MM->willPublish())
			{
				marker.points[0].x = 0.0;
				marker.points[0].y = 0.0;
				marker.points[0].z = 0.0;
				marker.points[1].x = x;
				marker.points[1].y = y;
				marker.points[1].z = z;
				MM->add(this);
			}
		}
		//! @brief Update function overload: Sets the from (tail) and to (tip) coordinates of the arrow
		void update(double fromx, double fromy, double fromz, double tox, double toy, double toz)
		{
			// Update the marker
			if(MM->willPublish())
			{
				marker.points[0].x = fromx;
				marker.points[0].y = fromy;
				marker.points[0].z = fromz;
				marker.points[1].x = tox;
				marker.points[1].y = toy;
				marker.points[1].z = toz;
				MM->add(this);
			}
		}
	};
}

#endif /* MARKER_MANAGER_H */
// EOF