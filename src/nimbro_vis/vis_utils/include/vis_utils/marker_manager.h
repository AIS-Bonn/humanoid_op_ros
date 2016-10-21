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
		* @param dynamic Set to true if constructing the marker as a dynamic marker (Default: false).
		**/
		explicit GenMarker(MarkerManager* MM, const std::string& frameID = "", const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE, bool dynamic = false);

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
		//! @brief Sets the marker action (add/modify or delete).
		void setAction(int action)
		{
			marker.action = action;
		}
		//! @brief Sets the marker action to add/modify or delete depending on whether the marker should be visible or not.
		void setVisible(bool visible)
		{
			marker.action = (visible ? (int32_t)visualization_msgs::Marker::ADD : (int32_t)visualization_msgs::Marker::DELETE);
		}
		//! @brief Sets the marker action to add/modify.
		void show()
		{
			marker.action = visualization_msgs::Marker::ADD;
		}
		//! @brief Sets the marker action to delete.
		void hide()
		{
			marker.action = visualization_msgs::Marker::DELETE;
		}
		//! @brief Gets whether the marker action is to add/modify.
		bool shown() const
		{
			return (marker.action == visualization_msgs::Marker::ADD);
		}
		//! @brief Gets whether the marker action is to delete.
		bool hidden() const
		{
			return (marker.action != visualization_msgs::Marker::ADD);
		}
		//! @brief Sets the marker's desired position relative to the coordinate frame that is specified (this overload is 2D only).
		void setPosition(double x, double y)
		{
			marker.pose.position.x = x;
			marker.pose.position.y = y;
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
		//! @brief Sets the scale (i.e. size) of the marker in the X axis
		void setScaleX(double scaleX)
		{
			marker.scale.x = scaleX;
		}
		//! @brief Sets the scale (i.e. size) of the marker in the Y axis
		void setScaleY(double scaleY)
		{
			marker.scale.y = scaleY;
		}
		//! @brief Sets the scale (i.e. size) of the marker in the Z axis
		void setScaleZ(double scaleZ)
		{
			marker.scale.z = scaleZ;
		}
		//! @brief Sets the text of the marker
		void setText(const std::string& text)
		{
			marker.text = text;
		}
		//! @brief Set the size of the points member
		void setNumPoints(size_t num)
		{
			geometry_msgs::Point defPoint;
			defPoint.x = defPoint.y = defPoint.z = 0.0;
			marker.points.resize(num, defPoint);
		}
		//! @brief Sets a value in the points member of the marker
		void setPoint(size_t i, double x, double y)
		{
			if(i < marker.points.size())
			{
				geometry_msgs::Point& p = marker.points[i];
				p.x = x;
				p.y = y;
			}
		}
		//! @brief Sets a value in the points member of the marker
		void setPoint(size_t i, double x, double y, double z)
		{
			if(i < marker.points.size())
			{
				geometry_msgs::Point& p = marker.points[i];
				p.x = x;
				p.y = y;
				p.z = z;
			}
		}
		//! @brief Sets the color of the marker in RGB using floating point numbers on the unit interval
		void setColor(float r, float g, float b, float a = 1.0)
		{
			marker.color.r = r;
			marker.color.g = g;
			marker.color.b = b;
			marker.color.a = a;
		}
		//! @brief Set the size of the colors member
		void setNumPtColors(size_t num)
		{
			std_msgs::ColorRGBA defCol;
			defCol.r = defCol.a = 1.0;
			defCol.g = defCol.b = 0.0;
			marker.colors.resize(num, defCol);
		}
		//! @brief Set a color of the marker in RGB using floating point numbers on the unit interval
		void setPtColor(size_t i, float r, float g, float b, float a = 1.0)
		{
			if(i < marker.colors.size())
			{
				std_msgs::ColorRGBA& c = marker.colors[i];
				c.r = r;
				c.g = g;
				c.b = b;
				c.a = a;
			}
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
		void updateAdd(); //!< @brief Signals to the owning MarkerManager that this marker wishes to be published in this step. This function adds the marker to the marker array embedded inside the MarkerManager.

		// Pointer to the parent MarkerManager
		MarkerManager* const MM; //!< @brief A pointer to the owning MarkerManager object of the marker.

		// Visualisation marker object
		visualization_msgs::Marker marker; //!< @brief The internal `visualization_msgs::Marker` object that is used for publishing and storing of the marker properties.

		// Flag whether the marker is dynamic
		const bool dynamic;
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
			// Construct the class
			ros::NodeHandle nh("~");
			construct(nh, topicName);
		}

		/**
		* @brief Constructor with explicit node handle
		*
		* @param nh The `ros::NodeHandle` to use for the marker manager.
		* @param topicName A `std::string` specifying the topic to publish the markers to (Default:
		* `DEFAULT_TOPICNAME`). If the first character is @c ~ then it is explicitly replaced by the
		* node name (e.g. @c ~foo becomes @c /node_name/foo) so as to ensure conformance.
		* @param publishInterval Specifies the rate at which to publish the markers (Default: `1`).
		* If this parameter is @c n then the markers are only published every `n`-th time.
		* @param enabled This controls the intial enabled state of the MarkerManager (Default: `true)`.
		* The enabled state can later be changed using the enable() and disable() functions, and
		* controls whether the MarkerManager actually publishes anything.
		**/
		explicit MarkerManager(ros::NodeHandle& nh, const std::string& topicName = DEFAULT_TOPICNAME, int publishInterval = 1, bool enabled = true)
			: publishInterval(publishInterval)
			, publishCount(1)
			, m_stamp()
			, enabled(enabled)
			, IDCount(0)
		{
			// Construct the class
			construct(nh, topicName);
		}

	private:
		// Constructor function
		void construct(ros::NodeHandle& nh, const std::string& topicName)
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
			m_pub_markers = nh.advertise<visualization_msgs::MarkerArray>(itopicName, 1);

			// Error checking on the downsample rate
			if(publishInterval < 1)
			{
				ROS_WARN_STREAM("Publish interval of MarkerManager class cannot be less than 1 ('" << publishInterval << "'), using 1 by default!");
				publishInterval = 1;
			}
		}

	public:
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

		//! @brief Function that allows you to set the current timestamp to @p stamp.
		void setStamp(ros::Time stamp)
		{
			// Set the timestamp
			m_stamp = stamp;
		}

		//! @brief Function that returns the current timestamp
		ros::Time getStamp() const
		{
			// Return the current timestamp
			return m_stamp;
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

		//! @brief Create an entry for a new dynamic marker
		void newDynamicMarker(int index) { m_dynamicMap[index] = getUniqueID(); }
		//! @brief Delete the entry for a dynamic marker
		void deleteDynamicMarker(int index) { m_dynamicMap.erase(index); }
		//! @brief Reset all dynamic markers
		void deleteAllDynamicMarkers() { m_dynamicMap.clear(); }
		/**
		* @brief Update a dynamic marker by index
		* 
		* The normal workflow for using dynamic markers in the main loop is something like:
		* @code
		* MM.clear();
		* ...
		* // <-- Plot normal static markers as usual
		* ...
		* vis_utils::TextMarker text(&MM, "trunk_link", 0.1, "mynamespace", true); // The last 'true' here is important, it makes the marker dynamic!
		* for(int i = 0; i < DYNAMIC; i++)
		* {
		* 	// <-- Configure the dynamic text marker as required
		* 	MM.updateDynamicMarker(i, text);
		* }
		* ...
		* vis_utils::GenMarker gen(&MM, "trunk_link", "mynamespace", true); // The last 'true' here is important, it makes the marker dynamic!
		* gen.setType(visualization_msgs::Marker::LINE_LIST);
		* gen.setScaleX(0.05);
		* // <-- Continue configuring the generic marker
		* MM.updateDynamicMarker(20, gen); // The dynamic marker will be overwritten by any future dynamic markers updated with the index 20
		* ...
		* MM.publish();
		* @endcode
		**/
		void updateDynamicMarker(int index, vis_utils::GenMarker& marker);

		// Get functions
		const std::string& getTopicName() const { return itopicName; } //!< @brief Returns the topic name in use by the MarkerManager for publishing of the visualization markers
		int getPublishInterval() const { return publishInterval; } //!< @brief Returns the publishing interval in use by the MarkerManager. If this is 'n' then the markers are only published every `n`-th time.
		int getUniqueID() { return ++IDCount; } //!< @brief Returns a unique ID for the purpose of uniquely identifying markers belonging to this MarkerManager (for the `id` field of the `visualization_msgs::Marker` class)
		bool getEnabled() const { return enabled; } //!< @brief Returns whether the MarkerManager is currently enabled
		int getNumMarkers() const { return m_markers.markers.size(); } //!< @brief Returns how many markers are currently located in the markers array

		// Properties
		void enable()  { enabled =  true; forcePublish(); } //!< @brief Enables the MarkerManager (see `publish()`)
		void disable() { enabled = false; forcePublish(); } //!< @brief Disables the MarkerManager (see `publish()`)
		void setEnabled(bool enabled) { this->enabled = enabled; forcePublish(); } //!< @brief Sets the enabled state of the MarkerManager (see `publish()`)

	protected:
		// ROS publisher
		ros::Publisher m_pub_markers;

	private:
		// Internal typedefs
		typedef std::map<int, int> IndexMap; // Maps indices to rviz IDs

		// Internal variables
		visualization_msgs::MarkerArray m_markers;
		int publishInterval, publishCount;
		std::string itopicName;
		IndexMap m_dynamicMap;
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
		* @param markerNamespace The namespace to use for the marker (`ns` field). A leading '~' is automatically
		* replaced by the node name (e.g. '~' turns into '/node_name', and '~foo' turns into '/node_name/foo').
		* @param dynamic Set to true if constructing the marker as a dynamic marker (Default: false).
		**/
		explicit SphereMarker(MarkerManager* MM, const std::string& frameID = "", const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE, bool dynamic = false) : GenMarker(MM, frameID, markerNamespace, dynamic)
		{
			// Set marker properties
			setType(visualization_msgs::Marker::SPHERE);
			setScale(0.025);
		}

		/**
		* @brief Specialised constructor.
		*
		* @param MM A pointer to the MarkerManager that should own this marker. If this parameter is null
		* then a segmentation fault will be of likely consequence.
		* @param frameID The name of the coordinate frame in which the marker should be published (e.g. "/odom").
		* If it is desired for the marker to be continually retransformed into this coordinate frame for the
		* duration of its lifetime, then use the `setFrameLocked()` function with argument `true`.
		* @param diameter The required diameter of the spherical marker (Default: 0.025).
		* @param markerNamespace The namespace to use for the marker (`ns` field). A leading '~' is automatically
		* replaced by the node name (e.g. '~' turns into '/node_name', and '~foo' turns into '/node_name/foo').
		* @param dynamic Set to true if constructing the marker as a dynamic marker (Default: false).
		**/
		explicit SphereMarker(MarkerManager* MM, const std::string& frameID, double diameter, const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE, bool dynamic = false) : GenMarker(MM, frameID, markerNamespace, dynamic)
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
				setPosition(x, y, z);
				if(!dynamic)
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
		* @param markerNamespace The namespace to use for the marker (`ns` field). A leading '~' is automatically
		* replaced by the node name (e.g. '~' turns into '/node_name', and '~foo' turns into '/node_name/foo').
		* @param dynamic Set to true if constructing the marker as a dynamic marker (Default: false).
		**/
		explicit CubeMarker(MarkerManager* MM, const std::string& frameID = "", const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE, bool dynamic = false) : GenMarker(MM, frameID, markerNamespace, dynamic)
		{
			// Set marker properties
			setType(visualization_msgs::Marker::CUBE);
			setScale(0.025);
		}

		/**
		* @brief Specialised constructor.
		*
		* @param MM A pointer to the MarkerManager that should own this marker. If this parameter is null
		* then a segmentation fault will be of likely consequence.
		* @param frameID The name of the coordinate frame in which the marker should be published (e.g. "/odom").
		* If it is desired for the marker to be continually retransformed into this coordinate frame for the
		* duration of its lifetime, then use the `setFrameLocked()` function with argument `true`.
		* @param size The required edge length of the cube marker (Default: 0.025).
		* @param markerNamespace The namespace to use for the marker (`ns` field). A leading '~' is automatically
		* replaced by the node name (e.g. '~' turns into '/node_name', and '~foo' turns into '/node_name/foo').
		* @param dynamic Set to true if constructing the marker as a dynamic marker (Default: false).
		**/
		explicit CubeMarker(MarkerManager* MM, const std::string& frameID, double size, const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE, bool dynamic = false) : GenMarker(MM, frameID, markerNamespace, dynamic)
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
				if(!dynamic)
					MM->add(this);
			}
		}
	};

	/**
	* @class BoxMarker
	*
	* @brief Encapsulates a rectangular box marker object. By default sets up a box marker where each dimension is 0.20.
	*
	* Avoid mixing use of the two `update()` overloads or the orientation of the BoxMarker may not end up as expected.
	**/
	class BoxMarker : public GenMarker
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
		* @param markerNamespace The namespace to use for the marker (`ns` field). A leading '~' is automatically
		* replaced by the node name (e.g. '~' turns into '/node_name', and '~foo' turns into '/node_name/foo').
		* @param dynamic Set to true if constructing the marker as a dynamic marker (Default: false).
		**/
		explicit BoxMarker(MarkerManager* MM, const std::string& frameID = "", const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE, bool dynamic = false) : GenMarker(MM, frameID, markerNamespace, dynamic)
		{
			// Set marker properties
			setType(visualization_msgs::Marker::CUBE);
			setScale(0.20, 0.20, 0.20);
		}

		/**
		* @brief Specialised constructor.
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
		* @param dynamic Set to true if constructing the marker as a dynamic marker (Default: false).
		**/
		explicit BoxMarker(MarkerManager* MM, const std::string& frameID, double sizeX, double sizeY, double sizeZ, const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE, bool dynamic = false) : GenMarker(MM, frameID, markerNamespace, dynamic)
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
				if(!dynamic)
					MM->add(this);
			}
		}
	};

	/**
	* @class CylinderMarker
	*
	* @brief Encapsulates a cylinder marker object. By default sets up a cylinder marker where height and diameter is 0.20.
	*
	* Avoid mixing use of the two `update()` overloads or the orientation of the CylinderMarker may not end up as expected.
	**/
	class CylinderMarker : public GenMarker
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
		* @param markerNamespace The namespace to use for the marker (`ns` field). A leading '~' is automatically
		* replaced by the node name (e.g. '~' turns into '/node_name', and '~foo' turns into '/node_name/foo').
		* @param dynamic Set to true if constructing the marker as a dynamic marker (Default: false).
		**/
		explicit CylinderMarker(MarkerManager* MM, const std::string& frameID = "", const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE, bool dynamic = false) : GenMarker(MM, frameID, markerNamespace, dynamic)
		{
			// Set marker properties
			setType(visualization_msgs::Marker::CYLINDER);
			setScale(0.20, 0.20, 0.20);
		}

		/**
		* @brief Specialised constructor.
		*
		* @param MM A pointer to the MarkerManager that should own this marker. If this parameter is null
		* then a segmentation fault will be of likely consequence.
		* @param frameID The name of the coordinate frame in which the marker should be published (e.g. "/odom").
		* If it is desired for the marker to be continually retransformed into this coordinate frame for the
		* duration of its lifetime, then use the `setFrameLocked()` function with argument `true`.
		* @param height The required height of the cylinder (Default: 0.20).
		* @param diameter The required diameter of the cylinder (Default: 0.20).
		* @param markerNamespace The namespace to use for the marker (`ns` field). A leading '~' is automatically
		* replaced by the node name (e.g. '~' turns into '/node_name', and '~foo' turns into '/node_name/foo').
		* @param dynamic Set to true if constructing the marker as a dynamic marker (Default: false).
		**/
		explicit CylinderMarker(MarkerManager* MM, const std::string& frameID, double height, double diameter, const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE, bool dynamic = false) : GenMarker(MM, frameID, markerNamespace, dynamic)
		{
			// Set marker properties
			setType(visualization_msgs::Marker::CYLINDER);
			setScale(diameter, diameter, height);
		}

		//! @brief Update function overload: Sets the position of the marker only (the orientation should have been set at some point using `setOrientation`, e.g. during construction)
		void update(double x, double y, double z)
		{
			// Update the marker
			if(MM->willPublish())
			{
				setPosition(x, y, z);
				if(!dynamic)
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
				if(!dynamic)
					MM->add(this);
			}
		}
		//! @brief Update function overload: Sets the position and orientation of the marker so that it joins two points in space
		void update(double fromX, double fromY, double fromZ, double toX, double toY, double toZ)
		{
			// Don't do anything if we're not going to publish
			if(!MM->willPublish())
				return;
			
			// Calculate the difference between the from and to points
			double diffX = toX - fromX;
			double diffY = toY - fromY;
			double diffZ = toZ - fromZ;
			double diffLen = sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
			double diffLenXY = sqrt(diffX*diffX + diffY*diffY);
			
			// Don't plot the marker if it is supposed to have zero length
			if(diffLen <= 0.0)
				return;
			
			// Calculate the angle of rotation
			double ratio = diffZ / diffLen;
			if(ratio > 1.0) ratio = 1.0;
			else if(ratio < -1.0) ratio = -1.0;
			double alpha = acos(ratio);
			double chalpha = cos(0.5*alpha);
			double shalpha = sin(0.5*alpha);
			
			// Set the orientation of the marker
			setOrientation(chalpha, -shalpha*diffY/diffLenXY, shalpha*diffX/diffLenXY, 0.0);
			
			// Set the position of the marker
			setPosition(0.5*(fromX + toX), 0.5*(fromY + toY), 0.5*(fromZ + toZ));
			
			// Update the marker
			if(!dynamic)
				MM->add(this);
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
		* @param markerNamespace The namespace to use for the marker (`ns` field). A leading '~' is automatically
		* replaced by the node name (e.g. '~' turns into '/node_name', and '~foo' turns into '/node_name/foo').
		* @param dynamic Set to true if constructing the marker as a dynamic marker (Default: false).
		**/
		explicit ArrowMarker(MarkerManager* MM, const std::string& frameID = "", const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE, bool dynamic = false) : GenMarker(MM, frameID, markerNamespace, dynamic)
		{
			// Set marker properties
			setType(visualization_msgs::Marker::ARROW);
			setScale(0.025, 0.050, 0.0);

			// Initialise the points array from/to pair
			marker.points.resize(2);
			marker.points[0].x = 0.0;
			marker.points[0].y = 0.0;
			marker.points[0].z = 0.0;
			marker.points[1].x = 1.0;
			marker.points[1].y = 0.0;
			marker.points[1].z = 0.0;
		}

		/**
		* @brief Specialised constructor.
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
		* @param dynamic Set to true if constructing the marker as a dynamic marker (Default: false).
		**/
		explicit ArrowMarker(MarkerManager* MM, const std::string& frameID, double shaft_diam, double head_diam, double head_length = 0.0, const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE, bool dynamic = false) : GenMarker(MM, frameID, markerNamespace, dynamic)
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
				if(!dynamic)
					MM->add(this);
			}
		}
		//! @brief Update function overload: Sets the from (tail) and to (tip) coordinates of the arrow
		void update(double fromX, double fromY, double fromZ, double toX, double toY, double toZ)
		{
			// Update the marker
			if(MM->willPublish())
			{
				marker.points[0].x = fromX;
				marker.points[0].y = fromY;
				marker.points[0].z = fromZ;
				marker.points[1].x = toX;
				marker.points[1].y = toY;
				marker.points[1].z = toZ;
				if(!dynamic)
					MM->add(this);
			}
		}
	};
	
	/**
	* @class TextMarker
	*
	* @brief Encapsulates a text marker object. By default sets up an empty text marker.
	**/
	class TextMarker : public GenMarker
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
		* @param markerNamespace The namespace to use for the marker (`ns` field). A leading '~' is automatically
		* replaced by the node name (e.g. '~' turns into '/node_name', and '~foo' turns into '/node_name/foo').
		* @param dynamic Set to true if constructing the marker as a dynamic marker (Default: false).
		**/
		explicit TextMarker(MarkerManager* MM, const std::string& frameID = "", const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE, bool dynamic = false) : GenMarker(MM, frameID, markerNamespace, dynamic)
		{
			// Set marker properties
			setType(visualization_msgs::Marker::TEXT_VIEW_FACING);
			setScale(0.05);
		}

		/**
		* @brief Specialised constructor.
		*
		* @param MM A pointer to the MarkerManager that should own this marker. If this parameter is null
		* then a segmentation fault will be of likely consequence.
		* @param frameID The name of the coordinate frame in which the marker should be published (e.g. "/odom").
		* If it is desired for the marker to be continually retransformed into this coordinate frame for the
		* duration of its lifetime, then use the `setFrameLocked()` function with argument `true`.
		* @param size The font size of the text, namely the height of an uppercase 'A' (Default: 0.05).
		* @param markerNamespace The namespace to use for the marker (`ns` field). A leading '~' is automatically
		* replaced by the node name (e.g. '~' turns into '/node_name', and '~foo' turns into '/node_name/foo').
		* @param dynamic Set to true if constructing the marker as a dynamic marker (Default: false).
		**/
		explicit TextMarker(MarkerManager* MM, const std::string& frameID, double size, const std::string& markerNamespace = DEFAULT_MARKER_NAMESPACE, bool dynamic = false) : GenMarker(MM, frameID, markerNamespace, dynamic)
		{
			// Set marker properties
			setType(visualization_msgs::Marker::TEXT_VIEW_FACING);
			setScale(size);
		}

		//! @brief Update function overload: Sets the text of the marker
		void update(const std::string& text)
		{
			// Update the marker
			if(MM->willPublish())
			{
				setText(text);
				if(!dynamic)
					MM->add(this);
			}
		}
		//! @brief Update function overload: Sets the position of the marker
		void update(double x, double y, double z)
		{
			// Update the marker
			if(MM->willPublish())
			{
				setPosition(x, y, z);
				if(!dynamic)
					MM->add(this);
			}
		}
		//! @brief Update function overload: Sets the position and text of the marker
		void update(const std::string& text, double x, double y, double z)
		{
			// Update the marker
			if(MM->willPublish())
			{
				setText(text);
				setPosition(x, y, z);
				if(!dynamic)
					MM->add(this);
			}
		}
	};
}

#endif
// EOF