// Utilities for plotting to the plotter widget
// File: plot_manager.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef PLOT_MANAGER_H
#define PLOT_MANAGER_H

// Includes
#include <string>
#include <ros/node_handle.h>
#include <ros/this_node.h> // Note: ros::this_node::getName() returns the node name with a '/' at the front!
#include <plot_msgs/Plot.h>
#include <Eigen/Core>

// Plot messages namespace
namespace plot_msgs
{
	/**
	 * @class PlotManager
	 *
	 * @brief Class that facilitates the plotting of data to the Plotter widget
	 *
	 * In general only one PlotManager or PlotManagerFS instance should be used per @b node, and all
	 * parts of the node should plot using the functions of that `PlotManager[FS]`. The basic process
	 * is this (assuming there is some kind of a step function that gets called in a timed main loop
	 * to do all the required processing of the node):
	 * @code
	 * // In class...
	 * plot_msgs::PlotManager PM;
	 *
	 * // In class constructor initialiser list... (only necessary at all if you don't want to use the default base name of "~", which expands to the node name)
	 * , PM("/my_base_name")
	 *
	 * // In the step function/main loop...
	 * PM.clear(); // <-- This sets the ROS timestamp of the points plotted in the next call to publish() (a ros::Time can be passed as an argument if the internal ros::Time::now() doesn't suffice)
	 * ...
	 * myvec = big_calculation();
	 * PM.plotVec3d(myvec, "my_vec");
	 * ...
	 * myresult = another_calculation();
	 * PM.plotScalar(myresult, "my_result");
	 * ...
	 * PM.plotEvent("state/my_event"); // <-- This event becomes '/my_base_name/events/state/my_event' in the tree hierarchy and 'my_event' in the plot window
	 * ...
	 * PM.publish();
	 * @endcode
	 * There should be no sleeping/ROS spinning between `PM.clear()` and `PM.publish()` or the
	 * timestamps get less accurate.
	 **/
	class PlotManager
	{
	public:
		/**
		 * @brief Default constructor
		 *
		 * @param baseName A `std::string` specifying the base name to use for all variables plotted by
		 * this class instance. This controls the location of the data in the plotter tree hierarchy.
		 * If the first character is @c ~ then it is replaced by the node name (e.g. @c ~foo becomes @c /node_name/foo/).
		 * @param enabled This controls the initial enabled state of the PlotManager. The enabled state
		 * can later be changed using the enable() and disable() functions, and controls whether the
		 * PlotManager actually publishes anything.
		 **/
		explicit PlotManager(const std::string& baseName = std::string(), bool enabled = true) : enabled(enabled)
		{
			// Terminate both ends of the basename with slashes
			basename = baseName;
			if(basename.empty()) basename = "~";
			if(basename.at(0) == '~') basename.replace(0, 1, ros::this_node::getName() + "/");
			if(basename.at(0) != '/') basename.insert(0, "/");
			if(basename.at(basename.length()-1) != '/') basename.insert(basename.length(), "/");

			// Work out the vector basenames
			basenamex = basename + "x/";
			basenamey = basename + "y/";
			basenamez = basename + "z/";

			// Advertise on the plot topic
			ros::NodeHandle nh("~");
			m_pub_plot = nh.advertise<plot_msgs::Plot>("/plot", 1);
		}

		// Get/set functions
		const std::string& getBasename() const { return basename; } //!< @brief Returns the base name in use for all variables plotted by this PlotManager instance. This controls the location of the data in the plotter tree hierarchy.
		bool getEnabled() const { return enabled; } //!< @brief Return the current enabled state of the PlotManager
		void enable()  { enabled = true; } //!< @brief Enables the PlotManager
		void disable() { enabled = false; } //!< @brief Disables the PlotManager

		// Plot functions
		//! @brief Clears the current internal plot/event data and sets the plot data timestamp to the current ROS time. Call one clear() overload at the start of every plot cycle.
		void clear()
		{
			// Clear the current plot/event data and set the timestamp
			clear(ros::Time::now());
		}
		//! @brief Clears the current internal plot/event data and sets the plot data timestamp to @p stamp. Call one clear() overload at the start of every plot cycle.
		void clear(ros::Time stamp)
		{
			// Clear the current plot/event data and set the timestamp
			m_plot.header.stamp = stamp;
			m_plot.points.clear();
			m_plot.events.clear();
		}
		//! @brief Sets the plot data timestamp to the current ROS time (this function should normally not be needed).
		void setTimestamp()
		{
			// Set the timestamp to the current ROS time
			setTimestamp(ros::Time::now());
		}
		//! @brief Sets the plot data timestamp to @p stamp (this function should normally not be needed).
		void setTimestamp(ros::Time stamp)
		{
			// Set the timestamp to the required value
			m_plot.header.stamp = stamp;
		}
		//! @brief Publishes the collected plot points (if the PlotManager is enabled). Call this at the end of every plot cycle (without a sleep between the call to clear and this or the timestamps get inaccurate).
		void publish()
		{
			// Publish the list to the configuration server
			if(enabled)
				m_pub_plot.publish(m_plot);
		}
		//! @brief Plot a scalar value under the given @p name
		void plotScalar(double value, const std::string& name)
		{
			// Package up the point and queue it away
			if(enabled)
			{
				m_point.name = basename + name;
				m_point.value = value;
				m_plot.points.push_back(m_point);
			}
		}
		//! @brief Plot a 2D Eigen vector under the given @p name
		void plotVec2d(const Eigen::Vector2d& vec2d, const std::string& name)
		{
			// Package up the 2D vector and queue it away
			if(enabled)
			{
				m_point.name = basenamex + name;
				m_point.value = vec2d.x();
				m_plot.points.push_back(m_point);
				m_point.name = basenamey + name;
				m_point.value = vec2d.y();
				m_plot.points.push_back(m_point);
			}
		}
		//! @brief Plot a 2D vector `(x,y)` under the given @p name
		void plotVec2d(double x, double y, const std::string& name)
		{
			// Package up the 2D vector and queue it away
			if(enabled)
			{
				m_point.name = basenamex + name;
				m_point.value = x;
				m_plot.points.push_back(m_point);
				m_point.name = basenamey + name;
				m_point.value = y;
				m_plot.points.push_back(m_point);
			}
		}
		//! @brief Plot a 3D Eigen vector under the given @p name
		void plotVec3d(const Eigen::Vector3d& vec3d, const std::string& name)
		{
			// Package up the 3D vector and queue it away
			if(enabled)
			{
				m_point.name = basenamex + name;
				m_point.value = vec3d.x();
				m_plot.points.push_back(m_point);
				m_point.name = basenamey + name;
				m_point.value = vec3d.y();
				m_plot.points.push_back(m_point);
				m_point.name = basenamez + name;
				m_point.value = vec3d.z();
				m_plot.points.push_back(m_point);
			}
		}
		//! @brief Plot a 3D vector `(x,y,z)` under the given @p name
		void plotVec3d(double x, double y, double z, const std::string& name)
		{
			// Package up the 3D vector and queue it away
			if(enabled)
			{
				m_point.name = basenamex + name;
				m_point.value = x;
				m_plot.points.push_back(m_point);
				m_point.name = basenamey + name;
				m_point.value = y;
				m_plot.points.push_back(m_point);
				m_point.name = basenamez + name;
				m_point.value = z;
				m_plot.points.push_back(m_point);
			}
		}

		// Event functions
		//! @brief Add an event to the current plot message (the event appears in the tree hierarchy as `basename/events/name`, and is labelled in the plot window with the last part of `name` that doesn't contain a `'/'`)
		void plotEvent(const std::string& name)
		{
			// Add the required event
			if(enabled)
				m_plot.events.push_back(basename + "events/" + name);
		}

	private:
		// Internal variables
		ros::Publisher m_pub_plot;
		plot_msgs::Plot m_plot;
		plot_msgs::PlotPoint m_point;
		std::string basename, basenamex, basenamey, basenamez;
		bool enabled;
	};

	/**
	 * @class PlotManagerFS
	 *
	 * @brief Class that facilitates the plotting of data to the Plotter widget, but using a fixed-size internal array of plot points
	 *
	 * This class limits the functionality of the PlotManager to a constant number of plot points, but
	 * in return is more lightweight as the internal plot array never has to be repopulated.
	 *
	 * In general only one PlotManager or PlotManagerFS instance should be used per @b node, and all
	 * parts of the node should plot using the functions of that `PlotManager[FS]`. The basic process
	 * is this (assuming there is some kind of a step function that gets called in a timed main loop
	 * to do all the required processing of the node):
	 * @code
	 * // In class...
	 * enum PMIds
	 * {
	 * 	PM_MYRESULT = 0,
	 * 	PM_MYVEC,
	 * 	PM_MYVEC_X = PM_MYVEC,
	 * 	PM_MYVEC_Y,
	 * 	PM_MYVEC_Z,
	 * 	PM_COUNT
	 * };
	 * plot_msgs::PlotManagerFS PM;
	 *
	 * // In class constructor initialiser list... (the second parameter is only necessary if you don't want to use the default base name of "~", which expands to the node name)
	 * , PM(PM_COUNT, "/my_base_name")
	 *
	 * // In class constructor... (this initialises the point names and data values)
	 * PM.plotVec3d(0.0, 0.0, 0.0, PM_MYVEC, "my_vec");
	 * PM.plotScalar(0.0, PM_MYRESULT, "my_result");
	 * // Alternatively for scalar: PM.setName(PM_MYRESULT, "my_result"); // Note: This alternative doesn't initialise the point value
	 *
	 * // In the step function/main loop...
	 * PM.clear(); // <-- This sets the ROS timestamp of the points plotted in the next call to publish() (a ros::Time can be passed as an argument if the internal ros::Time::now() doesn't suffice)
	 * ...
	 * myvec = big_calculation();
	 * PM.plotVec3d(myvec, PM_MYVEC);
	 * ...
	 * myresult = another_calculation();
	 * PM.plotScalar(myresult, PM_MYRESULT);
	 * ...
	 * PM.plotEvent("state/my_event"); // <-- This event becomes '/my_base_name/events/state/my_event' in the tree hierarchy and 'my_event' in the plot window
	 * ...
	 * PM.publish();
	 * @endcode
	 * There should be no sleeping/ROS spinning between `PM.clear()` and `PM.publish()` or the
	 * timestamps get less accurate.
	 **/
	class PlotManagerFS
	{
	public:
		/**
		 * @brief Default constructor
		 *
		 * This constructor initialises the internal plot point array to @p num_points copies of a zero-valued
		 * point with an empty string name. If the name is not passed to the plot functions in your main loop
		 * (which would mean that the full plot name is recalculated for each point in each cycle), then the
		 * names can be initialised as a one-off by a single named call to the respective plot functions.
		 * The setName() function is also provided but less recommended, especially when plotting vectors etc.
		 * To keep the ID's clear to the user, it is recommended that you use an enumeration to maintain the
		 * ID numbers. Refer to the documentation of PlotManagerFS for an example.
		 *
		 * @param num_points The number of plot points to allocate space for.
		 * @param baseName A `std::string` specifying the base name to use for all variables plotted by
		 * this class instance. This controls the location of the data in the plotter tree hierarchy.
		 * If the first character is @c ~ then it is replaced by the node name (e.g. @c ~foo becomes @c /node_name/foo/).
		 * @param enabled This controls the initial enabled state of the PlotManagerFS. The enabled state
		 * can later be changed using the enable() and disable() functions, and controls whether the
		 * PlotManagerFS actually publishes anything.
		 **/
		explicit PlotManagerFS(unsigned int num_points, const std::string& baseName = std::string(), bool enabled = true) : size(num_points), enabled(enabled)
		{
			// Terminate both ends of the basename with slashes
			basename = baseName;
			if(basename.empty()) basename = "~";
			if(basename.at(0) == '~') basename.replace(0, 1, ros::this_node::getName() + "/");
			if(basename.at(0) != '/') basename.insert(0, "/");
			if(basename.at(basename.length()-1) != '/') basename.insert(basename.length(), "/");

			// Work out the vector basenames
			basenamex = basename + "x/";
			basenamey = basename + "y/";
			basenamez = basename + "z/";

			// Initialise m_plot with a suitable number of dud points
			plot_msgs::PlotPoint point;
			point.name = "";
			point.value = 0.0;
			if(size > 0) m_plot.points.assign(size, point);

			// Advertise on the plot topic
			ros::NodeHandle nh("~");
			m_pub_plot = nh.advertise<plot_msgs::Plot>("/plot", 1);
		}

		// Get/set functions
		const std::string& getBasename() const { return basename; } //!< @brief Returns the base name in use for all variables plotted by this PlotManagerFS instance. This controls the location of the data in the plotter tree hierarchy.
		std::string getName(unsigned int id) const { return (id < size ? m_plot.points[id].name : ""); } //!< @brief Retrieves the current name of the nominated point in the internal point array.
		void setName(unsigned int id, const std::string& name) { if(id < size) m_plot.points[id].name = basename + name; } //!< @brief Sets the name of the nominated point in the internal point array (avoid using this function to modify the names of points that are published to by the vector plot functions, use an initialisation call to the vector plot function instead).
		bool getEnabled() const { return enabled; } //!< @brief Return the current enabled state of the PlotManagerFS
		void enable()  { enabled = true; } //!< @brief Enables the PlotManagerFS
		void disable() { enabled = false; } //!< @brief Disables the PlotManagerFS

		// Plot functions
		//! @brief Clears the events list and updates the current header stamp to the current ROS time. Call one clear() overload at the start of every plot cycle.
		void clear()
		{
			// Update the header stamp to the current ROS time
			clear(ros::Time::now());
		}
		//! @brief Clears the events list and updates the current header stamp to @p stamp. Call one clear() overload at the start of every plot cycle.
		void clear(ros::Time stamp)
		{
			// Update the header stamp to the given time
			m_plot.header.stamp = stamp;
			m_plot.events.clear();
		}
		//! @brief Sets the plot data timestamp to the current ROS time (this function should normally not be needed).
		void setTimestamp()
		{
			// Set the timestamp to the current ROS time
			setTimestamp(ros::Time::now());
		}
		//! @brief Sets the plot data timestamp to @p stamp (this function should normally not be needed).
		void setTimestamp(ros::Time stamp)
		{
			// Set the timestamp to the required value
			m_plot.header.stamp = stamp;
		}
		//! @brief Publishes the stored plot points (if the PlotManagerFS is enabled). Call this at the end of every plot cycle (without a sleep between the call to clear and this or the timestamps get inaccurate).
		void publish()
		{
			// Publish the list to the configuration server
			if(enabled)
				m_pub_plot.publish(m_plot);
		}
		//! @brief Plot a scalar value at the given @p id. The point name is unmodified if @p name is omitted.
		void plotScalar(double value, unsigned int id, const std::string& name = "")
		{
			// Package up the point and queue it away
			if(enabled && (id < size))
			{
				if(!name.empty())
					m_plot.points[id].name = basename + name;
				m_plot.points[id].value = value;
			}
		}
		//! @brief Plot a 2D Eigen vector at the given @p id (takes up `id` and `id + 1`). The point names are unmodified if @p name is omitted.
		void plotVec2d(const Eigen::Vector2d& vec2d, unsigned int id, const std::string& name = "")
		{
			// Package up the 2D vector and queue it away
			if(enabled && (id+1 < size))
			{
				if(!name.empty())
				{
					m_plot.points[id  ].name = basenamex + name;
					m_plot.points[id+1].name = basenamey + name;
				}
				m_plot.points[id  ].value = vec2d.x();
				m_plot.points[id+1].value = vec2d.y();
			}
		}
		//! @brief Plot a 2D vector `(x,y)` at the given @p id (takes up `id` and `id + 1`). The point names are unmodified if @p name is omitted.
		void plotVec2d(double x, double y, unsigned int id, const std::string& name = "")
		{
			// Package up the 2D vector and queue it away
			if(enabled && (id+1 < size))
			{
				if(!name.empty())
				{
					m_plot.points[id  ].name = basenamex + name;
					m_plot.points[id+1].name = basenamey + name;
				}
				m_plot.points[id  ].value = x;
				m_plot.points[id+1].value = y;
			}
		}
		//! @brief Plot a 3D Eigen vector at the given @p id (takes up `id`, `id + 1` and `id + 2`). The point names are unmodified if @p name is omitted.
		void plotVec3d(const Eigen::Vector3d& vec3d, unsigned int id, const std::string& name = "")
		{
			// Package up the 3D vector and queue it away
			if(enabled && (id+2 < size))
			{
				if(!name.empty())
				{
					m_plot.points[id  ].name = basenamex + name;
					m_plot.points[id+1].name = basenamey + name;
					m_plot.points[id+2].name = basenamez + name;
				}
				m_plot.points[id  ].value = vec3d.x();
				m_plot.points[id+1].value = vec3d.y();
				m_plot.points[id+2].value = vec3d.z();
			}
		}
		//! @brief Plot a 3D vector `(x,y,z)` at the given @p id (takes up `id`, `id + 1` and `id + 2`). The point names are unmodified if @p name is omitted.
		void plotVec3d(double x, double y, double z, unsigned int id, const std::string& name = "")
		{
			// Package up the 3D vector and queue it away
			if(enabled && (id+2 < size))
			{
				if(!name.empty())
				{
					m_plot.points[id  ].name = basenamex + name;
					m_plot.points[id+1].name = basenamey + name;
					m_plot.points[id+2].name = basenamez + name;
				}
				m_plot.points[id  ].value = x;
				m_plot.points[id+1].value = y;
				m_plot.points[id+2].value = z;
			}
		}

		// Event functions
		//! @brief Add an event to the current plot message (the event appears in the tree hierarchy as `basename/events/name`, and is labelled in the plot window with the last part of `name` that doesn't contain a `'/'`)
		void plotEvent(const std::string& name)
		{
			// Add the required event
			if(enabled)
				m_plot.events.push_back(basename + "events/" + name);
		}

	private:
		// Internal variables
		const unsigned int size;
		ros::Publisher m_pub_plot;
		plot_msgs::Plot m_plot;
		std::string basename, basenamex, basenamey, basenamez;
		bool enabled;
	};
}

#endif /* PLOT_MANAGER_H */
// EOF