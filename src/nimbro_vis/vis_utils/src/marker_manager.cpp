// Utilities for publishing markers
// File: marker_manager.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <vis_utils/marker_manager.h>

// Namespaces
using namespace vis_utils;

//
// MarkerManager
//

// Constants
const std::string MarkerManager::DEFAULT_TOPICNAME = "~vis_marker_array";

//
// GenMarker
//

// Constants
const std::string GenMarker::DEFAULT_MARKER_NAMESPACE = "~";

// Constructor
GenMarker::GenMarker(MarkerManager* MM, const std::string& frameID, const std::string& markerNamespace, bool dynamic) : MM(MM), dynamic(dynamic)
{
	// Refine the marker namespace
	std::string ns = markerNamespace;
	if(ns.empty()) ns = DEFAULT_MARKER_NAMESPACE;
	if(ns == "~") ns = ros::this_node::getName();
	else if(ns.at(0) == '~') ns.replace(0, 1, ros::this_node::getName() + "/");

	// Set the important marker fields
	setFrameID(frameID);
	marker.header.stamp = MM->m_stamp;
	marker.ns = ns;
	marker.id = (dynamic ? -1 : MM->getUniqueID());
	marker.action = visualization_msgs::Marker::MODIFY; // Equivalent to ADD...
	setType(visualization_msgs::Marker::SPHERE); // Sphere by default (instead of arrow by default, as the Marker implementation specifies)

	// Initialise the remaining fields
	setPosition(0.0, 0.0, 0.0);         // At origin by default
	setOrientation(1.0, 0.0, 0.0, 0.0); // Zero rotation by default
	setScale(1.0, 1.0, 1.0);            // No scaling by default
	setColor(1.0, 0.0, 0.0);            // Red by default
	setLifetime(0.2);                   // Persistent for only 200ms by default
	setFrameLocked(false);              // By default don't re-transform the marker into its frame at each time step
}

// Update function
void GenMarker::updateAdd()
{
	// Add the marker to the owning MarkerManager's marker array if it is going to be published in this step
	if(MM->willPublish())
	{
		// <-- In update() overloads, set any additional marker properties here
		MM->add(this);
	}
}

// Update a dynamic marker
void MarkerManager::updateDynamicMarker(int index, vis_utils::GenMarker& marker)
{
	// Set the ID and namespace of the marker
	IndexMap::const_iterator it = m_dynamicMap.find(index);
	if(it == m_dynamicMap.end())
	{
		newDynamicMarker(index);
		marker.marker.id = m_dynamicMap[index];
	}
	else
		marker.marker.id = it->second;

	// Add the marker for publishing in this cycle
	marker.updateAdd();
}
// EOF