// Helper classes for the visualisations used by RobotControl
// File: robotcontrolvis.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef ROBOTCONTROLVIS_H
#define ROBOTCONTROLVIS_H

// Includes
#include <vis_utils/marker_manager.h>

// RobotControl namespace
namespace robotcontrol
{
	/**
	* @class RCMarkerMan
	*
	* @brief Marker manager class for the RobotControl node.
	*
	* This class defines the visualisation markers used by the RobotControl node,
	* and assists with their plotting.
	**/
	class RCMarkerMan : public vis_utils::MarkerManager
	{
	public:
		// Constructor
		RCMarkerMan() : MarkerManager("~vis_marker_array")
			, CoM(this, "/trunk_link", 0.05)
			, GyroVec(this, "/trunk_link", 0.02, 0.04, 0.07)
			, AccVec(this, "/trunk_link", 0.02, 0.04, 0.07)
			, MagVec(this, "/trunk_link", 0.02, 0.04, 0.07)
			, MagVec2D(this, "/trunk_link", 0.02, 0.04, 0.07)
		{
			// Initialise the RobotModel markers
			CoM.setColor(1.0, 0.0, 0.0);
			CoM.setFrameLocked(true);
			GyroVec.setColor(0.5, 0.0, 1.0);
			GyroVec.setFrameLocked(true);
			AccVec.setColor(0.0, 0.6, 0.0);
			AccVec.setFrameLocked(true);
			MagVec.setColor(0.0, 0.0, 1.0);
			MagVec.setFrameLocked(true);
			MagVec2D.setColor(0.0, 0.5, 1.0);
			MagVec2D.setFrameLocked(true);
		}

		// RobotModel markers
		vis_utils::SphereMarker CoM;
		vis_utils::ArrowMarker GyroVec;
		vis_utils::ArrowMarker AccVec;
		vis_utils::ArrowMarker MagVec;
		vis_utils::ArrowMarker MagVec2D;
	};
}

#endif /* ROBOTCONTROLVIS_H */
// EOF