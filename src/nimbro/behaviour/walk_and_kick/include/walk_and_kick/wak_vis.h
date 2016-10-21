// Walk and kick: Visualisation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef WAK_VIS_H
#define WAK_VIS_H

// Includes
#include <walk_and_kick/wak_config.h>
#include <walk_and_kick/wak_utils.h>
#include <vis_utils/marker_manager.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class WAKMarkerMan
	* 
	* @brief Visualisation marker manager for the walk and kick node.
	**/
	class WAKMarkerMan : public vis_utils::MarkerManager
	{
	public:
		// Constants
		static const int NumCirclePoints = 51;

		// Constructor
		explicit WAKMarkerMan(WAKConfig& config);

		// Config parameters
		WAKConfig& config;

		// Field dimensions
		const FieldDimensions field;

		// Reference frames
		const std::string egoTrunk;
		const std::string egoFloor;
		const std::string alloFrame;

		// Function intercepts
		void clear(ros::Time stamp);
		void publish();

		// Helper functions
		void updateMarkerXY(vis_utils::GenMarker& marker, bool show, float x = 0.0f, float y = 0.0f, float a = 1.0f);

		// Publish a foreign marker array
		void publishMarkerArray(const visualization_msgs::MarkerArray& markers) { m_pub_markers.publish(markers); }

		// Markers
		vis_utils::SphereMarker BallTarget;
		vis_utils::SphereMarker Ball;
		vis_utils::TextMarker BallTargetType;
		vis_utils::BoxMarker BallTargetWidthBox;
		vis_utils::GenMarker BallToTargetLine;
		vis_utils::GenMarker BallToTargetWedge;
		vis_utils::GenMarker WalkingTarget;
		vis_utils::GenMarker WalkingTargetTol;
		vis_utils::CylinderMarker ObstacleA;
		vis_utils::CylinderMarker ObstacleB;
		vis_utils::ArrowMarker TargetPoseArrow;
		vis_utils::TextMarker ModeStateText;
		vis_utils::TextMarker GameStateText;
		vis_utils::TextMarker BehStateText;
		vis_utils::TextMarker SubStateText;
		vis_utils::BoxMarker GoalSign;
		vis_utils::ArrowMarker GcvXY;
		vis_utils::ArrowMarker GcvZ;
		vis_utils::ArrowMarker CompassHeading;
		vis_utils::TextMarker CompassHeadingText;
		vis_utils::BoxMarker SuggestedFoot;
		vis_utils::GenMarker ReqBallOffset;
		vis_utils::GenMarker GBBPath;
		vis_utils::GenMarker GBBPsiDes;
		vis_utils::GenMarker GBBBallView;
		vis_utils::GenMarker GBBFarCircle;
		vis_utils::GenMarker GBBNearCircle;
		vis_utils::GenMarker GBBHaloCircle;
		vis_utils::GenMarker GBBBehindBall;
		vis_utils::GenMarker GBBBetaAngle;
		vis_utils::GenMarker GBBRobotHalo;
		vis_utils::GenMarker DBBallRegion;
		vis_utils::GenMarker KBBallRegionL;
		vis_utils::GenMarker KBBallRegionR;
		vis_utils::ArrowMarker KBKickVector;

	private:
		// Hidden variables
		bool m_hiddenGBB;
	};

	/**
	* @class WAKBagMarkerMan
	* 
	* @brief Visualisation marker manager for visualisations that are intended for when playing behaviour bags.
	**/
	class WAKBagMarkerMan : public vis_utils::MarkerManager
	{
	public:
		// Constructor
		explicit WAKBagMarkerMan(WAKConfig& config);

		// Config parameters
		WAKConfig& config;

		// Reference frames
		const std::string egoTrunk;
		const std::string egoFloor;

		// Update functions
		void updateVis();
		void updateMarkers();

		// Markers
		vis_utils::SphereMarker RobotTrunk;
		vis_utils::BoxMarker RobotLeftFoot;
		vis_utils::BoxMarker RobotRightFoot;
		vis_utils::CylinderMarker RobotLeftToe;
		vis_utils::CylinderMarker RobotRightToe;
	};
}

#endif
// EOF