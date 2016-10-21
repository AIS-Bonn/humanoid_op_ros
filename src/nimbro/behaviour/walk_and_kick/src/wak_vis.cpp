// Walk and kick: Visualisation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/wak_vis.h>

// Defines
#define GOAL_HEIGHT        0.80
#define GOAL_DIAMETER      0.12
#define TEXT_FONT_HEIGHT   0.10
#define TEXT_POSZ_MODE     (0.39 + 0*0.10)
#define TEXT_POSZ_GAME     (0.39 + 1*0.10)
#define TEXT_POSZ_BEH      (0.39 + 2*0.10)
#define TEXT_POSZ_SUB      (0.39 + 3*0.10)
#define TEXT_OFFZ_BT       0.08
#define BTW_BOX_SIZE       0.03
#define BS_BOX_SIZEX       0.10
#define BS_BOX_SIZEZ       0.03
#define ARROW_SHAFT_DIAM   0.025
#define ARROW_HEAD_DIAM    0.050
#define ARROW_HEAD_LENGTH  0.0
#define FAT_SHAFT_DIAM     0.15
#define FAT_HEAD_DIAM      0.25
#define FAT_HEAD_LENGTH    0.17
#define LINE_THICKNESS     0.015
#define SPHERE_SIZE        0.050
#define BALLOFFSET_SIZE    0.03
#define SUGGFOOT_BOXLEN    0.30
#define FOOT_THICK         0.02
#define TOE_DIAM           0.03
#define WALK_TARGET_SIZE   0.1
#define OBSTACLE_HEIGHT    0.7
#define OBSTACLE_DIAMETER  0.2

// Namespaces
using namespace walk_and_kick;

//
// WAKMarkerMan class
//

// Constructor
WAKMarkerMan::WAKMarkerMan(WAKConfig& config)
 : MarkerManager("~behaviour_markers", 1, true)
 , config(config)
 , field()
 , egoTrunk("/ego_rot")
 , egoFloor("/ego_floor")
 , alloFrame("/beh_field")
 , BallTarget(this, egoFloor, 1.0, "BallTarget")
 , Ball(this, egoFloor, 1.0, "Ball")
 , BallTargetType(this, egoFloor, TEXT_FONT_HEIGHT, "BallTarget")
 , BallTargetWidthBox(this, egoFloor, BTW_BOX_SIZE, 1.0, BTW_BOX_SIZE, "BallTarget")
 , BallToTargetLine(this, egoFloor, "BallTarget")
 , BallToTargetWedge(this, egoFloor, "BallTarget")
 , WalkingTarget(this, egoFloor, "TargetPose")
 , WalkingTargetTol(this, egoFloor, "TargetPose")
 , ObstacleA(this, egoFloor, OBSTACLE_HEIGHT, OBSTACLE_DIAMETER, "Obstacle")
 , ObstacleB(this, egoFloor, OBSTACLE_HEIGHT, OBSTACLE_DIAMETER, "Obstacle")
 , TargetPoseArrow(this, alloFrame, FAT_SHAFT_DIAM, FAT_HEAD_DIAM, FAT_HEAD_LENGTH, "TargetPose")
 , ModeStateText(this, egoTrunk, TEXT_FONT_HEIGHT, "BehaviourState")
 , GameStateText(this, egoTrunk, TEXT_FONT_HEIGHT, "BehaviourState")
 , BehStateText(this, egoTrunk, TEXT_FONT_HEIGHT, "BehaviourState")
 , SubStateText(this, egoTrunk, TEXT_FONT_HEIGHT, "BehaviourState")
 , GoalSign(this, alloFrame, BS_BOX_SIZEX, field.fieldWidth(), BS_BOX_SIZEZ, "GameState")
 , GcvXY(this, egoTrunk, ARROW_SHAFT_DIAM, ARROW_HEAD_DIAM, ARROW_HEAD_LENGTH, "GCV")
 , GcvZ(this, egoTrunk, ARROW_SHAFT_DIAM, ARROW_HEAD_DIAM, ARROW_HEAD_LENGTH, "GCV")
 , CompassHeading(this, egoFloor, ARROW_SHAFT_DIAM, ARROW_HEAD_DIAM, ARROW_HEAD_LENGTH, "Compass")
 , CompassHeadingText(this, egoFloor, TEXT_FONT_HEIGHT, "Compass")
 , SuggestedFoot(this, egoFloor, SUGGFOOT_BOXLEN, FOOT_THICK, FOOT_THICK, "BehShared")
 , ReqBallOffset(this, egoFloor, "BehShared")
 , GBBPath(this, egoFloor, "GoBehindBall")
 , GBBPsiDes(this, egoFloor, "GoBehindBallDetail")
 , GBBBallView(this, egoFloor, "GoBehindBall")
 , GBBFarCircle(this, egoFloor, "GoBehindBallDetail")
 , GBBNearCircle(this, egoFloor, "GoBehindBallDetail")
 , GBBHaloCircle(this, egoFloor, "GoBehindBallDetail")
 , GBBBehindBall(this, egoFloor, "GoBehindBall")
 , GBBBetaAngle(this, egoFloor, "GoBehindBallDetail")
 , GBBRobotHalo(this, egoFloor, "GoBehindBallDetail")
 , DBBallRegion(this, egoFloor, "DribbleBall")
 , KBBallRegionL(this, egoFloor, "KickBall")
 , KBBallRegionR(this, egoFloor, "KickBall")
 , KBKickVector(this, egoFloor, ARROW_SHAFT_DIAM, ARROW_HEAD_DIAM, 6.0*ARROW_SHAFT_DIAM, "KickBall")
 , m_hiddenGBB(false)
{
	// Configure the markers as required
	Ball.setColor(0.88235, 0.19608, 0.0);
	BallTarget.setColor(0.5, 0.11, 0.0);
	BallTargetType.setColor(0.5, 0.11, 0.0);
	BallTargetWidthBox.setColor(0.5, 0.11, 0.0);
	BallToTargetLine.setType(visualization_msgs::Marker::LINE_LIST);
	BallToTargetLine.setScale(LINE_THICKNESS);
	BallToTargetLine.setNumPoints(4);
	BallToTargetLine.setNumPtColors(4);
	BallToTargetLine.setPtColor(0, 0.5, 0.3, 0.2);
	BallToTargetLine.setPtColor(1, 0.5, 0.3, 0.2);
	BallToTargetLine.setPtColor(2, 0.7, 0.5, 0.4);
	BallToTargetLine.setPtColor(3, 0.7, 0.5, 0.4);
	BallToTargetWedge.setType(visualization_msgs::Marker::LINE_STRIP);
	BallToTargetWedge.setColor(0.5, 0.3, 0.2);
	BallToTargetWedge.setScale(LINE_THICKNESS);
	BallToTargetWedge.setNumPoints(3);
	WalkingTarget.setType(visualization_msgs::Marker::LINE_LIST);
	WalkingTarget.setScale(LINE_THICKNESS * 2.0);
	WalkingTarget.setNumPoints(4);
	WalkingTarget.setPoint(0, WALK_TARGET_SIZE, WALK_TARGET_SIZE);
	WalkingTarget.setPoint(1, -WALK_TARGET_SIZE, -WALK_TARGET_SIZE);
	WalkingTarget.setPoint(2, WALK_TARGET_SIZE, -WALK_TARGET_SIZE);
	WalkingTarget.setPoint(3, -WALK_TARGET_SIZE, WALK_TARGET_SIZE);
	WalkingTargetTol.setType(visualization_msgs::Marker::LINE_STRIP);
	WalkingTargetTol.setScale(LINE_THICKNESS);
	WalkingTargetTol.setColor(0.8, 0.0, 0.8);
	WalkingTargetTol.setNumPoints(NumCirclePoints);
	ObstacleA.setColor(0.0, 0.0, 0.4);
	ObstacleA.setPosition(0.0, 0.0, 0.5*OBSTACLE_HEIGHT);
	ObstacleB.setColor(0.13, 0.13, 0.13);
	ObstacleB.setPosition(0.0, 0.0, 0.5*OBSTACLE_HEIGHT);
	TargetPoseArrow.setColor(0.8, 0.0, 0.8);
	ModeStateText.setPosition(0.0, 0.0, TEXT_POSZ_MODE);
	GameStateText.setPosition(0.0, 0.0, TEXT_POSZ_GAME);
	BehStateText.setPosition(0.0, 0.0, TEXT_POSZ_BEH);
	SubStateText.setPosition(0.0, 0.0, TEXT_POSZ_SUB);
	ModeStateText.setColor(0.8, 0.0, 0.0);
	GameStateText.setColor(0.0, 0.25, 0.0);
	BehStateText.setColor(0.6, 0.0, 1.0);
	SubStateText.setColor(0.0, 0.0, 1.0);
	GoalSign.setPosition(field.fieldLengthH() + field.boundary() - 0.5*BS_BOX_SIZEX, 0.0, 0.5*BS_BOX_SIZEZ);
	GcvXY.setColor(0.0, 0.6, 0.6);
	GcvZ.setColor(0.6, 0.2, 1.0);
	CompassHeading.setColor(0.3, 0.3, 0.3);
	CompassHeadingText.setColor(0.3, 0.3, 0.3);
	CompassHeadingText.setText("N");
	SuggestedFoot.setColor(0.8, 0.6, 0.0);
	SuggestedFoot.setPosition(0.0, 0.0, 0.5*FOOT_THICK);
	ReqBallOffset.setType(visualization_msgs::Marker::SPHERE_LIST);
	ReqBallOffset.setColor(0.8, 0.6, 0.0);
	ReqBallOffset.setScale(BALLOFFSET_SIZE);
	ReqBallOffset.setNumPoints(2);
	GBBPath.setType(visualization_msgs::Marker::LINE_STRIP);
	GBBPath.setScale(LINE_THICKNESS);
	GBBPath.setColor(1.0, 0.0, 0.0);
	GBBPsiDes.setType(visualization_msgs::Marker::LINE_STRIP);
	GBBPsiDes.setScale(LINE_THICKNESS);
	GBBPsiDes.setColor(0.9, 0.8, 0);
	GBBPsiDes.setNumPoints(2);
	GBBBallView.setType(visualization_msgs::Marker::LINE_STRIP);
	GBBBallView.setScale(LINE_THICKNESS);
	GBBBallView.setColor(0.7, 0.7, 0.7);
	GBBBallView.setNumPoints(3);
	GBBFarCircle.setType(visualization_msgs::Marker::LINE_STRIP);
	GBBFarCircle.setScale(LINE_THICKNESS);
	GBBFarCircle.setColor(0.4, 0.6, 1.0);
	GBBFarCircle.setNumPoints(NumCirclePoints);
	GBBNearCircle.setType(visualization_msgs::Marker::LINE_STRIP);
	GBBNearCircle.setScale(LINE_THICKNESS);
	GBBNearCircle.setColor(0.4, 0.6, 1.0);
	GBBNearCircle.setNumPoints(NumCirclePoints);
	GBBHaloCircle.setType(visualization_msgs::Marker::LINE_STRIP);
	GBBHaloCircle.setScale(LINE_THICKNESS);
	GBBHaloCircle.setColor(0.4940, 0.1840, 0.5560);
	GBBHaloCircle.setNumPoints(NumCirclePoints);
	GBBBehindBall.setType(visualization_msgs::Marker::LINE_STRIP);
	GBBBehindBall.setScale(LINE_THICKNESS);
	GBBBehindBall.setColor(0.0, 0.0, 1.0);
	GBBBehindBall.setNumPoints(3);
	GBBBetaAngle.setType(visualization_msgs::Marker::LINE_STRIP);
	GBBBetaAngle.setScale(LINE_THICKNESS);
	GBBBetaAngle.setColor(1.0, 0.0, 1.0);
	GBBBetaAngle.setNumPoints(3);
	GBBRobotHalo.setType(visualization_msgs::Marker::LINE_STRIP);
	GBBRobotHalo.setScale(LINE_THICKNESS);
	GBBRobotHalo.setColor(0.2, 0.2, 0.2);
	GBBRobotHalo.setNumPoints(NumCirclePoints);
	DBBallRegion.setType(visualization_msgs::Marker::LINE_STRIP);
	DBBallRegion.setScale(LINE_THICKNESS);
	DBBallRegion.setNumPoints(2*(30 + 1) + 3); // Note: This should match with the uses of this marker in the BehDribbleBall class
	KBBallRegionL.setType(visualization_msgs::Marker::LINE_STRIP);
	KBBallRegionL.setScale(LINE_THICKNESS);
	KBBallRegionL.setNumPoints(5);
	KBBallRegionR.setType(visualization_msgs::Marker::LINE_STRIP);
	KBBallRegionR.setScale(LINE_THICKNESS);
	KBBallRegionR.setNumPoints(5);
	KBKickVector.setColor(1.0, 0.5, 0.0);

	// Set properties of the markers that depend on the ball diameter
	double ballDiameter = field.ballDiameter();
	double ballRadius = 0.5*ballDiameter;
	Ball.setScale(ballDiameter);
	Ball.setPosition(0.0, 0.0, ballRadius);
	BallTarget.setScale(ballDiameter);
	BallTarget.setPosition(0.0, 0.0, ballRadius);
	BallTargetType.setPosition(0.0, 0.0, ballDiameter + TEXT_OFFZ_BT);
	BallTargetWidthBox.setPosition(0.0, 0.0, ballRadius);
	BallToTargetLine.marker.points[0].z = ballRadius;
	BallToTargetLine.marker.points[1].z = ballRadius;
	BallToTargetLine.marker.points[2].z = ballRadius + LINE_THICKNESS;
	BallToTargetLine.marker.points[3].z = ballRadius + LINE_THICKNESS;
	for(size_t i = 0; i < BallToTargetWedge.marker.points.size(); i++)
		BallToTargetWedge.marker.points[i].z = ballRadius;
	for(size_t i = 0; i < WalkingTarget.marker.points.size(); i++)
		WalkingTarget.marker.points[i].z = LINE_THICKNESS;
	WalkingTargetTol.setPosition(0.0, 0.0, 0.5f*LINE_THICKNESS);
	for(size_t i = 0; i < ReqBallOffset.marker.points.size(); i++)
		ReqBallOffset.marker.points[i].z = ballRadius;
	GBBPath.setPosition(0.0, 0.0, ballRadius + LINE_THICKNESS);
	for(size_t i = 0; i < GBBPsiDes.marker.points.size(); i++)
		GBBPsiDes.marker.points[i].z = ballRadius;
	for(size_t i = 0; i < GBBBallView.marker.points.size(); i++)
		GBBBallView.marker.points[i].z = ballRadius;
	GBBFarCircle.setPosition(0.0, 0.0, ballRadius);
	GBBNearCircle.setPosition(0.0, 0.0, ballRadius);
	GBBHaloCircle.setPosition(0.0, 0.0, ballRadius);
	for(size_t i = 0; i < GBBBehindBall.marker.points.size(); i++)
		GBBBehindBall.marker.points[i].z = ballRadius + 0.5*LINE_THICKNESS;
	for(size_t i = 0; i < GBBBetaAngle.marker.points.size(); i++)
		GBBBetaAngle.marker.points[i].z = ballRadius + 0.5*LINE_THICKNESS;
	GBBRobotHalo.setPosition(0.0, 0.0, ballRadius);
	for(size_t i = 0; i < DBBallRegion.marker.points.size(); i++)
		DBBallRegion.marker.points[i].z = ballRadius + LINE_THICKNESS;
	for(size_t i = 0; i < KBBallRegionL.marker.points.size(); i++)
		KBBallRegionL.marker.points[i].z = ballRadius + LINE_THICKNESS;
	for(size_t i = 0; i < KBBallRegionR.marker.points.size(); i++)
		KBBallRegionR.marker.points[i].z = ballRadius + LINE_THICKNESS;
	KBKickVector.setPoint(0, 0.0, 0.0, ballRadius + LINE_THICKNESS);
	KBKickVector.setPoint(1, 0.0, 0.0, ballRadius + LINE_THICKNESS);
}

// Intercepted clear function
void WAKMarkerMan::clear(ros::Time stamp)
{
	// Pass on the work to the base implementation
	vis_utils::MarkerManager::clear(stamp);

	// Hide a selected group of markers by default
	BallToTargetWedge.hide();
	GBBPath.hide();
	GBBPsiDes.hide();
	GBBBallView.hide();
	GBBFarCircle.hide();
	GBBNearCircle.hide();
	GBBHaloCircle.hide();
	GBBBehindBall.hide();
	GBBBetaAngle.hide();
	GBBRobotHalo.hide();
	DBBallRegion.hide();
	KBBallRegionL.hide();
	KBBallRegionR.hide();
	KBKickVector.hide();
}

// Intercepted publish function
void WAKMarkerMan::publish()
{
	// Add a selected group of markers by default
	BallToTargetWedge.updateAdd();
	if(!(GBBPath.hidden() && m_hiddenGBB))
	{
		m_hiddenGBB = GBBPath.hidden(); // If GBBPath is hidden then it is assumed/known that all others will be too...
		GBBPath.updateAdd();
		GBBPsiDes.updateAdd();
		GBBBallView.updateAdd();
		GBBFarCircle.updateAdd();
		GBBNearCircle.updateAdd();
		GBBHaloCircle.updateAdd();
		GBBBehindBall.updateAdd();
		GBBBetaAngle.updateAdd();
		GBBRobotHalo.updateAdd();
	}
	DBBallRegion.updateAdd();
	KBBallRegionL.updateAdd();
	KBBallRegionR.updateAdd();
	KBKickVector.updateAdd();

	// Pass on the work to the base implementation
	vis_utils::MarkerManager::publish();
}

// Update the visibility, 2D position and alpha value of a marker
void WAKMarkerMan::updateMarkerXY(vis_utils::GenMarker& marker, bool show, float x, float y, float a)
{
	// Update the marker as required
	if(show)
	{
		marker.marker.pose.position.x = x;
		marker.marker.pose.position.y = y;
		marker.marker.color.a = a;
		marker.show();
	}
	else
		marker.hide();
	marker.updateAdd();
}

//
// WAKMarkerMan class
//

// Constructor
WAKBagMarkerMan::WAKBagMarkerMan(WAKConfig& config)
 : MarkerManager("~behaviour_bag_markers", 1, true)
 , config(config)
 , egoTrunk("/ego_rot")
 , egoFloor("/ego_floor")
 , RobotTrunk(this, egoTrunk, SPHERE_SIZE, "Robot")
 , RobotLeftFoot(this, egoFloor, "Robot")
 , RobotRightFoot(this, egoFloor, "Robot")
 , RobotLeftToe(this, egoFloor, "Robot")
 , RobotRightToe(this, egoFloor, "Robot")
{
	// Configure the markers as required
	RobotTrunk.setColor(0.1, 0.1, 0.1);
	RobotLeftFoot.setColor(0.1, 0.1, 0.1);
	RobotRightFoot.setColor(0.1, 0.1, 0.1);
	RobotLeftToe.setColor(0.7, 0.1, 0.1);
	RobotRightToe.setColor(0.7, 0.1, 0.1);

	// Config parameter callbacks
	boost::function<void()> updateCb = boost::bind(&WAKBagMarkerMan::updateVis, this);
	config.visFootLength.setCallback(boost::bind(updateCb));
	config.visFootWidth.setCallback(boost::bind(updateCb));
	config.visFootOffsetX.setCallback(boost::bind(updateCb));
	config.visFootOffsetY.setCallback(boost::bind(updateCb));
	updateVis();
}

// Update function for visualisation
void WAKBagMarkerMan::updateVis()
{
	// Local variables
	float length = config.visFootLength();
	float width = config.visFootWidth();
	float offX = config.visFootOffsetX();
	float offY = config.visFootOffsetY();

	// Update the markers based on the config parameters
	RobotLeftFoot.setScale(length, width, FOOT_THICK);
	RobotLeftFoot.setPosition(offX, offY, 0.5*FOOT_THICK);
	RobotRightFoot.setScale(length, width, FOOT_THICK);
	RobotRightFoot.setPosition(offX, -offY, 0.5*FOOT_THICK);
	RobotLeftToe.setScale(TOE_DIAM, TOE_DIAM, width*0.99);
	RobotLeftToe.setPosition(offX + 0.5*(length - TOE_DIAM), offY, FOOT_THICK);
	RobotLeftToe.setOrientation(M_SQRT1_2, M_SQRT1_2, 0.0, 0.0);
	RobotRightToe.setScale(TOE_DIAM, TOE_DIAM, width*0.99);
	RobotRightToe.setPosition(offX + 0.5*(length - TOE_DIAM), -offY, FOOT_THICK);
	RobotRightToe.setOrientation(M_SQRT1_2, M_SQRT1_2, 0.0, 0.0);

}

// Update function for markers
void WAKBagMarkerMan::updateMarkers()
{
	// Add all the required markers to publish
	if(willPublish())
	{
		RobotTrunk.updateAdd();
		RobotLeftFoot.updateAdd();
		RobotRightFoot.updateAdd();
		RobotLeftToe.updateAdd();
		RobotRightToe.updateAdd();
	}
}
// EOF