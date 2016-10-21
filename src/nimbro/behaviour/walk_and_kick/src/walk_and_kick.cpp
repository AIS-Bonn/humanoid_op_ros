// Walk and kick: Main walk and kick class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/walk_and_kick.h>
#include <walk_and_kick/wak_vis.h>
#include <rc_utils/math_vec_mat.h>

// Defines
#define EXTRA_TEAMPACKET_COUNT  12

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// WalkAndKick class
//

// Constructor
WalkAndKick::WalkAndKick()
 : config()
 , RI(this)
 , SV(config, RI)
 , GM(config, SV, RI)
 , BM(config, SV, RI, GM.getWGS())
 , m_wakCycle(0)
 , m_teamPacketCount(0)
 , m_lastBlinkLED(false)
{
	// Reset the class
	reset();
}

// Reset function
void WalkAndKick::reset()
{
	// Reset the ROS interface
	RI.resetData();

	// Reset the finite state machine managers
	GM.reset();
	BM.reset();

	// Reset the cycle number
	m_wakCycle = 0;

	// Reset the team communications
	m_teamPacket.packetID = 0;
	m_teamPacket.timestamp.fromNSec(0);
	m_teamPacketCount = 0;
}

// Reset all function (cleans up and publishes everything that is required)
void WalkAndKick::resetAll()
{
	// Reset the walk and kick class
	reset();

	// Make totally certain that the button mode has been reset to stop (critical for the final published states below)
	RI.button = BTN_STOP;

	// Publish clean up data
	RI.writeNeutral();
	RI.clearLEDState();
	RI.clearRGBLED();
	RI.publishBehaviourState(GM.stoppedState(), BM.stoppedState());
}

// Initialisation function
bool WalkAndKick::init()
{
	// Return successful initialisation
	return true;
}

// Cycle step function
void WalkAndKick::step()
{
	// Increment the cycle counter
	m_wakCycle++;

	// Update the plot manager
	ros::Time now = ros::Time::now();
	ros::Time last = RI.PM->getTimestamp();
	double dT = (last.isZero() ? TINC : (now - last).toSec());
	RI.PM->clear(now);

	// Update the visualisation markers
	RI.MM->setEnabled(config.publishVis());
	RI.MMB->setEnabled(config.publishVis());
	RI.MM->clear(now);
	RI.MMB->clear(now);

	// Update the ROS interface
	bool LED4 = (BM.ballAction() != BA_KICK);
	RI.update(now, LED4);

	// Update the sensor variables
	SV.update(now);

	// Update and execute the game finite state machine
	GM.updateManager(m_wakCycle);
	GM.execute();

	// Update and execute the behaviour finite state machine
	BM.updateManager(GM.GV(), m_wakCycle);
	BM.execute();

	// Write the required outputs to the actuators
	RI.writeActuators(BM.AV());

	// Publish the team communications
	publishTeamComms(now);

	// Plotting
	if(config.plotData())
	{
		RI.PM->plotScalar(dT, PM_WAK_TRUEDT);
		RI.PM->plotScalar((m_wakCycle % PMSCALE_CYCLE) * (1.0/PMSCALE_CYCLE), PM_WAK_CYCLE);
		RI.PM->publish();
	}

	// Visualisation markers
	RI.MM->publish();
	RI.MMB->updateMarkers();
	RI.MMB->publish();
}

// Publish state function
void WalkAndKick::publishState()
{
	// Publish the behaviour state
	RI.publishBehaviourState(GM.currentState(), BM.currentState());
}

// Publish team communications function
void WalkAndKick::publishTeamComms(const ros::Time& now)
{
	// Throttle the team communications packets
	if((now - m_teamPacket.timestamp).toSec() < 0.1) return;

	// Decide whether we wish to be sending team communications packets
	bool sendTeamComms = (config.tcEnable() && SV.gameCommand != CMD_STOP && !RI.stateRelaxed() && RI.robotcontrolCommsOk && BM.currentState()->id() != WAKBehManager::BS_PANIC_ATTACK);

	// Send a few extra team packets after we shouldn't be anymore so that recipients have a good change of noticing the change
	if(sendTeamComms)
		m_teamPacketCount = EXTRA_TEAMPACKET_COUNT;
	else
	{
		if(m_teamPacketCount > 0)
			m_teamPacketCount--;
		else return;
	}

	// Calculate the pose of the closest obstacle
	Vec2f obstClosestPose = SV.robotPose2D + eigenRotatedCCW(SV.obstClosest.vec, SV.robotPose.z());
	float obstClosestPoseConf = SV.obstClosest.conf;
	bool obstClosestPoseValid = (SV.obstClosest.valid() && SV.haveRobotPose);

	// Calculate the walking target pose
	Vec2f walkingTargetPose = SV.robotPose2D + eigenRotatedCCW(BM.getWalkingTarget(), SV.robotPose.z());
	bool walkingTargetPoseValid = (BM.haveWalkingTarget() && SV.haveRobotPose);

	// Update the packet information
	m_teamPacket.packetID++;
	m_teamPacket.timestamp = now;
	m_teamPacket.robotNumber = config.robotNumber();
	m_teamPacket.silencing = !sendTeamComms;

	// Update the configuration information
	m_teamPacket.fieldType = field.fieldType();
	m_teamPacket.playOnYellow = SV.playOnYellow;
	m_teamPacket.playAsCyan = SV.playAsCyan;
	m_teamPacket.listenToGC = SV.listenToGC;
	m_teamPacket.listenToTC = SV.listenToTC;
	m_teamPacket.isPenaltyShoot = SV.isPenaltyShoot;
	m_teamPacket.kickoffType = SV.kickoffType;

	// Update the play state information
	m_teamPacket.buttonState = RI.button;
	m_teamPacket.gameCommand = SV.gameCommand;
	m_teamPacket.gameRole = SV.gameRole;
	m_teamPacket.playState = SV.playState;
	m_teamPacket.gameState = GM.currentState()->id();
	m_teamPacket.behState = BM.currentState()->id();
	m_teamPacket.fallen = (RI.stateFallen() || RI.stateIniting());

	// Update the game state information
	m_teamPacket.compassHeading = SV.compassHeading;
	m_teamPacket.robotPoseX = SV.robotPose.x();
	m_teamPacket.robotPoseY = SV.robotPose.y();
	m_teamPacket.robotPoseT = SV.robotPose.z();
	m_teamPacket.robotPoseConf = SV.robotPoseConf;
	m_teamPacket.robotPoseValid = SV.haveRobotPose;
	m_teamPacket.ballPoseX = SV.ballPose.x();
	m_teamPacket.ballPoseY = SV.ballPose.y();
	m_teamPacket.ballPoseConf = SV.ballConf;
	m_teamPacket.ballPoseValid = SV.haveBallPose;
	m_teamPacket.ballPoseStable = SV.ballPoseStable;
	m_teamPacket.obstClosestPoseX = obstClosestPose.x();
	m_teamPacket.obstClosestPoseY = obstClosestPose.y();
	m_teamPacket.obstClosestPoseConf = obstClosestPoseConf;
	m_teamPacket.obstClosestPoseValid = obstClosestPoseValid;
	m_teamPacket.walkingTargetPoseX = walkingTargetPose.x();
	m_teamPacket.walkingTargetPoseY = walkingTargetPose.y();
	m_teamPacket.walkingTargetPoseValid = walkingTargetPoseValid;

	// Update the status information
	m_teamPacket.timeSinceGCBase = (SV.GC.stampBase.isZero() ? INFINITY : (now - SV.GC.stampBase).toSec());
	m_teamPacket.timeSinceGCExtra = (SV.GC.stampExtra.isZero() ? INFINITY : (now - SV.GC.stampExtra).toSec());
	m_teamPacket.timeSinceTC = SV.TC.timeSinceData;
	m_teamPacket.numFreshTC = SV.TC.robotDataMap.size();

	// Publish the required team communications packet
	RI.publishTeamCommsPacket(m_teamPacket);
}

// Handle a call to the visualise clear service
bool WalkAndKick::handleVisualiseClear(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	// Constants
	static const std::string markerNamespace = "VisualiseClear";

	// Clear any previously published markers if required
	visualization_msgs::MarkerArray vis;
	vis_utils::GenMarker clear(RI.MM, RI.MM->alloFrame, markerNamespace, true);
	clear.setAction(3);
	vis.markers.push_back(clear.marker);
	RI.MM->publishMarkerArray(vis);

	// Return that the service was successfully handled
	return true;
	
}

// Handle a call to the visualise default ball handling service
bool WalkAndKick::handleVisualiseDBH(walk_and_kick::VisualiseDBHRequest& req, walk_and_kick::VisualiseDBHResponse& resp)
{
	// Constants
	static const std::string markerNamespace = "VisualiseDBH";

	// Clear any previously published markers if required
	if(req.type == 0)
	{
		visualization_msgs::MarkerArray vis;
		vis_utils::GenMarker clear(RI.MM, RI.MM->alloFrame, markerNamespace, true);
		clear.setAction(3);
		vis.markers.push_back(clear.marker);
		RI.MM->publishMarkerArray(vis);
		return true;
	}

	// Do not do anything unless the behaviours are stopped
	if(RI.button != BTN_STOP)
	{
		ROS_ERROR("Ignoring visualise DBH service call as behaviours are not stopped!");
		return false;
	}

	// Do not do anything unless the type parameter is ok
	if(req.type < 1 || req.type > 2)
	{
		ROS_ERROR("Unknown type specification %d in call to visualise DBH service!", req.type);
		return false;
	}

	// Do not do anything if the num parameters are invalid
	if(req.ballXNum <= 0 || req.ballYNum <= 0)
	{
		ROS_ERROR("One or both of the ballXNum/ballYNum specifications are zero!");
		return false;
	}

	// Save the relevant sensor variable states
	Vec3f origRobotPose = SV.robotPose;
	float origRobotPoseConf = SV.robotPoseConf;
	float origCompassHeading = SV.compassHeading;
	int origGoalSign = SV.goalSign;

	// Fake the required sensor variables data
	SV.robotPose.setZero();
	SV.robotPoseConf = 1.0f;
	SV.compassHeading = picut(req.compassHeading);
	SV.goalSign = (req.playOnYellow ? +1 : -1);

	// Calculate suitable marker sizes
	double asize = 0.8*std::min(fabs(req.ballXMax - req.ballXMin)/req.ballXNum, fabs(req.ballYMax - req.ballYMin)/req.ballYNum);
	double bsize = 0.1*asize;
	double ssize = 0.35*asize;
	double zval = 0.6*ssize;

	// Construct the required visualisation markers
	vis_utils::GenMarker targetDirn(RI.MM, RI.MM->alloFrame, markerNamespace, true);
	vis_utils::GenMarker footSelDot(RI.MM, RI.MM->alloFrame, markerNamespace, true);

	// Configure the required visualisation markers
	std::size_t N = (req.ballXNum + 1)*(req.ballYNum + 1);
	targetDirn.setType(visualization_msgs::Marker::TRIANGLE_LIST);
	targetDirn.setNumPoints(3*N);
	targetDirn.setNumPtColors(3*N);
	footSelDot.setType(visualization_msgs::Marker::SPHERE_LIST);
	footSelDot.setScale(ssize);

	// Initialise data structs
	geometry_msgs::Point dot;
	std_msgs::ColorRGBA color;
	dot.z = zval;
	color.a = 1.0f;

	// Calculate and visualise the required ball targets
	for(unsigned int i = 0; i <= req.ballXNum; i++)
	{
		float ballX = (req.ballXMin*(req.ballXNum - i) + req.ballXMax*i) / req.ballXNum;
		for(unsigned int j = 0; j <= req.ballYNum; j++)
		{
			float ballY = (req.ballYMin*(req.ballYNum - j) + req.ballYMax*j) / req.ballYNum;
			Vec2f ball(ballX, ballY);
			GameVars GV;
			if(req.type == 1)
				GM.DefaultBallHandling->calcPoseBallTarget(GV, ball, SV.goalSign);
			else if(req.type == 2)
				GM.DefaultBallHandling->calcCompassBallTarget(GV, ball, SV.goalSign);
			Vec2f unitDirn = eigenNormalized<float, 2>(GV.ballTargetDir - ball);
			Vec2f unitPerp = eigenRotatedCCW90(unitDirn);
			std::size_t index = 3*(i*(req.ballYNum + 1) + j);
			targetDirn.setPoint(index + 0, ball.x() - bsize*unitPerp.x(), ball.y() - bsize*unitPerp.y(), zval);
			targetDirn.setPoint(index + 1, ball.x() + asize*unitDirn.x(), ball.y() + asize*unitDirn.y(), zval);
			targetDirn.setPoint(index + 2, ball.x() + bsize*unitPerp.x(), ball.y() + bsize*unitPerp.y(), zval);
			if(GV.kickIfPossible) { color.r = 0.7f; color.g = 0.0f; color.b = 0.0f; }
			else { color.r = 0.0f; color.g = 0.0f; color.b = 1.0f; }
			targetDirn.setPtColor(index + 0, color.r, color.g, color.b);
			targetDirn.setPtColor(index + 1, color.r, color.g, color.b);
			targetDirn.setPtColor(index + 2, color.r, color.g, color.b);
			if(!GV.noSuggestedFoot())
			{
				dot.x = ball.x(); dot.y = ball.y();
				footSelDot.marker.points.push_back(dot);
				if(GV.suggestLeftFoot()) { color.r = 0.0f; color.g = 1.0f; color.b = 1.0f; }
				else { color.r = 0.8f; color.g = 0.6f; color.b = 0.0f; }
				footSelDot.marker.colors.push_back(color);
			}
		}
	}

	// Combine and publish the constructed visualisation markers
	visualization_msgs::MarkerArray vis;
	vis.markers.push_back(targetDirn.marker);
	vis.markers.push_back(footSelDot.marker);
	ros::Time now = ros::Time::now();
	for(size_t i = 0; i < vis.markers.size(); ++i)
	{
		vis.markers[i].header.stamp = now;
		vis.markers[i].id = i;
		vis.markers[i].lifetime = ros::Duration();
	}
	RI.MM->publishMarkerArray(vis);

	// Restore the relevant original sensor variable states
	SV.robotPose = origRobotPose;
	SV.robotPoseConf = origRobotPoseConf;
	SV.compassHeading = origCompassHeading;
	SV.goalSign = origGoalSign;

	// Return that the service was handled
	return true;
}

// Helper function for the visualise gcv XY service
void calcGcvXY(WAKBehShared& WBS, vis_utils::GenMarker& marker, float x, float y, float maxGcvX, float maxGcvY)
{
	// Construct the required gcv XY visualisation
	int Nm = ((int) marker.marker.points.size()) - 1;
	for(int i = 0; i <= Nm; i++)
	{
		float theta = M_2PI*(((float) i) / Nm);
		Vec2f vel = WBS.calcGcvXY(maxGcvX, maxGcvY, theta);
		marker.setPoint(i, vel.x(), vel.y(), 0.0);
	}
	marker.setPosition(x, y, 0.02);
}

// Handle a call to the visualise dribble approach service
bool WalkAndKick::handleVisualiseDbApp(VisualiseDbAppRequest& req, VisualiseDbAppResponse& resp)
{
	// Constants
	static const std::string markerNamespace = "VisualiseDbApp";

	// Do not do anything unless the behaviours are stopped
	if(RI.button != BTN_STOP)
	{
		ROS_ERROR("Ignoring visualise dribble approach service call as behaviours are not stopped!");
		return false;
	}

	// Input variables with a bit of range checking
	int XNum = std::max<int>(req.numX, 5);
	int YNum = std::max<int>(req.numY, 5);
	int YNumStep = 2*YNum + 1;
	std::size_t N = (XNum + 1)*YNumStep;
	float grid = std::max(req.gridSize, 0.005f);
	float robotAngle = picut(req.robotAngle);
	bool useRightFoot = req.useRightFoot;

	// Calculate suitable marker sizes
	float r = config.dbAppFunnelCurveRadius(), n = config.dbAppFunnelNeckMag(), o = config.dbAppPathTargetLineOffsetX(), e = config.dbAppFunnelEdgeExtra();
	double bound = std::max<double>(YNum*grid, n+r);
	double asize = 0.8*grid;
	double bsize = 0.1*asize;
	double ssize = 0.35*asize;
	double zval = 0.6*ssize;
	double zvals = 0.55*ssize;
	double zzval = 0.5*ssize;

	// Get the required ball offset
	Vec2f reqBallDir = (useRightFoot ? BM.WBS.reqBallDirRightDb : BM.WBS.reqBallDirLeftDb);

	// Construct the required visualisation markers
	visualization_msgs::MarkerArray vis;
	vis_utils::GenMarker pathAngleArrow(RI.MM, RI.MM->alloFrame, markerNamespace, true);
	pathAngleArrow.setType(visualization_msgs::Marker::TRIANGLE_LIST);
	pathAngleArrow.setNumPoints(3*N);
	pathAngleArrow.setNumPtColors(3*N);
	vis_utils::GenMarker walkAngleArrow(RI.MM, RI.MM->alloFrame, markerNamespace, true);
	walkAngleArrow.setType(visualization_msgs::Marker::TRIANGLE_LIST);
	walkAngleArrow.setNumPoints(3*N);
	walkAngleArrow.setNumPtColors(3*N);
	vis_utils::ArrowMarker robotAngleArrow(RI.MM, RI.MM->alloFrame, 0.008, 0.016, 0.0, markerNamespace, true);
	robotAngleArrow.setPosition(-reqBallDir.x() - o, -reqBallDir.y(), zval);
	robotAngleArrow.setPoint(0, 0.0, 0.0, 0.0);
	robotAngleArrow.setPoint(1, 0.17*cos(robotAngle), 0.17*sin(robotAngle), 0.0);
	robotAngleArrow.setColor(0.5, 0.5, 0.5);
	vis_utils::SphereMarker ball(RI.MM, RI.MM->alloFrame, field.ballDiameter(), markerNamespace, true);
	ball.setPosition(0.0, 0.0, field.ballRadius());
	ball.setColor(0.88235, 0.19608, 0.0);
	vis_utils::SphereMarker ballTarget(RI.MM, RI.MM->alloFrame, field.ballDiameter(), markerNamespace, true);
	ballTarget.setPosition(1.0, 0.0, field.ballRadius());
	ballTarget.setColor(0.5, 0.11, 0.0);
	vis_utils::GenMarker ballToTargetLine(RI.MM, RI.MM->alloFrame, markerNamespace, true);
	ballToTargetLine.setType(visualization_msgs::Marker::LINE_STRIP);
	ballToTargetLine.setScale(0.015);
	ballToTargetLine.setColor(0.5, 0.3, 0.2);
	ballToTargetLine.setNumPoints(2);
	ballToTargetLine.setPosition(0.0, 0.0, field.ballRadius());
	ballToTargetLine.setPoint(0, 0.0, 0.0);
	ballToTargetLine.setPoint(1, 1.0, 0.0);
	vis_utils::SphereMarker behindBallDot(RI.MM, RI.MM->alloFrame, 0.03, markerNamespace, true);
	behindBallDot.setPosition(-reqBallDir.x(), -reqBallDir.y(), zval);
	behindBallDot.setColor(0.8, 0.6, 0.0);
	vis_utils::GenMarker pathTargetLine(RI.MM, RI.MM->alloFrame, markerNamespace, true);
	pathTargetLine.setType(visualization_msgs::Marker::LINE_LIST);
	pathTargetLine.setScale(0.005);
	pathTargetLine.setColor(0.4, 1.0, 0.4);
	pathTargetLine.setNumPoints(4);
	pathTargetLine.setPoint(0, -reqBallDir.x() - o, bound - reqBallDir.y(), zzval);
	pathTargetLine.setPoint(1, -reqBallDir.x() - o, -bound - reqBallDir.y(), zzval);
	pathTargetLine.setPoint(2, -XNum*grid, -reqBallDir.y(), zzval);
	pathTargetLine.setPoint(3, 0.0, -reqBallDir.y(), zzval);
	vis_utils::GenMarker funnel(RI.MM, RI.MM->alloFrame, markerNamespace, true);
	vis_utils::GenMarker funnelExtra(RI.MM, RI.MM->alloFrame, markerNamespace, true);
	funnel.setType(visualization_msgs::Marker::LINE_STRIP);
	funnelExtra.setType(visualization_msgs::Marker::LINE_STRIP);
	funnel.setScale(0.005);
	funnelExtra.setScale(0.005);
	funnel.setColor(0.8, 0.6, 0.0);
	funnelExtra.setColor(1.0, 0.5, 0.0);
	const int funnelN = 30;
	funnel.setNumPoints(2*(funnelN + 1) + 4);
	funnelExtra.setNumPoints(2*(funnelN + 1) + 4);
	int i = 0, j = 0;
	funnel.setPosition(-reqBallDir.x() - o, -reqBallDir.y(), zzval);
	funnelExtra.setPosition(-reqBallDir.x() - o, -reqBallDir.y(), zzval);
	funnel.setPoint(i++, -r, bound);
	funnelExtra.setPoint(j++, -r + e, bound);
	for(int k = 0; k <= funnelN; k++)
	{
		double theta = k * M_PI_2 / funnelN;
		funnel.setPoint(i++, -r*cos(theta), n + r - r*sin(theta));
		funnelExtra.setPoint(j++, -(r-e)*cos(theta), n + r - (r-e)*sin(theta));
	}
	funnel.setPoint(i++, o + reqBallDir.x(), n);
	funnelExtra.setPoint(j++, o + reqBallDir.x(), n + e);
	funnel.setPoint(i++, o + reqBallDir.x(), -n);
	funnelExtra.setPoint(j++, o + reqBallDir.x(), -n - e);
	for(int k = 0; k <= funnelN; k++)
	{
		double theta = k * M_PI_2 / funnelN;
		funnel.setPoint(i++, -r*sin(theta), r*cos(theta) - n - r);
		funnelExtra.setPoint(j++, -(r-e)*sin(theta), (r-e)*cos(theta) - n - r);
	}
	funnel.setPoint(i++, -r, -bound);
	funnelExtra.setPoint(j++, -r + e, -bound);

	// Initialise data structs
	std_msgs::ColorRGBA color;
	color.a = 1.0f;

	// Calculate and visualise the required ball targets
	Vec2f robotE, robotToBehindBallE, robotToPathTargetE;
	for(int i = 0; i <= XNum; i++)
	{
		robotE.x() = -i*grid;
		for(int j = -YNum; j <= YNum; j++)
		{
			robotE.y() = j*grid - reqBallDir.y();
			float pathAngle = BM.DribbleBall->calcPathAngle(useRightFoot, robotE, robotToBehindBallE, robotToPathTargetE);
			float localPathAngle = picut(pathAngle - robotAngle);
			float u = BM.DribbleBall->calcPathInterpFactor(robotToBehindBallE, localPathAngle);
			float walkAngle = BM.DribbleBall->calcWalkAngle(robotToBehindBallE, u, localPathAngle, pathAngle);
			if(!std::isfinite(pathAngle) || !std::isfinite(u) || !std::isfinite(walkAngle))
				ROS_WARN("Encountered non-finite values while testing the dribble approach!");
			std::size_t index = 3*(i*YNumStep + j + YNum);
			Vec2f unitDirn(cos(pathAngle), sin(pathAngle));
			Vec2f unitPerp = eigenRotatedCCW90(unitDirn);
			pathAngleArrow.setPoint(index + 0, robotE.x() - bsize*unitPerp.x(), robotE.y() - bsize*unitPerp.y(), zval);
			pathAngleArrow.setPoint(index + 1, robotE.x() + asize*unitDirn.x(), robotE.y() + asize*unitDirn.y(), zval);
			pathAngleArrow.setPoint(index + 2, robotE.x() + bsize*unitPerp.x(), robotE.y() + bsize*unitPerp.y(), zval);
			unitDirn << cos(robotAngle + walkAngle), sin(robotAngle + walkAngle);
			unitPerp = eigenRotatedCCW90(unitDirn);
			walkAngleArrow.setPoint(index + 0, robotE.x() - bsize*unitPerp.x(), robotE.y() - bsize*unitPerp.y(), zvals);
			walkAngleArrow.setPoint(index + 1, robotE.x() + asize*unitDirn.x(), robotE.y() + asize*unitDirn.y(), zvals);
			walkAngleArrow.setPoint(index + 2, robotE.x() + bsize*unitPerp.x(), robotE.y() + bsize*unitPerp.y(), zvals);
			color.r = 0.7f*u; color.g = 0.0f; color.b = 1.0f - u;
			pathAngleArrow.setPtColor(index + 0, color.r, color.g, color.b);
			pathAngleArrow.setPtColor(index + 1, color.r, color.g, color.b);
			pathAngleArrow.setPtColor(index + 2, color.r, color.g, color.b);
			color.r = 0.7f*u; color.g = 0.5f; color.b = 1.0f - u;
			walkAngleArrow.setPtColor(index + 0, color.r, color.g, color.b);
			walkAngleArrow.setPtColor(index + 1, color.r, color.g, color.b);
			walkAngleArrow.setPtColor(index + 2, color.r, color.g, color.b);
		}
	}

	// Combine and publish the constructed visualisation markers
	vis.markers.push_back(pathAngleArrow.marker);
	vis.markers.push_back(walkAngleArrow.marker);
	vis.markers.push_back(robotAngleArrow.marker);
	vis.markers.push_back(ball.marker);
	vis.markers.push_back(ballTarget.marker);
	vis.markers.push_back(ballToTargetLine.marker);
	vis.markers.push_back(behindBallDot.marker);
	vis.markers.push_back(pathTargetLine.marker);
	vis.markers.push_back(funnel.marker);
	vis.markers.push_back(funnelExtra.marker);
	ros::Time now = ros::Time::now();
	for(size_t i = 0; i < vis.markers.size(); ++i)
	{
		vis.markers[i].header.stamp = now;
		vis.markers[i].id = i;
		vis.markers[i].lifetime = ros::Duration();
	}
	RI.MM->publishMarkerArray(vis);

	// Return that the service was handled
	return true;
}

// Handle a call to the visualise gcv XY service
bool WalkAndKick::handleVisualiseGcvXY(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	// Constants
	static const std::string markerNamespace = "VisualiseGcvXY";
	static const int N = 101;

	// Do not do anything unless the behaviours are stopped
	if(RI.button != BTN_STOP)
	{
		ROS_ERROR("Ignoring visualise gcv XY service call as behaviours are not stopped!");
		return false;
	}

	// Declare the marker array
	visualization_msgs::MarkerArray vis;

	// Construct the required visualisation markers
	vis_utils::GenMarker arrows(RI.MM, RI.MM->alloFrame, markerNamespace, true);
	arrows.setType(visualization_msgs::Marker::LINE_STRIP);
	arrows.setNumPoints(N);
	arrows.setScale(0.02);
	arrows.setColor(1.0, 0.3, 0.3);

	// Plot some gcv XY loci
	calcGcvXY(BM.WBS, arrows, 0.0f, 0.0f, 1.0f, 1.0f);
	vis.markers.push_back(arrows.marker);
	calcGcvXY(BM.WBS, arrows, 0.0f, 2.0f, 2.0f, 0.5f);
	vis.markers.push_back(arrows.marker);
	calcGcvXY(BM.WBS, arrows, -2.0f, 0.0f, 0.7f, 1.4f);
	vis.markers.push_back(arrows.marker);

	// Combine and publish the constructed visualisation markers
	ros::Time now = ros::Time::now();
	for(size_t i = 0; i < vis.markers.size(); ++i)
	{
		vis.markers[i].header.stamp = now;
		vis.markers[i].id = i;
		vis.markers[i].lifetime = ros::Duration();
	}
	RI.MM->publishMarkerArray(vis);

	// Return that the service was handled
	return true;
}
// EOF