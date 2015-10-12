// behaviour_exercise.cpp - Philipp Allgeuer - 12/07/13
// Source file for the behaviours exercise

// Includes
#include "behaviour_exercise/behaviour_exercise_sample.h" // Note: This has to be switched between 'sample' and 'template' to choose which version to compile (also needs to change in the CMakeLists file)
#include <gait/gait_common.h>

// Defines
#define SRV_CALL_DELAY  0.4

// Namespaces
using namespace std;
using namespace behaviourexercise;

//
// RobotSC class
//

// Constructor
RobotSC::RobotSC()
 : StateController("RobotSC")
 , CONFIG_PARAM_PATH("/behaviour_exercise/")
 , m_be_run(CONFIG_PARAM_PATH + "run", false)
 , m_be_use_obstacle(CONFIG_PARAM_PATH + "use_obstacle", false)
 , m_be_place_robot(CONFIG_PARAM_PATH + "on_run/place_robot", true)
 , m_be_rand_robot(CONFIG_PARAM_PATH + "on_run/random_robot", false)
 , m_be_rand_ball(CONFIG_PARAM_PATH + "on_run/random_ball", false)
 , m_be_rand_goal(CONFIG_PARAM_PATH + "on_run/random_goal", false)
 , m_be_rand_obst(CONFIG_PARAM_PATH + "on_run/random_obstacle", false)
 , m_be_imperfect_meas(CONFIG_PARAM_PATH + "use_imperfect_measurements", false)
 , m_field(field_model::FieldModel::getInstance())
 , m_robotState("relaxed")
 , m_usingPositiveGoal(true)
{
	// Create ROS node handle
	ros::NodeHandle nh("~");

	// Subscribe to the required ROS topics
	m_sub_robotState = nh.subscribe("/robotcontrol/state", 1, &RobotSC::handleRobotState, this);
	
	// Advertise the required ROS topics
	m_pub_plot = nh.advertise<plot_msgs::Plot>("/plot", 1);
	m_pub_gaitCommand = nh.advertise<gait_msgs::GaitCommand>("/gaitCommand", 1);
	m_pub_object_markers = nh.advertise<visualization_msgs::MarkerArray>("visualization/object_markers", 1);

	// Subscribe to the required ROS services
	m_srv_setOdom = nh.serviceClient<gait_msgs::SetOdom>("/gait/setOdom");
	m_srv_setParam = nh.serviceClient<config_server::SetParameter>("/config_server/set_parameter");
	m_srv_playMotion = nh.serviceClient<motion_player::PlayMotion>("/motion_player/play");

	// Set configuration parameters
	if(!setParam("run", "0")) ROS_WARN("Failed to set config server parameter 'run'!");
	if(!setParam("/indep_cpg_gait/odometry/use_simulation_odometry", "1")) ROS_WARN("Failed to set config server parameter '/indep_cpg_gait/odometry/use_simulation_odometry'!");

	// Seed the random number generator
	srand48((long int) ros::Time::now().nsec);

	// Initialise the field
	initField();

	// Call a user callback
	onConstruct();
}

// Initialisation function
void RobotSC::initField()
{
	// Reset the detection vectors
	m_ballVec.reset();
	m_goalVec.reset();
	m_obstVec.reset();

	// Choose a goal to play on and place the robot, ball and obstacle
	placeGoal();
	placeRobot();
	placeBall();
	placeObst();
}

// ROS topic handlers
void RobotSC::handleRobotState(const robotcontrol::StateConstPtr& msg)
{
	// Save the received data value
	m_robotState = msg->label; // Can be (for example): "standing", "walking", "kicking",...
}

// ROS interface functions
bool RobotSC::setOdom(double x, double y)
{
	// Declare variables
	bool ret = false;

	// Decide whether to call the service or not (don't want to spam the service by calling it every time!)
	if(m_lastOdomCall.hasElapsed(SRV_CALL_DELAY))
	{
		gait_msgs::SetOdom pos;
		pos.request.posX = x;
		pos.request.posY = y;
		pos.request.rotZ = 0.0;
		ret = m_srv_setOdom.call(pos);
		m_lastOdomCall.setMarker();
		if(ret) m_robotPos << x, y;
	}

	// Return whether the service call was carried out AND was successful (or not)
	return ret;
}
bool RobotSC::setParam(const std::string& name, const std::string& value) // Note: At least one call to ros::spinOnce() is required before the change is seen when retrieving the parameter!
{
	// Construct the required message
	config_server::SetParameter msg;
	if(!name.empty() && (name.at(0) == '/'))
		msg.request.name = name;
	else
		msg.request.name = "/behaviour_exercise/" + name;
	msg.request.no_notify = "0";
	msg.request.value = value;

	// Call the service
	return m_srv_setParam.call(msg);
}
bool RobotSC::playMotion(KeyMotionEnum motionID)
{
	// Declare variables
	bool ret = false;

	// Decide whether to call the service or not (don't want to spam the service by calling it every time!)
	if(m_lastPlayCall.hasElapsed(SRV_CALL_DELAY) && (motionID > KM_NO_MOTION) && (motionID < KM_NUM_MOTIONS))
	{
		motion_player::PlayMotion motion;
		motion.request.name = KeyMotionName[motionID];
		ret = m_srv_playMotion.call(motion);
		m_lastPlayCall.setMarker();
	}

	// Return whether the service call was carried out AND was successful (or not)
	return ret;
}

// State controller callbacks
bool RobotSC::preStepCallback()
{
	// Reset list of points to plot
	m_plot.points.clear();

	// Get latest robot pose from motion model
	bool valid = updateRobotPos();
	
	// Recalculate detection vectors
	if(valid)
	{
		updateBallVector();
		updateGoalVector();
		updateObstVector();
	}

	// Plot updated data
	plotScalar(robotIsStanding(), "RobotIs/Standing");
	plotScalar(robotIsWalking(), "RobotIs/Walking");
	plotScalar(robotIsKicking(), "RobotIs/Kicking");
	plotVector(m_robotPos, "RobotPosition");
	plotScalar(m_robotOrient, "RobotOrientation");
	plotVector(m_ballVec.vec, "BallVector");
	plotVector(m_goalVec.vec, "GoalVector");
	plotVector(m_obstVec.vec, "ObstacleVector");

	// Call user preStepCallback
	bool wantTransition = preStepCallbackUser();

	// Return whether we wish to force a state transition
	return wantTransition;
}
void RobotSC::preExecuteCallback()
{
	// Plot the current state
	plotScalar(getCurState()->id, "CurrentState");
}
void RobotSC::postStepCallback()
{
	// Call user postStepCallback
	postStepCallbackUser();

	// Plot the gait command vector
	Eigen::Vector3d gaitVec(m_lastGaitCmd.gcvX, m_lastGaitCmd.gcvY, m_lastGaitCmd.gcvZ);
	plotVector(gaitVec, "GaitCmdVector");
	plotScalar(m_lastGaitCmd.walk, "GaitCmdWalk");

	// Publish the latest marker positions
	publishMarkers();

	// Publish the latest plotter values
	publishPlotter();
}

// Data retrieval members
bool RobotSC::getRobotTransform(tf::StampedTransform& transform, bool suppressWarn) const
{
	// Try to get the latest transform of the robot
	try
	{
		m_tflistener.lookupTransform(gait::gaitOdomFrame, "/ego_floor", ros::Time(0), transform);
	}
	catch(tf::TransformException e)
	{
		if(!suppressWarn)
			ROS_WARN_STREAM_THROTTLE(0.4, "Could not look up transform: " << e.what());
		return false;
	}

	// Return success
	return true;
}

// Detection simulation functions
void RobotSC::calcRelativeVector(const Eigen::Vector2d& globalPos, Eigen::Vector2d& relativePos) const
{
	// Transform the global position to a vector in the robot's frame of reference (/ego_floor)
	Eigen::Vector2d ballRelPos = globalPos - m_robotPos;
	tf::Vector3 tfBallRelPos(ballRelPos.x(), ballRelPos.y(), 0.0);
	tf::Vector3 tfBallRobPos = m_robotTransformInv.getBasis() * tfBallRelPos;
	relativePos << tfBallRobPos.x(), tfBallRobPos.y();
}
void RobotSC::addDetectionNoise(Eigen::Vector2d& objVec, double magnitude) const
{
	// Add noise proportionally to how far away the object is
	if(m_be_imperfect_meas())
	{
		double newNorm = objVec.norm()*(1 + magnitude*(2*drand48()-1));
		double newAngle = atan2(objVec.y(), objVec.x()) + magnitude*(2*drand48()-1);
		objVec << newNorm*cos(newAngle), newNorm*sin(newAngle);
	}
}
bool RobotSC::isInView(const Eigen::Vector2d& objVec, double objAperture) const
{
	// Constants
	const double fov = 150*M_PI/180;

	// Calculate whether object is in field of view (adjusting for required aperture)
	return 2.0*fabs(atan2(objVec.y(), objVec.x())) + fabs(objAperture) <= fov;
}
bool RobotSC::isDetected(const Eigen::Vector2d& objVec, double minDist, double maxDist) const
{
	// Randomly do not detect the object proportionally to how far away it is
	if(m_be_imperfect_meas())
		return (drand48() < (objVec.norm() - maxDist)/(minDist - maxDist));
	else
		return true;
}

// Placement functions
void RobotSC::placeGoal()
{
	// Retrieve the goal locations
	std::vector<field_model::WorldObject> objects = m_field->objects(field_model::WorldObject::Type_Goal);

	// Choose which goal to aim at
	int goalSide = (m_be_rand_goal() ? ((drand48() < 0.5) ? 0 : 1) : 1);

	// Retrieve the position of the chosen goal
	Eigen::Vector3d goal = objects[goalSide].pose();
	m_goalPos << goal.x(), goal.y();
	m_usingPositiveGoal = (goalSide == 1);
}
void RobotSC::placeRobot()
{
	// Place the robot on the field (if placement is not disabled in config server)
	if(m_be_place_robot())
	{
		if(m_be_rand_robot())
		{
			// Constants
			const double minEdgeDist = 0.3;

			// Robot placement limits
			double xmin = -m_field->length()/4.0;
			double xmax = m_field->length()/2.0 - minEdgeDist;
			double ymin = -m_field->width()/2.0 + minEdgeDist;
			double ymax =  m_field->width()/2.0 - minEdgeDist;
			if(!m_usingPositiveGoal)
			{
				xmin = -xmin;
				xmax = -xmax;
			}

			// Place the robot in a random location on the field
			setOdom(xmin + (xmax - xmin)*drand48(), ymin + (ymax - ymin)*drand48());
		}
		else setOdom(0.0, 0.0);
		m_robotOrient = 0.0;
	}
}
void RobotSC::placeBall()
{
	// Constants
	const double minEdgeDist = 0.3;
	const double minRobotDist = 0.3;

	// Place the ball as required
	if(m_be_rand_ball())
	{
		// Ball placement limits
		double xmin = 0.0 + minEdgeDist;
		double xmax = m_field->length()/2.0 - minEdgeDist;
		double ymin = -m_field->width()/2.0 + minEdgeDist;
		double ymax =  m_field->width()/2.0 - minEdgeDist;
		if(!m_usingPositiveGoal)
		{
			xmin = -xmin;
			xmax = -xmax;
		}

		// Place the ball in a random location on the field
		for(int i = 0;i < 100;i++)
		{
			m_ballPos << xmin + (xmax - xmin)*drand48(), ymin + (ymax - ymin)*drand48();
			if((m_robotPos - m_ballPos).norm() > minRobotDist) break;
		}
	}
	else
		m_ballPos << (m_usingPositiveGoal ? m_field->centerCircleDiameter()/2.0 : -m_field->centerCircleDiameter()/2.0), 0.0;
}
void RobotSC::placeObst()
{
	// Place the obstacle as required
	if(m_be_rand_obst())
	{
		// Calculate unit vector pointing from robot to ball in field coordinates
		Eigen::Vector2d udirn = m_ballPos - m_robotPos;
		double ballDist = udirn.norm();
		udirn /= ballDist;

		// Constants
		const double minEdgeDist = 0.3;
		const double minBallDist = 0.6;
		const double umin = 0.6;
		const double umax = ballDist + 2.0*umin;
		const double vmax = 1.0;

		// Place the obstacle in a random location on the field
		double u, v;
		for(int i = 0;i < 100;i++)
		{
			u = umin + (umax - umin)*drand48();
			v = vmax*(2.0*drand48() - 1);
			m_obstPos << m_robotPos.x() + u*udirn.x() - v*udirn.y(), m_robotPos.y() + u*udirn.y() + v*udirn.x();
			if(((m_obstPos - m_ballPos).norm() > minBallDist) && (fabs(m_obstPos.x()) < m_field->length()/2.0 - minEdgeDist) && (fabs(m_obstPos.y()) < m_field->width()/2.0)) break;
		}
	}
	else
		m_obstPos << (m_usingPositiveGoal ? m_field->length()/2.0 - m_field->goalAreaDepth()/2.0 : m_field->goalAreaDepth()/2.0 - m_field->length()/2.0), 0.0;
}

// Update functions
bool RobotSC::updateRobotPos()
{
	// Retrieve the required transform
	if(!getRobotTransform(m_robotTransform, (getCycle() < 10))) return false;
	m_robotTransformInv = m_robotTransform.inverse();

	// Update the robot position
	m_robotPos << m_robotTransform.getOrigin().x(), m_robotTransform.getOrigin().y();
	tf::Vector3 xaxis = m_robotTransform.getBasis().getColumn(0);
	m_robotOrient = atan2(xaxis.y(), xaxis.x());

	// Return that m_robotPos, m_robotOrient, m_robotTransform and m_robotTransformInv are valid
	return true;
}
void RobotSC::updateBallVector()
{
	// Declare variables
	Eigen::Vector2d ballVec;
	bool canSee = true;
	
	// Calculate the true vector to the ball
	calcRelativeVector(m_ballPos, ballVec);

	// Don't report the ball randomly, or if it is outside our field of view
	canSee &= isInView(ballVec);
	canSee &= isDetected(ballVec, 2.0, 9.0);

	// Don't report the ball if the obstacle is obstructing it
	if(m_be_use_obstacle())
	{
		Eigen::Vector2d ballRelPos = m_ballPos - m_robotPos;
		Eigen::Vector2d obstRelPos = m_obstPos - m_robotPos;
		double obstacleAngle = atan2(obstRelPos.y(), obstRelPos.x()) - atan2(ballRelPos.y(), ballRelPos.x());
		double perpObstacleDist = obstRelPos.norm() * sin(obstacleAngle);
		double parallelObstacleDist = obstRelPos.norm() * cos(obstacleAngle);
		canSee &= !((fabs(perpObstacleDist) < 0.15) && (parallelObstacleDist >= 0.0) && (parallelObstacleDist <= ballRelPos.norm()));
	}

	// Add noise to the ball vector
	addDetectionNoise(ballVec, 0.05);

	// Report the ball vector if required
	if(canSee)
	{
		m_ballVec.vec = ballVec;
		m_ballVec.setStampNow();
	}

	// Plot whether the ball is seen or not
	plotScalar(canSee, "CanSee/Ball");
}
void RobotSC::updateGoalVector()
{
	// Declare variables
	Eigen::Vector2d goalVec;
	bool canSee = true;

	// Calculate the true vector to the goal
	calcRelativeVector(m_goalPos, goalVec);

	// Don't report the goal randomly, or if it is outside our field of view
	Eigen::Vector2d goalRelPos = m_goalPos - m_robotPos;
	if(goalRelPos.norm() > 0.0)
	{
		double goalAperture = 2.0*fabs(atan2(0.5*m_field->goalWidth()*fabs(goalRelPos.x()/goalRelPos.norm()), goalVec.norm()));
		canSee &= isInView(goalVec, goalAperture); // Slightly more restrictive as we need to see both goal posts
	}
	else canSee = false;
	canSee &= isDetected(goalVec, 2.0, 9.0);

	// Add noise to the goal vector
	addDetectionNoise(goalVec, 0.1);

	// Report the goal vector if required
	if(canSee)
	{
		m_goalVec.vec = goalVec;
		m_goalVec.setStampNow();
	}

	// Plot whether the goal is seen or not
	plotScalar(canSee, "CanSee/Goal");
}
void RobotSC::updateObstVector()
{
	// Declare variables
	Eigen::Vector2d obstVec;
	bool canSee = true;

	// Calculate the true vector to the obstacle
	calcRelativeVector(m_obstPos, obstVec);

	// Don't report the obstacle randomly, or if it is outside our field of view
	canSee &= isInView(obstVec);
	canSee &= isDetected(obstVec, 1.0, 5.0); // More restrictive in order to model obstacles coming into view

	// Add noise to the obstacle vector
	addDetectionNoise(obstVec, 0.1);

	// Report the obstacle vector if required
	if(canSee && m_be_use_obstacle())
	{
		m_obstVec.vec = obstVec;
		m_obstVec.setStampNow();
	}

	// Plot whether the obstacle is seen or not
	plotScalar(canSee && m_be_use_obstacle(), "CanSee/Obstacle");
}

// Visualisation functions
void RobotSC::publishMarkers()
{
	// Declare variables
	visualization_msgs::Marker marker;

	// Clear any existing markers in the marker array
	m_markerArray.markers.clear();

	// Ball marker
	marker.header.frame_id = gait::gaitOdomFrame;
	marker.header.stamp = ros::Time::now();
	marker.ns = "ball";
	marker.id = m_markerArray.markers.size();
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = m_ballPos.x();
	marker.pose.position.y = m_ballPos.y();
	marker.pose.position.z = 0.10;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.color.r = 0.88235f;
	marker.color.g = 0.19608f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration(0.2);
	m_markerArray.markers.push_back(marker);

	// Obstacle marker
	if(m_be_use_obstacle())
	{
		marker.header.frame_id = gait::gaitOdomFrame;
		marker.header.stamp = ros::Time::now();
		marker.ns = "obstacle";
		marker.id = m_markerArray.markers.size();
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = m_obstPos.x();
		marker.pose.position.y = m_obstPos.y();
		marker.pose.position.z = 0.50;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.35;
		marker.scale.y = 0.35;
		marker.scale.z = 1.0;
		marker.color.r = 0.1000f;
		marker.color.g = 0.1000f;
		marker.color.b = 0.1000f;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration(0.2);
		m_markerArray.markers.push_back(marker);
	}
	
	// Publish the required markers
	m_pub_object_markers.publish(m_markerArray);
}
void RobotSC::publishPlotter()
{
	// Write the required data points to the plotter ROS topic
	m_plot.header.stamp = ros::Time::now();
	m_pub_plot.publish(m_plot);
}

// Gait functions
void RobotSC::sendGaitCommand(gait_msgs::GaitCommand cmd)
{
	// Constants
	const double maxVx = 1.0;
	const double maxVy = 1.0;
	const double maxWz = 0.7;

	// Saturate the desired gait command
	double invScaleX = fabs(cmd.gcvX)/maxVx;
	double invScaleY = fabs(cmd.gcvY)/maxVy;
	double invScaleW = fabs(cmd.gcvZ)/maxWz;
	double scale = (invScaleX > invScaleY ? invScaleX : invScaleY);
	scale = (scale > invScaleW ? scale : invScaleW);
	if(scale > 1.0)
	{
		cmd.gcvX /= scale;
		cmd.gcvY /= scale;
		cmd.gcvZ /= scale;
	}

	// Publish the desired gait command
	m_lastGaitCmd = cmd;
	m_pub_gaitCommand.publish(cmd);
}
void RobotSC::sendGaitCommand(bool walk, double Vx, double Vy, double Wz)
{
	// Declare variables
	gait_msgs::GaitCommand cmd;

	// Construct the desired gait command
	cmd.walk = walk;
	cmd.gcvX = (walk ? Vx : 0.0);
	cmd.gcvY = (walk ? Vy : 0.0);
	cmd.gcvZ = (walk ? Wz : 0.0);

	// Publish the desired gait command
	sendGaitCommand(cmd);
}

// Plotting functions
void RobotSC::plotScalar(double value, const std::string& name)
{
	// Add the required plot point
	plot_msgs::PlotPoint point;
	point.name = "/Behaviour Exercise/" + name;
	point.value = value;
	m_plot.points.push_back(point);
}
void RobotSC::plotVector(const Eigen::Vector2d& value, const std::string& name)
{
	// Add the required x plot point
	plot_msgs::PlotPoint point;
	point.name = "/Behaviour Exercise/" + name + "/x";
	point.value = value.x();
	m_plot.points.push_back(point);

	// Add the required y plot point
	point.name = "/Behaviour Exercise/" + name + "/y";
	point.value = value.y();
	m_plot.points.push_back(point);
}
void RobotSC::plotVector(const Eigen::Vector3d& value, const std::string& name)
{
	// Add the required x plot point
	plot_msgs::PlotPoint point;
	point.name = "/Behaviour Exercise/" + name + "/x";
	point.value = value.x();
	m_plot.points.push_back(point);

	// Add the required y plot point
	point.name = "/Behaviour Exercise/" + name + "/y";
	point.value = value.y();
	m_plot.points.push_back(point);

	// Add the required z plot point
	point.name = "/Behaviour Exercise/" + name + "/z";
	point.value = value.z();
	m_plot.points.push_back(point);
}

//
// Main function
//
int main(int argc, char **argv)
{
	// Process ROS command line arguments
	ros::init(argc, argv, "behaviour_exercise");

	// Create an instance of the state controller and initialise it to the idle state
	ret_t ret;
	RobotSC rsc;
	if((ret = rsc.init(NewStateInstance<IdleState>(&rsc))) != SCR_OK)
	{
		ROS_FATAL_STREAM("Could not initialise the state controller " << rsc.name << " (return code " << ret << ")!");
		return 1;
	}

	// Set the loop rate (in Hz)
	ros::Rate rate(15);

	// Keep looping while everything is ok
	while(ros::ok())
	{
		// Execute all required ROS callbacks
		ros::spinOnce();

		// Perform a state controller step
		rsc.step();

		// Sleep for the required duration
		rate.sleep();
	}

	// Return value
	return 0;
}
// EOF