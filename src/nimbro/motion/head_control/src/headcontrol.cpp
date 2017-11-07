// Motion control for the robot head
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <head_control/headcontrol.h>
#include <pluginlib/class_list_macros.h>
#include <rc_utils/math_funcs.h>
#include <Eigen/Core>
#include <cmath>

// Defines
#define HEAD_PLOT_SCALE 2.0

// Namespaces
using namespace headcontrol;

//
// HeadControl class
//

// Constants
const std::string HeadControl::RESOURCE_PATH = "headcontrol/";
const std::string HeadControl::CONFIG_PARAM_PATH = "/headcontrol/";

// Constructor
HeadControl::HeadControl()
 : m_nhs()
 , m_lastActive(false)
 , m_headControlEnabled(CONFIG_PARAM_PATH + "headControlEnabled", true)
 , m_yawRelax(CONFIG_PARAM_PATH + "yaw/relax", false)
 , m_yawMaxVel(CONFIG_PARAM_PATH + "yaw/maxVel", 0.5, 0.25, 15.0, 3.0) // Units: rad/s
 , m_yawMaxAcc(CONFIG_PARAM_PATH + "yaw/maxAcc", 0.5, 0.25, 50.0, 10.0) // Units: rad/s^2
 , m_yawDefaultEffort(CONFIG_PARAM_PATH + "yaw/defaultEffort", 0.0, 0.01, 1.0, 0.25)
 , m_pitchRelax(CONFIG_PARAM_PATH + "pitch/relax", false)
 , m_pitchMaxVel(CONFIG_PARAM_PATH + "pitch/maxVel", 0.5, 0.25, 15.0, 3.0) // Units: rad/s
 , m_pitchMaxAcc(CONFIG_PARAM_PATH + "pitch/maxAcc", 0.5, 0.25, 50.0, 10.0) // Units: rad/s^2
 , m_pitchDefaultEffort(CONFIG_PARAM_PATH + "pitch/defaultEffort", 0.0, 0.01, 1.0, 0.25)
 , m_yawTarget(0.0)
 , m_yawCurX(0.0)
 , m_yawCurV(0.0)
 , m_yawCurEffort(0.0)
 , m_yawEffortTarget(0.0)
 , m_pitchTarget(0.0)
 , m_pitchCurX(0.0)
 , m_pitchCurV(0.0)
 , m_pitchCurEffort(0.0)
 , m_pitchEffortTarget(0.0)
 , m_enableJoystick(CONFIG_PARAM_PATH + "enableJoystick", true)
 , m_joystickButton9Pressed(false)
 , m_joystickSaysEnabled(false)
 , m_haveTarget(false)
 , m_recording(false)
 , m_calibrating(false)
 , m_calibSafetyMargin(CONFIG_PARAM_PATH + "calib/safetyMargin", 0.0, 0.005, 0.4, 0.05)
 , m_calibParamString(CONFIG_PARAM_PATH + "calib/paramString", "")
 , m_plotCalibData(CONFIG_PARAM_PATH + "calib/plotCalibData", false)
 , m_plotData(CONFIG_PARAM_PATH + "plotData", false)
 , m_PM(PM_COUNT, RESOURCE_PATH)
{
	// Initialise calibration feature
	initCalibration();

	// Configure the plot manager
	configurePlotManager();

	// Subscribe to ROS topics
	m_sub_lookAtTarget = m_nhs.subscribe(RESOURCE_PATH + "target", 1, &HeadControl::handleLookAtTarget, this);
	m_sub_joystickData = m_nhs.subscribe("joy", 1, &HeadControl::handleJoystickData, this);

	// Advertise ROS topics
	m_pub_headControlStatus = m_nhs.advertise<head_control::HeadControlStatus>(RESOURCE_PATH + "status", 1, true);
	updateHeadControlStatus();

	// Configuration parameter callbacks
	m_plotData.setCallback(boost::bind(&HeadControl::callbackPlotData, this), true);

	// Initialise the position limiter
	m_posLimiter.setParams(&m_curCalibParams);
}

// Destructor
HeadControl::~HeadControl()
{
	// Send a last status update
	resetTarget();
	updateHeadControlStatus();
}

// Initialisation function
bool HeadControl::init(robotcontrol::RobotModel* model)
{
	// Save pointer to RobotModel
	m_model = model;

	// Initialise the motion module
	MotionModule::init(m_model);
  
	// Get references to the head joints
	m_headYawJoint = m_model->getJoint("neck_yaw");
	m_headPitchJoint = m_model->getJoint("head_pitch");

	// Reset the head control target
	resetTarget();

	// Return successful initialisation
	return true;
}

// Head control trigger function
bool HeadControl::isTriggered()
{
	// Clear the plot manager for a new cycle
	m_PM.clear();

	// Determine whether head control should execute
	bool shouldTrigger = active();

	// Print messages for the enabling and disabling of head control
	if(shouldTrigger != m_lastActive)
		ROS_INFO("Headcontrol has been %s", (shouldTrigger ? "enabled" : "disabled"));

	// Handle case if we don't wish to trigger
	if(!shouldTrigger)
	{
		resetTarget();
		m_yawCurX = m_headYawJoint->lastCmd.pos;
		m_yawCurV = 0.0;
		m_yawCurEffort = m_headYawJoint->lastCmd.effort;
		m_pitchCurX = m_headPitchJoint->lastCmd.pos;
		m_pitchCurV = 0.0;
		m_pitchCurEffort = m_headPitchJoint->lastCmd.effort;
		if(m_lastActive)
			updateHeadControlStatus();
	}
	m_lastActive = shouldTrigger;

	// Record data if we are currently required to do so
	if(m_recording)
		recordPoint();

	// Plotting
	if(m_PM.getEnabled())
	{
		m_PM.plotScalar(shouldTrigger, PM_ENABLED);
		m_PM.plotScalar(m_haveTarget, PM_HAVETARGET);
		m_PM.plotScalar(m_yawTarget, PM_TARGETYAW);
		m_PM.plotScalar(m_yawEffortTarget, PM_TARGETYAWEFFORT);
		m_PM.plotScalar(m_pitchTarget, PM_TARGETPITCH);
		m_PM.plotScalar(m_pitchEffortTarget, PM_TARGETPITCHEFFORT);
		if(!shouldTrigger)
		{
			m_PM.plotScalar(m_yawCurX, PM_CURYAWX);
			m_PM.plotScalar(m_yawCurV, PM_CURYAWV);
			m_PM.plotScalar(m_yawCurEffort, PM_CURYAWEFFORT);
			m_PM.plotScalar(m_pitchCurX, PM_CURPITCHX);
			m_PM.plotScalar(m_pitchCurV, PM_CURPITCHV);
			m_PM.plotScalar(m_pitchCurEffort, PM_CURPITCHEFFORT);
			m_PM.publish();
		}
	}

	// Return whether head control should trigger
	return shouldTrigger;
}

// Head control step function
void HeadControl::step()
{
	// Retrieve the current simulation loop time
	const double dT = m_model->timerDuration();

	// If another motion module is overwriting head control, then adapt the current head control state as appropriate
	if(m_yawCurX != m_headYawJoint->lastCmd.pos)
	{
		m_yawCurX = m_headYawJoint->lastCmd.pos;
		m_yawCurV = m_headYawJoint->lastCmd.vel;
	}
	m_yawCurEffort = m_headYawJoint->lastCmd.effort;
	if(m_pitchCurX != m_headPitchJoint->lastCmd.pos)
	{
		m_pitchCurX = m_headPitchJoint->lastCmd.pos;
		m_pitchCurV = m_headPitchJoint->lastCmd.vel;
	}
	m_pitchCurEffort = m_headPitchJoint->lastCmd.effort;

	// Initialise variables
	double yawCmdX = m_yawCurX;
	double yawCmdV = m_yawCurV;
	double yawEffort = m_yawCurEffort;
	double pitchCmdX = m_pitchCurX;
	double pitchCmdV = m_pitchCurV;
	double pitchEffort = m_pitchCurEffort;

	// Update the yaw and pitch commands based on a spline approach
	m_yawSpline.setParams(m_yawCurX, m_yawCurV, m_yawTarget, 0.0, m_yawMaxVel(), m_yawMaxAcc());
	m_pitchSpline.setParams(m_pitchCurX, m_pitchCurV, m_pitchTarget, 0.0, m_pitchMaxVel(), m_pitchMaxAcc());
	double yawTime = m_yawSpline.T();
	double pitchTime = m_pitchSpline.T();
	double time = std::max(yawTime, pitchTime);
	if(time < dT) time = dT;
	if(yawTime >= dT && pitchTime >= dT)
	{
		double incYawTime = dT * yawTime / time;
		yawCmdX = m_yawSpline.x(incYawTime);
		yawCmdV = m_yawSpline.v(incYawTime);
		double incPitchTime = dT * pitchTime / time;
		pitchCmdX = m_pitchSpline.x(incPitchTime);
		pitchCmdV = m_pitchSpline.v(incPitchTime);
	}
	else
	{
		if(yawTime < dT) // Yaw is within reach of its target
		{
			yawCmdX = m_yawTarget;
			yawCmdV = 0.0;
		}
		else // Control yaw normally as only it has far to go still
		{
			yawCmdX = m_yawSpline.x(dT);
			yawCmdV = m_yawSpline.v(dT);
		}
		if(pitchTime < dT) // Pitch is within reach of its target
		{
			pitchCmdX = m_pitchTarget;
			pitchCmdV = 0.0;
		}
		else // Control pitch normally as only it has far to go still
		{
			pitchCmdX = m_pitchSpline.x(dT);
			pitchCmdV = m_pitchSpline.v(dT);
		}
	}

	// Apply yaw velocity limiting
	double yawMaxInc = fabs(dT * m_yawMaxVel());
	if(fabs(yawCmdX - m_yawCurX) > yawMaxInc)
	{
		if(yawCmdX < m_yawCurX)
		{
			yawCmdX = m_yawCurX - yawMaxInc;
			yawCmdV = -m_yawMaxVel();
		}
		else
		{
			yawCmdX = m_yawCurX + yawMaxInc;
			yawCmdV = m_yawMaxVel();
		}
	}

	// Apply pitch velocity limiting
	double pitchMaxInc = fabs(dT * m_pitchMaxVel());
	if(fabs(pitchCmdX - m_pitchCurX) > pitchMaxInc)
	{
		if(pitchCmdX < m_pitchCurX)
		{
			pitchCmdX = m_pitchCurX - pitchMaxInc;
			pitchCmdV = -m_pitchMaxVel();
		}
		else
		{
			pitchCmdX = m_pitchCurX + pitchMaxInc;
			pitchCmdV = m_pitchMaxVel();
		}
	}

	// Apply head position limiting
	m_posLimiter.applyLimit(yawCmdX, pitchCmdX);

	// Update the yaw effort based on a spline approach
	double yawEffortTarget = m_yawEffortTarget;
	if(yawEffortTarget == 0.0) yawEffortTarget = m_yawDefaultEffort();
	else if(yawEffortTarget < 0.0) yawEffortTarget = 0.0;
	else if(yawEffortTarget > 1.0) yawEffortTarget = 1.0;
	if(yawEffortTarget <= 0.0)
		yawEffort = 0.0;
	else
	{
		m_yawEffortSpline.setParams(m_yawCurEffort, yawEffortTarget, time);
		yawEffort = m_yawEffortSpline.x(dT);
	}
	if(m_yawRelax()) yawEffort = 0.0;

	// Update the pitch effort based on a spline approach
	double pitchEffortTarget = m_pitchEffortTarget;
	if(pitchEffortTarget == 0.0) pitchEffortTarget = m_pitchDefaultEffort();
	else if(pitchEffortTarget < 0.0) pitchEffortTarget = 0.0;
	else if(pitchEffortTarget > 1.0) pitchEffortTarget = 1.0;
	if(pitchEffortTarget <= 0.0)
		pitchEffort = 0.0;
	else
	{
		m_pitchEffortSpline.setParams(m_pitchCurEffort, pitchEffortTarget, time);
		pitchEffort = m_pitchEffortSpline.x(dT);
	}
	if(m_pitchRelax()) pitchEffort = 0.0;

	// Command the required new head joint positions
	m_headYawJoint->cmd.setFromPos(yawCmdX);
	m_headPitchJoint->cmd.setFromPos(pitchCmdX);
	m_headYawJoint->cmd.effort = yawEffort;
	m_headPitchJoint->cmd.effort = pitchEffort;
	m_headYawJoint->cmd.raw = (yawEffort <= 0.0);
	m_headPitchJoint->cmd.raw = (pitchEffort <= 0.0);

	// Update the last sent commands
	m_yawCurX = yawCmdX;
	m_yawCurV = yawCmdV;
	m_yawCurEffort = yawEffort;
	m_pitchCurX = pitchCmdX;
	m_pitchCurV = pitchCmdV;
	m_pitchCurEffort = pitchEffort;

	// Plotting
	if(m_PM.getEnabled())
	{
		m_PM.plotScalar(time, PM_TIMETOTARGET);
		m_PM.plotScalar(m_yawCurX, PM_CURYAWX);
		m_PM.plotScalar(m_yawCurV, PM_CURYAWV);
		m_PM.plotScalar(m_yawCurEffort, PM_CURYAWEFFORT);
		m_PM.plotScalar(m_pitchCurX, PM_CURPITCHX);
		m_PM.plotScalar(m_pitchCurV, PM_CURPITCHV);
		m_PM.plotScalar(m_pitchCurEffort, PM_CURPITCHEFFORT);
		m_PM.publish();
	}
}

// Reset the head control target
void HeadControl::resetTarget()
{
	// Reset the target variables
	m_haveTarget = false;
	m_yawTarget = 0.0;
	m_yawEffortTarget = m_yawDefaultEffort();
	m_pitchTarget = 0.0;
	m_pitchEffortTarget = m_pitchDefaultEffort();
}

// Handle data from look at target ROS topic
void HeadControl::handleLookAtTarget(const head_control::LookAtTargetConstPtr& msg)
{
	// Protect against numerical corruption
	if(!std::isfinite(msg->pitchEffort) || !std::isfinite(msg->yawEffort))
	{
		ROS_WARN_THROTTLE(0.5, "A non-finite effort (%.2f, %.2f) was received by the head control => Ignoring entire message!", msg->pitchEffort, msg->yawEffort);
		return;
	}
	if(!std::isfinite(msg->vec.x) || !std::isfinite(msg->vec.y) || !std::isfinite(msg->vec.z))
	{
		ROS_WARN_THROTTLE(0.5, "A non-finite vector target (%.3f, %.3f, %.3f) was received by the head control => Ignoring entire message!", msg->vec.x, msg->vec.y, msg->vec.z);
		return;
	}

	// Check whether head control should be enabled
	if(msg->enabled)
		m_haveTarget = true;
	else
	{
		resetTarget();
		updateHeadControlStatus();
		return;
	}

	// Process the received command vector data
	if(msg->is_angular_data)
	{
		m_yawTarget   = msg->vec.z;
		m_pitchTarget = msg->vec.y;
		if(msg->is_relative)
		{
			m_yawTarget   += m_headYawJoint->feedback.pos;
			m_pitchTarget += m_headPitchJoint->feedback.pos;
		}
	}
	else
	{
		m_yawTarget   = atan2(-msg->vec.x, msg->vec.z);
		m_pitchTarget = atan2( msg->vec.y, msg->vec.z);
	}

	// Transcribe the requested efforts
	m_yawEffortTarget = msg->yawEffort;
	m_pitchEffortTarget = msg->pitchEffort;

	// Apply position limits to the target
	m_posLimiter.applyLimit(m_yawTarget, m_pitchTarget);

	// Update the head control status
	updateHeadControlStatus();
}

// Publish head control status data
void HeadControl::updateHeadControlStatus()
{
	// Construct the required message
	m_msgStatus.active = active();
	m_msgStatus.yawTarget = m_yawTarget;
	m_msgStatus.yawTargetEffort = m_yawEffortTarget;
	m_msgStatus.pitchTarget = m_pitchTarget;
	m_msgStatus.pitchTargetEffort = m_pitchEffortTarget;

	// Publish the required data
	m_pub_headControlStatus.publish(m_msgStatus);
}

// Handle data from the joystick
void HeadControl::handleJoystickData(const sensor_msgs::JoyConstPtr& msg)
{
	// Ignore this message if the joystick isn't enabled or doesn't have enough axes
	if(!m_enableJoystick() || msg->axes.size() < 4) return;

	// Toggle joystick head control on falling edges of button 9
	if(m_joystickButton9Pressed && !msg->buttons[9])
		m_joystickSaysEnabled = !m_joystickSaysEnabled;
	m_joystickButton9Pressed = msg->buttons[9];

	// If head control is active then accept the joystick position as a new target
	head_control::LookAtTargetPtr lookMsg = boost::make_shared<head_control::LookAtTarget>();
	lookMsg->enabled = m_joystickSaysEnabled;
	lookMsg->is_angular_data = true;
	lookMsg->is_relative = false;
	lookMsg->pitchEffort = 0.0;
	lookMsg->yawEffort = 0.0;
	lookMsg->vec.x = 0.0;
	lookMsg->vec.y = 0.1;
	lookMsg->vec.z = 1.5*msg->axes[3];
	handleLookAtTarget(lookMsg);
}

// Initialise the calibration feature
void HeadControl::initCalibration()
{
	// Advertise ROS services
	m_srv_startRecording = m_nhs.advertiseService(RESOURCE_PATH + "startRecording", &HeadControl::startRecording, this);
	m_srv_stopRecording = m_nhs.advertiseService(RESOURCE_PATH + "stopRecording", &HeadControl::stopRecording, this);
	m_srv_startCalibration = m_nhs.advertiseService(RESOURCE_PATH + "startCalibration", &HeadControl::startCalibration, this);
	m_srv_stopCalibration = m_nhs.advertiseService(RESOURCE_PATH + "stopCalibration", &HeadControl::stopCalibration, this);
	m_srv_showHeadLimits = m_nhs.advertiseService(RESOURCE_PATH + "showHeadLimits", &HeadControl::showHeadLimits, this);
	m_srv_testHeadLimits = m_nhs.advertiseService(RESOURCE_PATH + "testHeadLimits", &HeadControl::testHeadLimits, this);

	// Set up the plotting of the recorded data points
	m_pub_dataMarker = m_nhs.advertise<visualization_msgs::Marker>(RESOURCE_PATH + "head_recorded_marker", 1);
	m_dataMarker.header.frame_id = "/ego_rot";
	m_dataMarker.header.stamp = ros::Time::now();
	m_dataMarker.ns = "headcontrol";
	m_dataMarker.id = 0;
	m_dataMarker.type = visualization_msgs::Marker::POINTS;
	m_dataMarker.action = visualization_msgs::Marker::ADD;
	m_dataMarker.pose.position.x = 0.0;
	m_dataMarker.pose.position.y = 0.0;
	m_dataMarker.pose.position.z = 0.0;
	m_dataMarker.scale.x = 0.007 * HEAD_PLOT_SCALE;
	m_dataMarker.scale.y = 0.007 * HEAD_PLOT_SCALE;
	m_dataMarker.color.a = 1.0;
	m_dataMarker.color.r = 0.5;
	m_dataMarker.color.g = 0.0;
	m_dataMarker.color.b = 1.0;
	m_dataMarker.lifetime = ros::Duration(0, 0);

	// Set up the plotting of the recorded data points
	m_pub_calibMarker = m_nhs.advertise<visualization_msgs::Marker>(RESOURCE_PATH + "head_calib_marker", 1);
	m_calibMarker.header.frame_id = "/ego_rot";
	m_calibMarker.header.stamp = ros::Time::now();
	m_calibMarker.ns = "headcontrol";
	m_calibMarker.id = 0;
	m_calibMarker.type = visualization_msgs::Marker::POINTS;
	m_calibMarker.action = visualization_msgs::Marker::ADD;
	m_calibMarker.pose.position.x = 0.0;
	m_calibMarker.pose.position.y = 0.0;
	m_calibMarker.pose.position.z = 0.0;
	m_calibMarker.scale.x = 0.016 * HEAD_PLOT_SCALE;
	m_calibMarker.scale.y = 0.016 * HEAD_PLOT_SCALE;
	m_calibMarker.color.a = 1.0;
	m_calibMarker.color.r = 0.0;
	m_calibMarker.color.g = 0.5;
	m_calibMarker.color.b = 1.0;
	m_calibMarker.lifetime = ros::Duration(0, 0);

	// Set up the calibration parameter string config server callback
	m_calibParamString.setCallback(boost::bind(&HeadControl::handleCalibParamString, this));
	handleCalibParamString();

	// Set up the calibration data plotting config server parameter callback
	m_plotCalibData.setCallback(boost::bind(&HeadControl::updatePlotCalibData, this));
	updatePlotCalibData();
}

// Add a calibration data point
void HeadControl::recordPoint()
{
	// Add the current measured feedback position
	Point data = std::make_pair(m_headYawJoint->feedback.pos, m_headPitchJoint->feedback.pos);
	m_recordedData.push_back(data);
	geometry_msgs::Point p;
	p.x = HEAD_PLOT_SCALE * data.first;
	p.y = HEAD_PLOT_SCALE * data.second;
	p.z = 0.0;
	m_dataMarker.points.push_back(p);
	updatePlotCalibData();
}

// Handle the receipt of a clicked point
void HeadControl::handleClickedPoint(const geometry_msgs::PointStampedConstPtr& msg)
{
	// Don't do anything if we're not calibrating
	if(!m_calibrating) return;

	// Add the point to our list
	Point data = std::make_pair(msg->point.x / HEAD_PLOT_SCALE, msg->point.y / HEAD_PLOT_SCALE);
	m_calibData.push_back(data);

	// Calculate and plot the calibration parameters
	calcCalibParams(false);
	plotCalibParams();

	// Inform the user that a point was clicked
	ROS_INFO("Clicked boundary point #%u: (%.3f, %.3f)", (unsigned int) m_calibData.size(), data.first, data.second);
}

// Handle a change of the calibration parameter string config server parameter
void HeadControl::handleCalibParamString()
{
	// Deserialise the calib param string
	BoundarySegment::deserialiseArray(m_calibParamString(), m_curCalibParams);
}

// Fit a boundary segment line to two points
BoundarySegment HeadControl::fitLine(const Point& p1, const Point& p2)
{
	// Declare variables
	BoundarySegment B;

	// Solve a set of linear equations for the required coefficients
	Eigen::Matrix2d A;
	A << p1.first, p1.second, p2.first, p2.second;
	Eigen::Vector2d b(-1.0, -1.0);
	Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
	double relativeError = (A*x - b).norm() / b.norm();
	if(relativeError > 1e-8 || (fabs(x[0]) < 1e-6 && fabs(x[1]) < 1e-6)) return B; // Returns invalid B

	// Transcribe the coefficients into the boundary segment object
	B.a = x[0];
	B.b = x[1];
	B.c = 1.0;
	B.d = 0.0;

	// Set up the boundary segment parameterisation to be the least numerically sensitive
	if(fabs(B.a) < fabs(B.b))
	{
		B.useX = true;
		B.tStart = std::min(p1.first, p2.first);
		B.tEnd   = std::max(p1.first, p2.first);
	}
	else
	{
		B.useX = false;
		B.tStart = std::min(p1.second, p2.second);
		B.tEnd   = std::max(p1.second, p2.second);
	}

	// Return the constructed boundary segment object
	return B;
}

// Fit a boundary segment parabola to three points
BoundarySegment HeadControl::fitParabola(const Point& p1, const Point& p2, const Point& p3)
{
	// Declare variables
	BoundarySegment B;

	// Solve a set of linear equations for the required coefficients
	Eigen::Matrix3d A;
	A << p1.first, p1.second, p1.first*p1.first, p2.first, p2.second, p2.first*p2.first, p3.first, p3.second, p3.first*p3.first;
	Eigen::Vector3d b(-1.0, -1.0, -1.0);
	Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
	double relativeError = (A*x - b).norm() / b.norm();
	if(relativeError > 1e-8 || (fabs(x[0]) < 1e-6 && fabs(x[1]) < 1e-6 && fabs(x[2]) < 1e-6)) return B; // Returns invalid B

	// Transcribe the coefficients into the boundary segment object
	B.a = x[0];
	B.b = x[1];
	B.c = 1.0;
	B.d = x[2];
	B.useX = true;
	B.tStart = std::min(std::min(p1.first, p2.first), p3.first);
	B.tEnd   = std::max(std::max(p1.first, p2.first), p3.first);

	// Return the constructed boundary segment object
	return B;
}

// ROS service handler for starting recording of data
bool HeadControl::startRecording()
{
	// Inform the user that recording has started
	ROS_INFO("Starting to record head position data...");
	ROS_INFO("Rotate the robot head along the entire mechanical limits of the neck");

	// Set the internal recording flag
	m_recording = true;

	// Clear the existing recorded data
	m_recordedData.clear();
	m_dataMarker.points.clear();

	// Publish the cleared marker
	updatePlotCalibData();

	// Return that the service call was successfully handled
	return true;
}

// ROS service handler for stopping recording of data
bool HeadControl::stopRecording()
{
	// Don't do anything if a calibration isn't active
	if(!m_recording)
	{
		ROS_INFO("Stop recording: Service call received even though recording is not active");
		return true;
	}

	// Inform the user that recording has stopped
	ROS_INFO("Stopping recording of head position data...");

	// Unset the internal recording flag
	m_recording = false;

	// Return that the service call was successfully handled
	return true;
}

// ROS service handler for starting calibration
bool HeadControl::startCalibration()
{
	// Stop recording automatically if this service is called
	if(m_recording)
		stopRecording();

	// Inform the user that the head control calibration has started
	ROS_INFO("Starting calibration of the allowable head workspace...");
	ROS_INFO("I expect points to be clicked/published in RViz, where the first five points are joined by two parabolas, and the remaining points are joined by lines.");

	// Set the internal calibrating flag
	m_calibrating = true;

	// Clear the existing calibration data
	m_calibData.clear();
	m_calibParams.clear();
	m_calibMarker.points.clear();

	// Publish the cleared marker
	updatePlotCalibData();

	// Start monitoring for clicked points in RViz
	subscribeClickedPoint();

	// Return that the service call was successfully handled
	return true;
}

// ROS service handler for stopping calibration
bool HeadControl::stopCalibration()
{
	// Don't do anything if a calibration isn't active
	if(!m_calibrating)
	{
		ROS_INFO("Stop calibration: Service call received even though calibration is not active");
		return true;
	}

	// Indicate to the user that this service call has been received
	ROS_INFO("Stopped head control calibration");

	// Stop monitoring for clicked points in RViz
	unsubscribeClickedPoint();

	// Unset the internal calibrating flag
	m_calibrating = false;

	// Make sure we have enough points
	if(m_calibData.size() < 3)
	{
		ROS_INFO("Not enough points were clicked for a boundary => I expect at least three!");
		m_calibParams.clear();
		m_calibMarker.points.clear();
		return true;
	}

	// Calculate and plot the calibration parameters
	calcCalibParams(true);
	plotCalibParams();

	// Serialise the calibration parameters
	std::string paramString = BoundarySegment::serialiseArray(m_calibParams);

	// Set the parameter string on the config server
	m_calibParamString.set(paramString);
	m_curCalibParams.assign(m_calibParams.begin(), m_calibParams.end());

	// Print the required parameters just in case
	ROS_INFO("Head limit calibration finished with new parameter string:\nparamString: \"%s\"", paramString.c_str());
	ROS_INFO("This parameter string has been set on the config server");

	// Return that the service call was successfully handled
	return true;
}

// Update the calib markers to show the current head limits in use
bool HeadControl::showHeadLimits()
{
	// Don't do anything if calibration if currently in progress
	if(m_calibrating)
	{
		ROS_INFO("Show head limits: Not modifying anything as calibration is currently active");
		return true;
	}

	// Copy the current calibration parameters into the working array
	m_calibParams.assign(m_curCalibParams.begin(), m_curCalibParams.end());

	// Plot the required head limits
	plotCalibParams();

	// Return that the service was handled
	return true;
}

// Test the head limits
bool HeadControl::testHeadLimits()
{
	// Plot the calibration data
	if(!showHeadLimits()) return false;

	// We hijack the recorded points marker
	m_dataMarker.points.clear();

	// Declare variables
	geometry_msgs::Point p, q;
	p.z = 0.0;
	q.z = 0.02;

	// Push test data into the points
	static const int N = 3600;
	double radius = 0.5 + 2.0*drand48();
	for(int n = 0; n < N; n++)
	{
		double angle = n * (M_2PI / N);
		p.x = q.x = radius * cos(angle);
		p.y = q.y = radius * sin(angle);
		m_posLimiter.applyLimit(q.x, q.y);
		p.x *= HEAD_PLOT_SCALE;
		p.y *= HEAD_PLOT_SCALE;
		q.x *= HEAD_PLOT_SCALE;
		q.y *= HEAD_PLOT_SCALE;
		m_dataMarker.points.push_back(p);
		m_dataMarker.points.push_back(q);
	}

	// Update the plotted information
	updatePlotCalibData();

	// Return success
	return true;
}

// Recalculate the calibration parameters
void HeadControl::calcCalibParams(bool shrink)
{
	// Get the number of calibration points we have to work with
	size_t numPts = m_calibData.size();

	// Compute the calibration boundary parameters based on the clicked points
	m_calibParams.clear();
	if(numPts == 2) m_calibParams.push_back(fitLine(m_calibData[0], m_calibData[1]));
	else if(numPts >= 3) m_calibParams.push_back(fitParabola(m_calibData[0], m_calibData[1], m_calibData[2]));
	if(numPts == 4) m_calibParams.push_back(fitLine(m_calibData[2], m_calibData[3]));
	else if(numPts >= 5) m_calibParams.push_back(fitParabola(m_calibData[2], m_calibData[3], m_calibData[4]));
	for(size_t i = 5; i < numPts; i++)
		m_calibParams.push_back(fitLine(m_calibData[i-1], m_calibData[i]));
	if(numPts >= 3) m_calibParams.push_back(fitLine(m_calibData.back(), m_calibData.front()));
	m_calibParams.erase(std::remove_if(m_calibParams.begin(), m_calibParams.end(), BoundarySegment::isInvalid), m_calibParams.end());

	// Shrink the boundary by a configured safety margin
	if(shrink)
	{
		double margin = m_calibSafetyMargin();
		for(size_t i = 0; i < m_calibParams.size(); i++)
			m_calibParams[i].shiftBy(margin);
	}
}

// Plot the calibration parameters
void HeadControl::plotCalibParams()
{
	// Declare variables
	const int N = 50; // Number of points per segment
	geometry_msgs::Point p;
	p.z = 0.0;

	// Plot the calib boundary defined by the calib params
	m_calibMarker.points.clear();
	for(size_t i = 0; i < m_calibParams.size(); i++)
	{
		const BoundarySegment& B = m_calibParams[i];
		for(int j = 0; j <= N; j++)
		{
			double t = B.tStart + j*(B.tEnd - B.tStart) / ((double) N);
			if(B.useX)
			{
				p.x = HEAD_PLOT_SCALE * t;
				p.y = HEAD_PLOT_SCALE * (B.c + t*(B.a + t*B.d)) / -B.b;
			}
			else // Assume this is not a quadratic if we are using y...
			{
				p.x = HEAD_PLOT_SCALE * (B.c + t*B.b) / -B.a;
				p.y = HEAD_PLOT_SCALE * t;
			}
			m_calibMarker.points.push_back(p);
		}
	}

	// Add in all the original clicked calibration points just in case
	for(size_t i = 0; i < m_calibData.size(); i++)
	{
		p.x = HEAD_PLOT_SCALE * m_calibData[i].first;
		p.y = HEAD_PLOT_SCALE * m_calibData[i].second;
		m_calibMarker.points.push_back(p);
	}

	// Update the plotted information
	updatePlotCalibData();
}

// Update functions for config server parameters
void HeadControl::updatePlotCalibData()
{
	// Get the current time
	ros::Time now = ros::Time::now();

	// Update the plotting of the recorded data marker
	if(m_plotCalibData() && !m_dataMarker.points.empty())
		m_dataMarker.lifetime = ros::Duration(0, 0); // Set the marker to never expire
	else
		m_dataMarker.lifetime = ros::Duration(0.05); // Set the marker to expire after a short amount of time
	m_dataMarker.header.stamp = now;
	m_pub_dataMarker.publish(m_dataMarker);

	// Update the plotting of the calib data marker
	if(m_plotCalibData() && !m_calibMarker.points.empty())
		m_calibMarker.lifetime = ros::Duration(0, 0); // Set the marker to never expire
	else
		m_calibMarker.lifetime = ros::Duration(0.05); // Set the marker to expire after a short amount of time
	m_calibMarker.header.stamp = now;
	m_pub_calibMarker.publish(m_calibMarker);
}

// Configure the plot manager
void HeadControl::configurePlotManager()
{
	// Configure gait command vector variables
	m_PM.setName(PM_ENABLED,           "enabled");
	m_PM.setName(PM_HAVETARGET,        "haveTarget");
	m_PM.setName(PM_TARGETYAW,         "target/yaw");
	m_PM.setName(PM_TARGETYAWEFFORT,   "target/yawEffort");
	m_PM.setName(PM_TARGETPITCH,       "target/pitch");
	m_PM.setName(PM_TARGETPITCHEFFORT, "target/pitchEffort");
	m_PM.setName(PM_TIMETOTARGET,      "timeToTarget");
	m_PM.setName(PM_CURYAWX,           "current/yaw");
	m_PM.setName(PM_CURYAWV,           "current/yawVel");
	m_PM.setName(PM_CURYAWEFFORT,      "current/yawEffort");
	m_PM.setName(PM_CURPITCHX,         "current/pitch");
	m_PM.setName(PM_CURPITCHV,         "current/pitchVel");
	m_PM.setName(PM_CURPITCHEFFORT,    "current/pitchEffort");

	// Check that we have been thorough
	if(!m_PM.checkNames())
		ROS_ERROR("Please review any warnings above that are related to the naming of plotter variables!");
}

// Callback for when the plotData parameter is updated
void HeadControl::callbackPlotData()
{
	// Enable or disable plotting as required
	if(m_plotData()) m_PM.enable();
	else             m_PM.disable();
}

//
// BoundarySegment struct
//

// Constants
const double BoundarySegment::minC = 0.01;

// Evaluate the boundary segment at a parameterised t value
void BoundarySegment::evaluateAt(double t, double& x, double& y) const
{
	// Evaluate the boundary segment as required
	if(useX)
	{
		x = t;
		y = -(a*x + c + d*x*x) / b;
	}
	else
	{
		y = t;
		x = -(b*y + c) / a; // If useX is false then d is assumed to be zero
	}
}

// Shift a boundary by a given margin towards (0,0)
void BoundarySegment::shiftBy(double margin)
{
	// Shift differently depending on whether it's a straight line segment or a parabola
	if(fabs(d) < 1e-6) // Line
		c -= margin * sqrt(a*a + b*b);
	else // Parabola
		c -=  margin * fabs(b);
	if(c < minC) c = minC;
}

// Serialise an array of boundary segment parameters to a string
std::string BoundarySegment::serialiseArray(const std::vector<BoundarySegment>& array)
{
	// Initialise string stream
	std::ostringstream ss;
	ss << std::fixed << std::setprecision(8);

	// Serialise each element in the array in turn
	for(size_t i = 0; i < array.size(); i++)
	{
		if(i != 0) ss << "|";
		ss << array[i].a << " " << array[i].b << " " << array[i].c << " " << array[i].d << " " << array[i].tStart << " " << array[i].tEnd << " " << array[i].useX;
	}

	// Return the serialised string
	return ss.str();
}

// Deserialise a string into an array of boundary segment parameters
bool BoundarySegment::deserialiseArray(const std::string& str, std::vector<BoundarySegment>& array)
{
	// Declare variables
	BoundarySegment B;
	char pipe = '|';

	// Initialise string stream
	std::istringstream ss(str);

	// Keep reading in boundary segments until we run out
	while(!ss.fail() && pipe == '|')
	{
		ss >> B.a >> B.b >> B.c >> B.d >> B.tStart >> B.tEnd >> B.useX;
		if(ss.fail()) break;
		array.push_back(B);
		pipe = '\0';
		ss >> pipe;
	}

	// Return whether we were able to parse a single boundary segment
	return !array.empty();
}

//
// SimpleLimiter class
//

// Constants
const double SimpleLimiter::minYawCmd = -1.57;
const double SimpleLimiter::maxYawCmd = 1.57;
const double SimpleLimiter::minPitchCmd = -0.5;
const double SimpleLimiter::maxPitchCmd = 0.5;

// Apply simple limits to a head position
bool SimpleLimiter::applyLimit(double& yaw, double& pitch) const
{
	// Save the original values
	double origYaw = yaw, origPitch = pitch;

	// Apply the required independent range limits
	if(yaw < minYawCmd)
		yaw = minYawCmd;
	else if(yaw > maxYawCmd)
		yaw = maxYawCmd;
	if(pitch < minPitchCmd)
		pitch = minPitchCmd;
	else if(pitch > maxPitchCmd)
		pitch = maxPitchCmd;

	// Return whether we have modified the values
	return (yaw != origYaw || pitch != origPitch);
}

//
// BoundaryLimiter class
//

// Apply boundary limits to a head position
// The following assumptions are made about the calibration:
//  - When recording data and clicking points in RViz to set up the calibration, the fixed frame in RViz must be "ego_rot"
//  - The origin must be inside the allowed head range and not that close to the boundary of it
//  - Quadratic segments are only limiting in the wedge defined by their endpoints, so if it is not 1-1 in a polar sense
//    then the result may not be exactly what was intended
//  - Quadratic segments should not subtend more than 180 degrees at the origin as then the wedge flips
bool BoundaryLimiter::applyLimit(double& yaw, double& pitch) const
{
	// Constants
	static const double polarTol = 1e-8;
	static const double quadTol = 1e-8;

	// Save the original values
	double origYaw = yaw, origPitch = pitch;

	// Draw a ray from the origin (0,0) to the point and find the smallest contraction change along this ray that leads in a point that statisfies all boundary segment constraints
	double minLambda = 2.0; // Lambda values are measured in the range [0,1], where 0 => Origin, 1 => Input point, and outside this range means invalid/ignore.
	for(size_t i = 0; i < m_params->size(); i++)
	{
		// Get the boundary segment to limit to
		const BoundarySegment& Bseg = m_params->at(i);

		// Get the segment endpoints
		double yawS, pitchS;
		double yawE, pitchE;
		Bseg.evaluateStart(yawS, pitchS);
		Bseg.evaluateEnd(yawE, pitchE);

		// Get the polar angles of the endpoints and the input point
		double polarS = atan2(pitchS, yawS);
		double polarE = atan2(pitchE, yawE);
		double polarI = atan2(pitch, yaw);

		// Only apply this boundary segment if the input point is in the wedge of the segment, judged by endpoints
		double polarSE = rc_utils::picut(polarE - polarS);
		double polarSI = rc_utils::picut(polarI - polarS);
		if(polarSE * polarSI < 0.0 || fabs(polarSI) > fabs(polarSE) + polarTol) continue;

		// Substitute the point lambda*(yaw,pitch) into a*x + b*y + c + d*x^2 = 0 to get our equation Ca*lambda^2 + Cb*lambda + Cc = 0 that defines the lambda to use to be on the boundary segment
		double Ca = Bseg.d*yaw*yaw;
		double Cb = Bseg.a*yaw + Bseg.b*pitch;
		double Cc = Bseg.c;

		// Solve the quadratic equation numerically safely, knowing that we can ignore all solutions for lambda outside the range [0,1]
		double critsq = Cb*Cb - 4.0*Ca*Cc;
		if(critsq < 0.0) continue; // No intersection means we're fine
		double crit = sqrt(critsq);
		int signCb = rc_utils::sign(Cb);
		double A = 2.0*Ca;                // The solutions to the quadratic are now:
		double B = -Cb - signCb*crit;     //   lambda1 = B/A = C/Bstar
		double Bstar = -Cb + signCb*crit; //   lambda2 = C/B = Bstar/A
		double C = 2.0*Cc;
		double absA = fabs(A);            // If A = B = 0 then Ca = Cb = 0, which means that the segment is a line (from Ca = 0),
		double absB = fabs(B);            // that is parallel to the ray from the origin to the input point (from Cb = 0 given Ca = 0).
		double lambda1, lambda2;
		if(absA < absB)
		{
			if(absB < quadTol) continue; // Constraint line is parallel to ray, so it is not violated, so we can skip this constraint
			lambda1 = 2.0;
			lambda2 = C / B;
		}
		else
		{
			if(absA < quadTol) continue; // Constraint line is parallel to ray, so it is not violated, so we can skip this constraint
			lambda1 = B / A;
			lambda2 = Bstar / A;
		}
		if(lambda1 <= 0.0 || lambda1 >= 1.0) lambda1 = 2.0;
		if(lambda2 <= 0.0 || lambda2 >= 1.0) lambda2 = 2.0;
		if(lambda1 >= 1.0 && lambda2 >= 1.0) continue;
		double lambda = std::min(lambda1, lambda2);
		if(lambda <= 0.0 || lambda >= 1.0) continue;

		// See if this is the most restrictive lambda we've had so far
		if(lambda < minLambda) minLambda = lambda;
	}

	// Apply the limits as necessary
	if(minLambda > 0.0 && minLambda < 1.0)
	{
		yaw *= minLambda;
		pitch *= minLambda;
	}

	// Return whether we have modified the values
	return (yaw != origYaw || pitch != origPitch);
}

PLUGINLIB_EXPORT_CLASS(headcontrol::HeadControl, robotcontrol::MotionModule)
// EOF