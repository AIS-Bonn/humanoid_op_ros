// Robot control node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <robotcontrol/robotcontrol.h>
#include <robotcontrol/hw/dummyinterface.h>
#include <plot_msgs/JointCommand.h>
#include <motiontimer.h>
#include <nanosleeptimer.h>

#include <sched.h>
#include <signal.h>
#include <sstream>
#include <sys/timerfd.h>

#include <pluginlib/class_loader.h>

#include <XmlRpcValue.h>

#if defined(__i386__)
#define __NR_ioprio_set         289
#define __NR_ioprio_get         290
#elif defined(__ppc__)
#define __NR_ioprio_set         273
#define __NR_ioprio_get         274
#elif defined(__x86_64__)
#define __NR_ioprio_set         251
#define __NR_ioprio_get         252
#elif defined(__ia64__)
#define __NR_ioprio_set         1274
#define __NR_ioprio_get         1275
#else
#error "Unsupported arch"
#endif

#define IOPRIO_CLASS_SHIFT      13

static inline int ioprio_set(int which, int who, int ioprio)
{
        return syscall(__NR_ioprio_set, which, who, ioprio);
}

static inline int ioprio_get(int which, int who)
{
        return syscall(__NR_ioprio_get, which, who);
}

enum {
        IOPRIO_CLASS_NONE,
        IOPRIO_CLASS_RT,
        IOPRIO_CLASS_BE,
        IOPRIO_CLASS_IDLE,
};

enum {
        IOPRIO_WHO_PROCESS = 1,
        IOPRIO_WHO_PGRP,
        IOPRIO_WHO_USER,
};

// RobotControl namespace
namespace robotcontrol
{

// RobotControl constructor
RobotControl::RobotControl()
 : m_nh("~")
 , m_hwLoader("robotcontrol", "robotcontrol::HardwareInterface")
 , m_pluginLoader("robotcontrol", "robotcontrol::MotionModule")
 , m_robotModel(this)
 , m_pub_js_counter(0)
 , m_fadeTorqueServer(m_nh, "fade_torque", false)
 , m_fadeTorqueState(0)
 , m_newFadeTorqueGoal(false)
 , m_velLimit("jointVelLimit", 0, 0.05, 25.0, 0.5)
 , m_accLimit("jointAccLimit", 0, 0.05, 25.0, 0.5)
 , m_publishCommand("publishCommand", false)
 , m_plotRobotControlData("plotRobotControlData", false)
 , m_fadeInMaxDelta("fadeInMaxDelta", 0, 0.0001, 1.0, 0.005)
{
	// Subscribe to topics
	m_sub_js = m_nh.subscribe("/joint_state_cmds", 1, &RobotControl::handleJointStateCommand, this);
	m_sub_js_raw = m_nh.subscribe("raw_cmds", 1, &RobotControl::handleRawJointCommand, this);

	// Advertise topics to publish on
	m_pub_js = m_nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
	m_pub_js_cmd = m_nh.advertise<plot_msgs::JointCommand>("/joint_commands", 10);
	m_pub_diag = m_nh.advertise<robotcontrol::Diagnostics>("diagnostics", 1);
	m_pub_plot = m_nh.advertise<plot_msgs::Plot>("/plot", 2);
	
	// Advertise services
	m_srv_printJointCommands = m_nh.advertiseService("printJointCommands", &RobotControl::handlePrintJointCommands, this);

	// Initialise the plotter variable names
	m_plot.points.resize(PM_COUNT);
	m_plot.points[PM_TIM_PERIOD].name = "/robotcontrol/Timing/Period";
	m_plot.points[PM_TIM_MOTION].name = "/robotcontrol/Timing/0_Motion";
	m_plot.points[PM_TIM_TX    ].name = "/robotcontrol/Timing/1_Tx";
	m_plot.points[PM_TIM_RX    ].name = "/robotcontrol/Timing/2_Rx";

	// Retrieve the ROS param for whether to publish TF transforms
	m_nh.param("publish_tf", m_publishTF, true);

	// Periodic timer for the diagnostics message
	m_diagnosticsTimer = m_nh.createTimer(ros::Duration(1.0), &RobotControl::sendDiagnostics, this);
	m_diagnosticsTimer.start();

	// Start handling fade in/out requests
	m_fadeTorqueServer.start();

	// We want to be notified if the joint limits change
	m_velLimit.setCallback(boost::bind(&RobotControl::handleVelLimitUpdate, this, _1));
	m_accLimit.setCallback(boost::bind(&RobotControl::handleAccLimitUpdate, this, _1));

	// Register initial and relaxed states
	m_state_init = m_robotModel.registerState("init");
	m_state_relaxed = m_robotModel.registerState("relaxed");

	// Set the current robotmodel state
	// We keep the initState as 'unknown' unless there is a ROS parameter ordering us to do otherwise
	std::string initState = "unknown";
	m_nh.param("initial_state", initState, initState); // <-- This ROS parameter should in general *not* be set, leading to retrieval failure, and use of the default "unknown" state...
	m_robotModel.setState(m_robotModel.registerState(initState));
}

// RobotControl destructor
RobotControl::~RobotControl()
{
}

/**
 * The enabled modules are described in a "motion_modules"
 * list on the parameter server.
 *
 * @return true on success
 **/
bool RobotControl::initModules()
{
	XmlRpc::XmlRpcValue list;
	m_nh.getParam("motion_modules", list);
	ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	std::string name, param;
	size_t openBracket, closeBracket, endName;
	for(int i = 0; i < list.size(); ++i)
	{
		name = static_cast<std::string>(list[i]);

		// Allow a motion module to be specified as "motion::module" or "motion::module(param_string)"
		// For a parameter string to be successfully parsed:
		// - The ')' character must be the very last character in the string (trailing whitespace disables parameter string extraction)
		// - A matching '(' must precede the ')', and the first such opening bracket in the string is taken to delimit the parameter string
		// - Whitespace characters immediately preceding the opening bracket are discarded, to allow the parameter strings to be visually vertically aligned in the launch file
		// Example(s):
		// - "gait::Gait                (cpg_gait::CPGGait)"
		// - "limb_control::LimbControl (right_leg)"
		param.clear();
		closeBracket = name.length() - 1;
		if(name.length() > 0 && name[closeBracket] == ')')
		{
			openBracket = name.find_first_of('(');
			if(openBracket != std::string::npos)
			{
				param = name.substr(openBracket + 1, closeBracket - openBracket - 1);
				endName = name.find_last_not_of(" (", openBracket);
				if(endName != std::string::npos)
					name = name.substr(0, endName + 1);
				else
					name.clear();
				ROS_INFO("loading MotionModule '%s' with param string '%s'", name.c_str(), param.c_str());
			}
		}
		if(param.empty())
			ROS_INFO("loading MotionModule '%s'", name.c_str());

		boost::shared_ptr<MotionModule> module;
		try
		{
			module = m_pluginLoader.createInstance(name);
			if(!module)
			{
				ROS_ERROR("Could not load plugin '%s'", name.c_str());
				ROS_ERROR("I'm going to continue anyway...");
				continue;
			}
			module->setName(name);
			module->setParamString(param);
		}
		catch(pluginlib::LibraryLoadException& exc)
		{
			ROS_ERROR("Could not load plugin '%s': %s", name.c_str(), exc.what());
			ROS_ERROR("I'm going to continue anyway...");
			continue;
		}

		if(!module->init(&m_robotModel))
		{
			ROS_ERROR("Could not initialize plugin '%s'", name.c_str());
			ROS_ERROR("I'm going to continue anyway...");
			continue;
		}

		m_modules.push_back(module);
	}

	return true;
}

/**
 * Initializes the hardware interface, the robot model and the motion modules.
 *
 * @return true on success
 **/
bool RobotControl::init()
{
	// Hardware interface
	std::string hwInterface;
	m_nh.param("hw_interface", hwInterface, std::string("robotcontrol::RobotInterface"));

	m_hw = m_hwLoader.createInstance(hwInterface);
	if(!m_hw)
	{
		ROS_ERROR("Could not load hw interface plugin");
		return false;
	}

	// Robot model (from URDF model on parameter server)
	m_model = boost::make_shared<urdf::Model>();
	if(!m_model->initParam("robot_description"))
	{
		ROS_ERROR("Could not get URDF model");
		return false;
	}

	m_robotModel.setModel(m_model);

	// Create robotcontrol::Joint (or subclasses, this is decided by the
	// factory method on the HardwareInterface) for every joint in the model.
	std::vector<boost::shared_ptr<urdf::Link> > links;
	m_model->getLinks(links);

	for(size_t i = 0; i < links.size(); ++i)
	{
		// The urdf link belonging to the joint
		const boost::shared_ptr<urdf::Link>& link = links[i];

		// The urdf joint corresponding to the joint
		const boost::shared_ptr<urdf::Joint>& modelJoint = link->parent_joint;

		// Ignore joints we don't need to handle (e.g. fixed joints)
		if(!modelJoint || (
				   modelJoint->type != urdf::Joint::CONTINUOUS
				&& modelJoint->type != urdf::Joint::REVOLUTE))
			continue;

		boost::shared_ptr<Joint> joint = m_hw->createJoint(modelJoint->name);
		if(!joint)
		{
			ROS_ERROR("Could not create Joint for name '%s'", modelJoint->name.c_str());
			return false;
		}

		joint->modelJoint = modelJoint;
		joint->name = modelJoint->name;

		m_robotModel.addJoint(joint);
	}

	if(m_robotModel.numJoints() == 0)
	{
		ROS_ERROR("No joints created. Something is wrong with the URDF model.");
		return false;
	}

	// Initialize the robot model's kinematic trees for each support foot
	m_robotModel.initTrees();

	// Hardware interface initialization
	if(!m_hw->init(&m_robotModel))
	{
		ROS_ERROR("Could not initialize hw interface");
		return false;
	}

	// By default, the robot is relaxed initially to prevent accidents
	bool initRelaxed;
	m_nh.param("init_relaxed", initRelaxed, true);
	if(initRelaxed)
	{
		m_hw->setStiffness(0.0);
		m_fadeTorqueState = 0.0; // Initialise the internal fade torque server state
		m_robotModel.setState(m_state_relaxed); // Set the robot state to relaxed
	}

	// If we don't wan't to fade out in the beginning do nothing. The robot will stay
	// in state 'unknown', therefore other parts of the software are responsible to
	// manage the state. If the ROS parameter 'initial_state' is also specified however,
	// then this will be the state instead of 'unknown'.

	// Set a default joint effort
	double defaultEffort;
	m_nh.param("default_effort", defaultEffort, 0.2);
	for(size_t i = 0; i < m_robotModel.numJoints(); ++i)
	{
		m_robotModel.joint(i)->cmd.effort = defaultEffort;
	}

	// Initialize all loaded motion modules
	if(!initModules())
	{
		ROS_ERROR("Could not initialize motion modules");
		return false;
	}

	// Initialize the velocity/acceleration limits on the joints
	handleVelLimitUpdate(m_velLimit());
	handleAccLimitUpdate(m_accLimit());

	ROS_INFO("Initialization finished.");

	return true;
}

/**
 * This gets called when a JointState message arrives on the /joint_state_cmds
 * topic. This topic is mainly used for quick tests (e.g. quickly send a
 * position to a single joint) and is not recommended for real robot operation.
 *
 * Use a MotionModule instead.
 *
 * @param cmd the command in JointState format
 * @param raw set the Joint::Command::raw flag, which indicates that the servo
 *   command should not be translated by the HardwareInterface
 **/
void RobotControl::doHandleJSCommand(const sensor_msgs::JointStatePtr& cmd, bool raw)
{
	for(size_t i = 0; i < cmd->name.size(); ++i)
	{
		boost::shared_ptr<Joint> joint = m_robotModel.getJoint(cmd->name[i]);
		if(!joint)
		{
			ROS_WARN("Got joint state command for unknown joint: '%s'", cmd->name[i].c_str());
			continue;
		}

		double pos = cmd->position[i];

		// Have a quick look at the URDF limits
		boost::shared_ptr<urdf::JointLimits> limits = joint->modelJoint->limits;
		if(!raw && limits)
		{
			if(pos > limits->upper)
			{
				ROS_WARN("Joint '%s' is above upper limit: %5.3lf > %5.3lf",
					joint->name.c_str(), pos, limits->upper
				);
				pos = limits->upper;
			}
			else if(pos < limits->lower)
			{
				ROS_WARN("Joint '%s' is below lower limit: %5.3lf < %5.3lf",
					joint->name.c_str(), pos, limits->lower
				);
				pos = limits->lower;
			}
		}

		joint->cmd.raw = raw;

		// If we got velocity information, use it.
		if(cmd->velocity.size() > i)
			joint->cmd.setFromPosVel(m_robotModel.timerDuration(), pos, cmd->velocity[i]);
		else
			joint->cmd.setFromPos(m_robotModel.timerDuration(), pos);
	}
}

void RobotControl::handleJointStateCommand(const sensor_msgs::JointStatePtr& cmd)
{
	doHandleJSCommand(cmd, false);
}

void RobotControl::handleRawJointCommand(const sensor_msgs::JointStatePtr& cmd)
{
	doHandleJSCommand(cmd, true);
}

void RobotControl::handleFadeTorque()
{
	int over = 0;
	const float MAX_DELTA = m_fadeInMaxDelta(); // If the robotcontrol cycle time is dT then the fade in takes dT/MAX_DELTA seconds to complete
	float diff = m_fadeTorqueGoal->torque - m_fadeTorqueState;
	if(diff > MAX_DELTA)
		diff = MAX_DELTA;
	else if(diff < -MAX_DELTA)
		diff = -MAX_DELTA;

	if(diff > 0)
	{
		// Check if all MotionModules agree that fading in is safe
		for(unsigned int i = 0; i < m_modules.size(); ++i)
		{
			if(!m_modules[i]->isSafeToFadeIn())
			{
				ROS_WARN("MotionModule '%s' prevents us from fading in...", m_modules[i]->name().c_str());

				FadeTorqueResult res;
				res.torque = m_fadeTorqueState;
				m_fadeTorqueServer.setAborted(res);
				m_newFadeTorqueGoal = false;
				return;
			}
		}
	}

	if(fabs(diff) < 0.001)
	{
		m_fadeTorqueState = m_fadeTorqueGoal->torque;
		FadeTorqueResult res;
		res.torque = m_fadeTorqueState;
		m_fadeTorqueServer.setSucceeded(res);

		if(m_fadeTorqueState > 0.5)
		{
			if(m_robotModel.state() == m_state_relaxed)
				m_robotModel.setState(m_state_init);
			over = 1;
		}
		else
		{
			m_robotModel.setState(m_state_relaxed);
			over = 2;
		}
	}
	else
		m_fadeTorqueState += diff;

	// Don't fade the goal torque twice
	if((diff != 0.0) || m_newFadeTorqueGoal)
		m_hw->setStiffness(m_fadeTorqueState);

	FadeTorqueFeedbackPtr fb = boost::make_shared<FadeTorqueFeedback>();
	fb->current_torque = m_fadeTorqueState;
	m_fadeTorqueServer.publishFeedback(fb);

	if(over == 1)
	{
		ROS_INFO("Fade in finished => Stiffness now %.3f", m_fadeTorqueState);
		plotEvent("End fade in");
	}
	if(over == 2)
	{
		ROS_INFO("Fade out finished => Stiffness now %.3f", m_fadeTorqueState);
		plotEvent("End fade out");
	}
	m_newFadeTorqueGoal = false;
}

/**
 * This takes care of all real-time specific stuff that needs to be done
 * in every iteration.
 **/
void RobotControl::step()
{
	// Save current ROS time
	ros::Time startTime = ros::Time::now();
	ros::WallTime t0 = ros::WallTime::now();

	// Back up the current joint commands in robot model
	m_robotModel.newCommands();

	// Handle torque fade requests
	if(m_robotModel.isRelaxed())
	{
		ROS_WARN_THROTTLE(3.0, "RobotModel is artificially relaxed by motion module request");
		m_hw->setStiffness(0.0);
		m_fadeTorqueState = 0.0;
	}

	if(m_fadeTorqueServer.isActive()) // If we have a fade torque goal...
	{
		handleFadeTorque();
	}
	else if(m_fadeTorqueServer.isNewGoalAvailable())
	{
		m_fadeTorqueGoal = m_fadeTorqueServer.acceptNewGoal();
		m_newFadeTorqueGoal = true;
		ROS_INFO("Fade started with goal %.3f:", m_fadeTorqueGoal->torque);
		if(m_fadeTorqueGoal->torque > m_fadeTorqueState)
			plotEvent("Fade in");
		else if(m_fadeTorqueGoal->torque < m_fadeTorqueState)
			plotEvent("Fade out");
	}

	// Ask all loaded motion modules for their input.
	bool triggered = false;
	if(!m_hw->emergencyStopActive())
	{
		for(size_t i = 0; i < m_modules.size(); i++)
		{
			if(m_modules[i]->isTriggered())
			{
				m_modules[i]->step();
				triggered = true;
			}
		}
	}
	else
	{
		ROS_WARN_THROTTLE(1.0, "Emergency stop active! All motion stopped.");

		for(size_t i = 0; i < m_modules.size(); ++i)
			m_modules[i]->handleEmergencyStop();
	}

	// If no motion module triggered then just reassert the current position command
	if(!triggered) // TODO: Mechanism to detect which joints were written to, to make sure that joint position reassertion is more thorough
	{
		for(size_t i = 0; i < m_robotModel.numJoints(); i++)
		{
			Joint::Ptr joint = m_robotModel.joint(i);
			joint->cmd.setFromPos(m_robotModel.timerDuration(), joint->lastCmd.pos);
		}
	}

	// Calculate the inverse dynamics (i.e. the needed torques) on our robot model
	m_robotModel.doInverseDynamics();

	// Do the hardware communication cycle => Write the calculated joint targets
	ros::WallTime t1 = ros::WallTime::now();
	m_dur_motion = t1 - t0;
	m_hw->sendJointTargets();
	ros::WallTime t2 = ros::WallTime::now();
	m_dur_tx = t2 - t1;

	// Do the hardware communication cycle => Read the joint states
	m_hw->readJointStates();
	ros::WallTime t3 = ros::WallTime::now();
	m_dur_rx = t3 - t2;

	// Allow RobotModel to process the read data (e.g. perform numerical differentiation, which must be performed only once a cycle)
	m_robotModel.processReadData();

	// Publish joint states and other information
	if(m_pub_js_counter >= 0) // Note: We are checking >= 0 at the moment, so publishing will occur *every* cycle
	{
		publishJointStates();
		publishJointCommands();

		if(m_publishTF)
		{
			if(m_publishCommand())
			{
				ROS_WARN_THROTTLE(1.0, "robotcontrol/publishCommand is active! This will send invalid transforms to other nodes!");
				m_robotModel.publishTF(false);
			}
			else
				m_robotModel.publishTF(true);
		}

		for(size_t i = 0; i < m_modules.size(); ++i)
			m_modules[i]->publishTransforms();

		// Clear the visualization markers list
		m_markers.clear();

		// Update the RobotModel visualization markers
		m_robotModel.visualizeData(&m_markers);

		// Publish the accumulated RobotControl visualization markers
		m_markers.publish();

		m_pub_js_counter = 0;
	}
	else
		m_pub_js_counter++;

	// Publish plotter data
	if(m_plotRobotControlData() || !m_plot.events.empty())
	{
		// Set plot message header data
		m_plot.header.stamp = startTime;

		// Plot the timing data
		m_plot.points[PM_TIM_PERIOD].value = (t0 - m_lastIterationTime).toSec();
		m_plot.points[PM_TIM_MOTION].value = m_dur_motion.toSec();
		m_plot.points[PM_TIM_TX    ].value = m_dur_motion.toSec() + m_dur_tx.toSec();
		m_plot.points[PM_TIM_RX    ].value = m_dur_motion.toSec() + m_dur_tx.toSec() + m_dur_rx.toSec();

		// Publish the plot data
		m_pub_plot.publish(m_plot);
		
		// Clear the published events
		m_plot.events.clear();
	}

	// Save the last robotcontrol iteration time
	m_lastIterationTime = t0;
}

void RobotControl::publishJointStates()
{
	sensor_msgs::JointStatePtr js = boost::make_shared<sensor_msgs::JointState>();

	size_t nj = m_robotModel.numJoints();

	js->header.stamp = m_robotModel[0]->feedback.stamp;
	js->name.resize(nj);
	js->position.resize(nj);
	js->velocity.resize(nj);
	js->effort.resize(nj);

	for(size_t i = 0; i < nj; i++)
	{
		const Joint& joint = *m_robotModel[i];
		js->name[i] = joint.modelJoint->name.c_str();
		js->position[i] = (m_publishCommand() ? joint.cmd.pos : joint.feedback.pos);
		js->velocity[i] = 0.0; // TODO: Always returning zero velocity for now
		js->effort[i] = joint.feedback.torque;
	}

	m_pub_js.publish(js);
}

void RobotControl::publishJointCommands()
{
	plot_msgs::JointCommandPtr js = boost::make_shared<plot_msgs::JointCommand>();

	size_t nj = m_robotModel.numJoints();

	js->header.stamp = ros::Time::now();
	js->name.resize(nj);
	js->position.resize(nj);
	js->velocity.resize(nj);
	js->acceleration.resize(nj);
	js->rawPosition.resize(nj);
	js->torque.resize(nj);
	js->effort.resize(nj);

	for(size_t i = 0; i < nj; i++)
	{
		const Joint& joint = *m_robotModel[i];
		js->name[i] = joint.modelJoint->name.c_str();
		js->position[i] = joint.cmd.pos;
		js->velocity[i] = joint.cmd.vel;
		js->acceleration[i] = joint.cmd.acc;
		js->rawPosition[i] = joint.cmd.rawPos;
		js->torque[i] = joint.feedback.modelTorque;
		js->effort[i] = joint.cmd.effort;
	}

	m_pub_js_cmd.publish(js);
}

/**
 * This just asks the hardware interface to fill in the Diagnostics message
 * before sending it off.
 **/
void RobotControl::sendDiagnostics(const ros::TimerEvent& )
{
	robotcontrol::DiagnosticsPtr msg = boost::make_shared<robotcontrol::Diagnostics>();

	msg->header.stamp = ros::Time::now();
	msg->state = m_robotModel.currentStateLabel();

	m_hw->getDiagnostics(msg);

	m_pub_diag.publish(msg);
}

void RobotControl::handleVelLimitUpdate(float value)
{
	Joint::Command::velLimit = value;
}

void RobotControl::handleAccLimitUpdate(float value)
{
	Joint::Command::accLimit = value;
}

void RobotControl::plotEvent(const std::string& event)
{
	// Add the required event
	m_plot.events.push_back("/robotcontrol/events/" + event);
}

void RobotControl::printJointCommands()
{
	// Initialise variables
	std::ostringstream ss;
	size_t nj = m_robotModel.numJoints();
	
	// Print the current joint positions
	for(size_t i = 0; i < nj; i++)
	{
		const Joint& joint = *m_robotModel[i];
		ss << joint.modelJoint->name << ": " << joint.feedback.pos << " (Effort " << joint.lastCmd.effort << ")" << std::endl;
	}
	ROS_INFO("The current joint positions are:\n\n%s", ss.str().c_str());
	
	// Reset the string stream
	ss.str("");
	ss.clear();
	
	// Print the required joint commands
	// Note: You can make this output suitable for inclusion in a motion yaml file using the following regex replacement:
	//       '(.*): (.*) \(Effort (.*)\)$' --> '      \1:\n        position: \2\n        effort: \3\n        velocity: 0'
	for(size_t i = 0; i < nj; i++)
	{
		const Joint& joint = *m_robotModel[i];
		ss << joint.modelJoint->name << ": " << joint.lastCmd.pos << " (Effort " << joint.lastCmd.effort << ")" << std::endl;
	}
	ss << std::endl << "The following regex find/replace might help you:" << std::endl << "Find: '(.*): (.*) \\(Effort (.*)\\)$'" << std::endl << "Replace: '      \\1:\\n        position: \\2\\n        effort: \\3\\n        velocity: 0'" << std::endl;
	ROS_INFO("The joint commands currently being returned by the motion modules are:\n\n%s", ss.str().c_str());
}

}

bool g_shouldShutdown = false;

void signal_handler(int sig)
{
	g_shouldShutdown = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotcontrol", ros::init_options::NoSigintHandler);

	// Make sure that the timer duration is available on the config server
	config_server::Parameter<float> duration("timerDuration", MIN_TIMER_DURATION, 0.0001, MAX_TIMER_DURATION, DEFAULT_TIMER_DURATION);

	robotcontrol::RobotControl ctrl;

	// Inform the internal RobotModel of the nominal loop rate
	ROS_INFO("Using nominal loop rate of %.4fs (%.1fHz)", duration(), 1.0/duration());
	ctrl.setTimerDuration(duration());

	if(!ctrl.init())
	{
		ROS_ERROR("Could not initialize RobotControl");
		return 1;
	}

	// Handle SIGINT ourselves for a clean and controlled shutdown
	signal(SIGINT, signal_handler);

	// Get realtime priority
	sched_param schedparm;
	memset(&schedparm, 0, sizeof(schedparm));
	schedparm.sched_priority = 1;
	if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedparm) != 0)
	{
		ROS_ERROR("Could not get realtime priority");
		ROS_ERROR("I'm going to run without realtime priority!");
	}

	// Set I/O priority to realtime
	ioprio_set(IOPRIO_WHO_PROCESS, getpid(), (IOPRIO_CLASS_RT << 13) | 0);

	MotionTimer timer(duration());
	ros::Rate rate = ros::Rate(1.0 / duration());

	config_server::Parameter<bool> pause("pause", false);
	bool use_sim_time = ros::Time::isSimTime();

	ros::WallTime checkpointA, checkpointB, checkpointC;
	double TimeAB = -1.0, TimeBC = -1.0, TimeCA = -1.0;

	while(!g_shouldShutdown)
	{
		checkpointA = ros::WallTime::now();
		TimeCA = (checkpointA - checkpointC).toSec();

		ros::spinOnce(); // Do all ROS callbacks before sleeping, so the next step() iteration starts right after the next timer tick

		checkpointB = ros::WallTime::now();
		TimeAB = (checkpointB - checkpointA).toSec();

		uint64_t expirations = 0;
		if(use_sim_time)
			rate.sleep();
		else
			expirations = timer.sleep();

		checkpointC = ros::WallTime::now();
		TimeBC = (checkpointC - checkpointB).toSec();

		if(expirations > 1)
		{
			if(expirations > 2)
			{
				ctrl.printTimeDiagnostics();
				ROS_WARN("Missed %u timer cycles: Step %.6lfs -> Spin %.6lfs -> Sleep %.6lfs -> Now (%.3f)", (unsigned int) expirations-1, TimeCA, TimeAB, TimeBC, checkpointC.toSec());
				if(expirations >= 6)
				{
					std::ostringstream ss;
					ss << "Missed " << expirations-1 << " cycles";
					ctrl.plotEvent(ss.str());
					ctrl.plotEvent("Missed cycles"); // Trigger an event with a common name so that all cycle misses can be viewed in a single plot event topic
				}
			}
			else
				ROS_INFO_THROTTLE(0.1, "Missed a timer cycle: Step %.6lfs -> Spin %.6lfs -> Sleep %.6lfs -> Now (%.3f)", TimeCA, TimeAB, TimeBC, checkpointC.toSec());
		}

		if(!pause())
			ctrl.step();
	}

	return 0;
}

void robotcontrol::RobotControl::printTimeDiagnostics()
{
	ROS_INFO("Robotcontrol step timings: Motion %.6lfs, TX %.6lfs, RX %.6lfs", m_dur_motion.toSec(), m_dur_tx.toSec(), m_dur_rx.toSec());
}
// EOF
