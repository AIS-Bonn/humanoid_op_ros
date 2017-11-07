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
#include <cstdlib>
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

// Namespaces
using namespace robotcontrol;

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

// Constants
const std::string RobotControl::CONFIG_PARAM_PATH = "/robotcontrol/";

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
 , m_velLimit(CONFIG_PARAM_PATH + "jointVelLimit", 0, 0.05, 25.0, 0.5)
 , m_accLimit(CONFIG_PARAM_PATH + "jointAccLimit", 0, 0.05, 25.0, 0.5)
 , m_publishCommand(CONFIG_PARAM_PATH + "publishCommand", false)
 , m_fadeInMaxDelta(CONFIG_PARAM_PATH + "fadeInMaxDelta", 0, 0.0001, 1.0, 0.005)
 , m_PM(PM_COUNT, "~")
 , m_plotRobotControlData(CONFIG_PARAM_PATH + "plotRobotControlData", false)
{
	// Retrieve a node handle
	ros::NodeHandle nhs;

	// Advertise topics to publish on
	m_pub_js = nhs.advertise<sensor_msgs::JointState>("joint_states", 10);
	m_pub_js_cmd = nhs.advertise<plot_msgs::JointCommand>("joint_commands", 10);
	m_pub_diag = m_nh.advertise<robotcontrol::Diagnostics>("diagnostics", 1);
	
	// Advertise services
	m_srv_printJointCommands = m_nh.advertiseService("printJointCommands", &RobotControl::handlePrintJointCommands, this);

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

	// Configure the plot manager
	configurePlotManager();
}

// RobotControl destructor
RobotControl::~RobotControl()
{
}

// Deinitialise robotcontrol
void RobotControl::deinit()
{
	// Stop the diagnostics timer
	m_diagnosticsTimer.stop();

	// Deinitialise the motion modules
	for(size_t i = 0; i < m_modules.size(); i++)
		m_modules[i]->deinit();

	// Deinitialise the hardware interface
	m_hw->deinit();

	// Shut down the fade torque server
	m_fadeTorqueServer.shutdown();
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

	if(!list.valid())
	{
		ROS_ERROR("No MotionModules were given. I need a motion_modules parameter!");
		return false;
	}

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
				ROS_INFO("Loading MotionModule '%s' with param string '%s'", name.c_str(), param.c_str());
			}
		}
		if(param.empty())
			ROS_INFO("Loading MotionModule '%s'", name.c_str());

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
		joint->mimic = modelJoint->mimic;

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
		m_robotModel.joint(i)->cmd.effort = defaultEffort;

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
				ROS_WARN("Motion module '%s' is preventing us from fading in...", m_modules[i]->name().c_str());

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
		ROS_WARN_THROTTLE(3.0, "Robot is artificially relaxed by robot model request");
		m_hw->setStiffness(0.0);
		m_fadeTorqueState = 0.0;
		if(m_fadeTorqueServer.isActive())
		{
			ROS_INFO("Aborted fade due to robot model relax request");
			if(m_fadeTorqueGoal->torque <= 0.5)
				m_robotModel.setState(m_state_relaxed);
			m_fadeTorqueServer.setAborted();
		}
	}
	if(m_fadeTorqueServer.isActive()) // If we have a fade torque goal...
		handleFadeTorque();
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

	// Start the joint command update phase
	for(size_t i = 0; i < m_robotModel.numJoints(); i++)
		m_robotModel.joint(i)->cmd.startUpdatePhase();

	// Ask all loaded motion modules for their input
	if(!m_hw->emergencyStopActive())
	{
		for(size_t i = 0; i < m_modules.size(); i++)
		{
			if(m_modules[i]->isTriggered())
				m_modules[i]->step();
		}
	}
	else
	{
		ROS_WARN_THROTTLE(1.0, "Emergency stop active! All motion stopped.");
		for(size_t i = 0; i < m_modules.size(); ++i)
			m_modules[i]->handleEmergencyStop();
	}

	// Stop the joint command update phase
	for(size_t i = 0; i < m_robotModel.numJoints(); i++)
		m_robotModel.joint(i)->cmd.stopUpdatePhase(m_robotModel.timerDuration());

	// Allow the hardware interface to process the joint commands from the motion modules
	m_hw->processJointCommands();

	// Calculate the inverse dynamics (i.e. the needed torques) on our robot model
	m_robotModel.doInverseDynamics();

	// Allow the hardware interface to process the joint torques from the inverse dynamics
	m_hw->processJointTorques();

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
				ROS_WARN_THROTTLE(1.0, "Robotcontrol publishCommand is active! This will send invalid transforms to other nodes!");
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
	if(m_plotRobotControlData() || m_PM.haveEvents())
	{
		// Set the plotting time stamp
		m_PM.setTimestamp(startTime);

		// Plot data
		m_PM.plotScalar((t0 - m_lastIterationTime).toSec(), PM_TIM_PERIOD);
		m_PM.plotScalar(m_dur_motion.toSec(), PM_TIM_MOTION);
		m_PM.plotScalar(m_dur_motion.toSec() + m_dur_tx.toSec(), PM_TIM_TX);
		m_PM.plotScalar(m_dur_motion.toSec() + m_dur_tx.toSec() + m_dur_rx.toSec(), PM_TIM_RX);

		// Publish and clear the plot data
		m_PM.publish();
		m_PM.clear(startTime);
	}

	// Save the last robotcontrol iteration time
	m_lastIterationTime = t0;
}

void RobotControl::plotEvent(const std::string& name)
{
	m_PM.plotEvent(name);
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
		js->velocity[i] = 0.0; // Always returning zero velocity for now
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

void RobotControl::configurePlotManager()
{
	// Configure the plot manager variable names
	m_PM.setName(PM_TIM_PERIOD, "Timing/Period");
	m_PM.setName(PM_TIM_MOTION, "Timing/0_Motion");
	m_PM.setName(PM_TIM_TX,     "Timing/1_Tx");
	m_PM.setName(PM_TIM_RX,     "Timing/2_Rx");

	// Check that we have been thorough
	if(!m_PM.checkNames())
		ROS_ERROR("Please review any warnings above that are related to the naming of plotter variables!");
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

// Global variables
bool g_shouldShutdown = false;

// Custom SIGINT handler
void signal_handler(int sig)
{
	// Signal via a global variable that the main loop should exit
	g_shouldShutdown = true;
}

// Print time diagnostics
void robotcontrol::RobotControl::printTimeDiagnostics()
{
	// Print the required diagnostics
	ROS_INFO("Robotcontrol step timings: Motion %.6lfs, TX %.6lfs, RX %.6lfs", m_dur_motion.toSec(), m_dur_tx.toSec(), m_dur_rx.toSec());
}

// Main function
int main(int argc, char** argv)
{
	// Initialise the ROS node
	ros::init(argc, argv, "robotcontrol", ros::init_options::NoSigintHandler);

	// Make sure that the timer duration is available on the config server
	config_server::Parameter<float> duration(RobotControl::CONFIG_PARAM_PATH + "timerDuration", MIN_TIMER_DURATION, 0.0001, MAX_TIMER_DURATION, DEFAULT_TIMER_DURATION);

	// Create an instance of the main robotcontrol class
	robotcontrol::RobotControl ctrl;

	// Seed the random number generation function drand48()
	srand48((long int) ros::Time::now().nsec);

	// Inform the internal RobotModel of the nominal loop rate
	ROS_INFO("Using nominal loop rate of %.4fs (%.1fHz)", duration(), 1.0/duration());
	ctrl.setTimerDuration(duration());

	// Initialise the robotcontrol object
	if(!ctrl.init())
	{
		ROS_ERROR("Could not initialize RobotControl");
		return 1;
	}

	// Handle SIGINT ourselves for a clean and controlled shutdown
	signal(SIGINT, signal_handler);

	// Get real-time priority
	sched_param schedparm;
	memset(&schedparm, 0, sizeof(schedparm));
	schedparm.sched_priority = 1;
	if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedparm) != 0)
	{
		ROS_ERROR("Could not get realtime priority");
		ROS_ERROR("I'm going to run without realtime priority!");
	}

	// Set I/O priority to real-time
	ioprio_set(IOPRIO_WHO_PROCESS, getpid(), (IOPRIO_CLASS_RT << 13) | 0);

	// Configure the motion timer for the required nominal loop rate
	MotionTimer timer(duration());
	ros::Rate rate = ros::Rate(1.0 / duration());

	// Initialise variables
	config_server::Parameter<bool> pause(RobotControl::CONFIG_PARAM_PATH + "pause", false);
	bool use_sim_time = ros::Time::isSimTime();

	// Timing variables
	ros::WallTime checkpointA, checkpointB, checkpointC;
	double TimeAB = -1.0, TimeBC = -1.0, TimeCA = -1.0;

	// Main robotcontrol loop
	while(!g_shouldShutdown)
	{
		// Checkpoint A: Between step execution and ROS spin
		checkpointA = ros::WallTime::now();
		TimeCA = (checkpointA - checkpointC).toSec();

		// Handle ROS callbacks (Note: We do all ROS callbacks before sleeping, so that the next step() iteration starts right after the next timer tick)
		ros::spinOnce();

		// Checkpoint B: Between ROS spin and sleeping
		checkpointB = ros::WallTime::now();
		TimeAB = (checkpointB - checkpointA).toSec();

		// Sleep for the required duration
		uint64_t expirations = 0;
		if(use_sim_time)
			rate.sleep();
		else
			expirations = timer.sleep();

		// Checkpoint C: Between sleeping and step execution
		checkpointC = ros::WallTime::now();
		TimeBC = (checkpointC - checkpointB).toSec();

		// Handle case of missed timer cycles
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
		else if(expirations > 1)
			ROS_INFO_THROTTLE(10.0, "Missed a timer cycle: Step %.6lfs -> Spin %.6lfs -> Sleep %.6lfs -> Now (%.3f)", TimeCA, TimeAB, TimeBC, checkpointC.toSec());

		// Execute a step of robotcontrol
		if(!pause())
			ctrl.step();
	}

	// Deinitialise the robotcontrol object
	ctrl.deinit();

	// Notification that robotcontrol is shutting down
	ROS_INFO("Robotcontrol is exiting cleanly...");

	// Return success
	return 0;
}
// EOF
