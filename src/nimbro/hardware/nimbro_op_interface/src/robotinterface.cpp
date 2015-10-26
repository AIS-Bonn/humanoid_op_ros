// NimbRo-OP robot hardware interface
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//         Sebastian Schueller <schuell1@cs.uni-bonn.de>
//         Max Schwarz <max.schwarz@uni-bonn.de>

// Includes - Local
#include <nimbro_op_interface/robotinterface.h>

// Includes - ROS
#include <ros/console.h>
#include <ros/node_handle.h>
#include <urdf/model.h>

// Includes - Robotcontrol
#include <robotcontrol/hw/dynamiccommandgenerator.h>
#include <robotcontrol/model/robotmodel.h>
#include <robotcontrol/hw/slopelimited.h>

// Includes - Packages
#include <cm730/dynamixel.h>
#include <nimbro_op_interface/Button.h>
#include <servomodel/servocommandgenerator.h>

// Includes - Boost
#include <boost/make_shared.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/foreach.hpp>

// Includes - Misc
#include <pluginlib/class_list_macros.h>

// Configuration defines
#define DYNAMIC_BRPACKET_REORDER  1  // Non-zero => [RECOMMENDED] Servos that fail to respond too often are moved to the end of the bulk read packet
#define RESET_DEATH_CYCLES        1  // Non-zero => [RECOMMENDED] Detect severe death cycles and temporarily stop servo communications to allow the affected servo(s) to time out
#define PROD_DEATH_CYCLES         1  // Non-zero => [RECOMMENDED] Detect death cycles and prod the failing servos with a bogus write in order to snap them out of their incorrect wait state
#define PROD_DEATH_CYCLES_PREV    0  // Non-zero => [EXPERIMENTAL] When prodding the same servo a few times, sometimes prod the servo that comes before it in the bulk read packet in case that's the one the current servo is waiting for

// The following preprocessor values have been hand-tuned to work well (experimentally) with a cycle rate of 10ms. At other cycle rates that are not too different one would
// expect similar performance, but take care if the cycle period increases too much, as then the death cycle criteria may no longer be effective or even possible to fulfil
// at all. We assume nominally in the following explanations that the cycle period is 10ms (keeping in mind that the effective cycle period doubles due to the step skipping
// when servos fail).
// MAX_SENSOR_DT              ==> Cycle rate independent.
// BR_REORDER_FAIL_COUNT      ==> Cycle rate independent.
// DEATH_CYCLE_TIME/COUNT     ==> For a nominal death cycle detection time of 0.125s, we expect 0.125s / (2*10ms) = 6.25 cycles to occur.
//                                If we require a count of only 5 in this time period, then we have a 5 / 6.25 = 80% failure rate (should nominally be approx 80%).
// CONSEC_CYCLE_COUNT         ==> Cycle rate independent.
//                                For a cycle count of 4, this corresponds to a consec cycle time of 4 * (2*10ms) = 0.080s.
// SEV_DEATH_CYCLE_TIME/COUNT ==> For a nominal severe death cycle detection time of 0.400s, we expect 0.400s / (2*10ms) = 20 cycles to occur.
//                                If we require a count of only 17 in this time period, then we have a 17 / 20 = 85% failure rate (should nominally be approx 85%).
// SEV_CONSEC_CYCLE_COUNT     ==> Can be considered to be cycle rate independent, but the time period that the cycle period corresponds to should also be considered.
//                                For a target severe consec cycle time of 0.300s, we need 0.300 / (2*10ms) = 15 cycles.
// SEV_COMMS_SUSPEND_TIME     ==> Depends only and directly on the Dynamixel protocol comms timeout specification.
// COMMS_SUSPEND_CHECK_TIME   ==> Indirectly dependent on SEV_CONSEC_CYCLE_COUNT, SEV_DEATH_CYCLE_TIME and SEV_COMMS_SUSPEND_TIME.
// COMMS_SUSPEND_CHECK_COUNT  ==> Cycle rate independent.
// COMMS_SUSPEND_RECOV_TIME   ==> Cycle rate independent.

// Defines
#define MAX_CM730_DT               3.5   // Maximum dT for the CM730, in units of nominal timer durations (i.e. in nominal robotcontrol cycles), beyond which certain data is reset to zero to avoid things such as gyro integrations 'running away'
#define MIN_SENSOR_DT              0.6   // Minimum dT allowed for the sensor data, in units of nominal timer durations (i.e. in nominal robotcontrol cycles)
#define MAX_SENSOR_DT              2.5   // Maximum dT allowed for the sensor data, in units of nominal timer durations (i.e. in nominal robotcontrol cycles)
#define BR_REORDER_FAIL_COUNT      25    // Number of individual servo fails required to trigger dynamic bulk read packet reordering
#define DEATH_CYCLE_TIME           0.125 // Time over which to count total servo fails for the purpose of death cycle detection
#define DEATH_CYCLE_COUNT          5     // Number of total servo fails required within DEATH_CYCLE_TIME in order to qualify as a servo death cycle
#define CONSEC_CYCLE_COUNT         4     // Number of consecutive servo fails required in order to qualify as a servo death cycle
#define SEV_DEATH_CYCLE_TIME       0.400 // Time over which to count total servo fails for the purpose of severe death cycle detection
#define SEV_DEATH_CYCLE_COUNT      17    // Number of total servo fails required within SEV_DEATH_CYCLE_TIME in order to qualify as a severe servo death cycle
#define SEV_CONSEC_CYCLE_COUNT     15    // Number of consecutive servo fails required in order to qualify as a severe servo death cycle
#define SEV_COMMS_SUSPEND_TIME     0.115 // Time to suspend all servo communications for (that is, to ensure a servo comms timeout) in the case of a severe servo death cycle
#define COMMS_SUSPEND_CHECK_TIME   2.5   // Time over which to count comms suspensions for the purpose of comms suspension disabling
#define COMMS_SUSPEND_CHECK_COUNT  4     // Number of comms suspensions required within COMMS_SUSPEND_CHECK_TIME in order to trigger comms suspension disabling
#define COMMS_SUSPEND_RECOV_TIME   5.0   // Suspension request-free recovery time required for automatic re-enabling of comms suspensions

// Namespaces
using namespace robotcontrol;

// NimbRo-OP interface namespace
namespace nimbro_op_interface
{

/**
 * DXLJoint constructor. This function creates parameters on the config server
 * for the joint.
 **/
RobotInterface::DXLJoint::DXLJoint(const std::string& _name)
 : CONFIG_PARAM_PATH("nopInterface/DXLJoint/")
 , type(CONFIG_PARAM_PATH + "joints/" + _name + "/type", "default_type")
 , id(CONFIG_PARAM_PATH + "joints/" + _name + "/id", 1, 1, 253, 1)
 , tickOffset(CONFIG_PARAM_PATH + "offsets/" + _name, 0, 1, 4095, 2048)
 , invert(CONFIG_PARAM_PATH + "joints/" + _name + "/invert", false)
 , readFeedback(CONFIG_PARAM_PATH + "joints/" + _name + "/readFeedback", true)
 , realEffort(cmd.effort)
 , rawState(1.0)
{
	// Initialize variables
	name = _name;
	diag.name = name;
}

/**
 * RobotInterface constructor.
 **/
RobotInterface::RobotInterface()
 : m_model(NULL)
 , CONFIG_PARAM_PATH("nopInterface/")
 , m_haveHardware(false)
 , m_buttonPress0(CONFIG_PARAM_PATH + "button/pressButton0", false)
 , m_buttonPress1(CONFIG_PARAM_PATH + "button/pressButton1", false)
 , m_buttonPress2(CONFIG_PARAM_PATH + "button/pressButton2", false)
 , m_relaxed(true)
 , m_statIndex(0)
 , m_statVoltage(15.0)
 , m_attEstUseFusedMethod(CONFIG_PARAM_PATH + "attEstUseFusedMethod", true)
 , m_attEstKp(CONFIG_PARAM_PATH + "attEstPIGains/Kp", 0.05, 0.01, 30.0, 2.20)
 , m_attEstTi(CONFIG_PARAM_PATH + "attEstPIGains/Ti", 0.05, 0.01, 10.0, 2.65)
 , m_attEstKpQuick(CONFIG_PARAM_PATH + "attEstPIGains/KpQuick", 0.05, 0.01, 30.0, 10.0)
 , m_attEstTiQuick(CONFIG_PARAM_PATH + "attEstPIGains/TiQuick", 0.05, 0.01, 10.0, 1.25)
 , m_attEstMagCalibX(CONFIG_PARAM_PATH + "attEstMagCalib/x", -1.5, 0.01, 1.5, 1.0)
 , m_attEstMagCalibY(CONFIG_PARAM_PATH + "attEstMagCalib/y", -1.5, 0.01, 1.5, 0.0)
 , m_attEstMagCalibZ(CONFIG_PARAM_PATH + "attEstMagCalib/z", -1.5, 0.01, 1.5, 0.0)
 , m_useMagnetometer(CONFIG_PARAM_PATH + "useMagnetometer", false)
 , m_magSpikeMaxDelta(CONFIG_PARAM_PATH + "magSpikeMaxDelta", 0.0, 0.001, 0.20, 0.07)
 , m_magSpikeFilterX()
 , m_magSpikeFilterY()
 , m_magSpikeFilterZ()
 , m_magFirFilterX(FIRFilterType::FT_AVERAGE)
 , m_magFirFilterY(FIRFilterType::FT_AVERAGE)
 , m_magFirFilterZ(FIRFilterType::FT_AVERAGE)
 , m_magHardIronFilter(CONFIG_PARAM_PATH + "magFilter")
 , m_useModel(CONFIG_PARAM_PATH + "useModel", false)
 , m_effortSlopeLimit(CONFIG_PARAM_PATH + "effortSlopeLimit", 0.0, 0.01, 1.0, 0.02)
 , m_rawStateSlopeLimit(CONFIG_PARAM_PATH + "rawStateSlopeLimit", 0.0, 0.01, 1.0, 0.02)
 , m_PM(PM_COUNT)
 , m_plotRobotInterfaceData(CONFIG_PARAM_PATH + "plotData", false)
 , m_lastButtons(0)
 , m_act_fadeTorque("fade_torque")
 , m_totalFailCount(0)
 , m_consecFailCount(0)
 , m_commsSuspCount(0)
 , m_lastFailCount(0)
 , m_lastFailCountSev(0)
 , m_lastCommsSuspCount(0)
 , m_enableCommsSusp(true)
 , m_timeLastFailCount(0, 0)
 , m_timeLastFailCountSev(0, 0)
 , m_timeLastCommsSusp(0, 0)
 , m_timeLastSuspCheck(0, 0)
 , m_showServoFailures(CONFIG_PARAM_PATH + "showServoFailures", false)
 , m_skipStep(false)
 , m_cm730Suspend(false)
 , m_lastSensorTime(0, 0)
 , m_lastCM730Time(0, 0)
 , m_robPause("pause", false) // See main() in robotcontrol.cpp for the main use of this config variable
 , m_fir_accX(FIRFilterType::FT_AVERAGE)
 , m_fir_accY(FIRFilterType::FT_AVERAGE)
 , m_fir_accZ(FIRFilterType::FT_AVERAGE)
 , m_fadeTorqueClient("/robotcontrol/fade_torque")
 , m_fadingIsTriggered(false)
{
	// Retrieve the node handle
	ros::NodeHandle nh("~");

	// Initialize statistics timer
	m_statisticsTimer = nh.createTimer(ros::Duration(0.5), &RobotInterface::handleStatistics, this);
	m_statisticsTimer.stop();

	// Advertise topics
	m_pub_buttons = nh.advertise<nimbro_op_interface::Button>("/button", 1);
	m_pub_led_state = nh.advertise<nimbro_op_interface::LEDCommand>("/led_state", 1);

	// Subscribe topics
	m_sub_led = nh.subscribe("/led", 2, &RobotInterface::handleLEDCommand, this);

	// Initialise the board voltage
	m_boardData.voltage = (unsigned char)((15.0 / INT_TO_VOLTS) + 0.5); // Note: The 0.5 is for rounding purposes
	
	// Configure the magnetometer filtering
	m_magSpikeMaxDelta.setCallback(boost::bind(&RobotInterface::updateMagSpikeMaxDelta, this));
	updateMagSpikeMaxDelta();

	// Configure the attitude estimator
	boost::function<void()> updateCallbackME = boost::bind(&RobotInterface::updateAttEstMethod, this);
	m_attEstUseFusedMethod.setCallback(boost::bind(updateCallbackME));
	updateAttEstMethod(); // Calls m_attitudeEstimator.setAccMethod() internally
	boost::function<void()> updateCallbackPI = boost::bind(&RobotInterface::updateAttEstPIGains, this);
	m_attEstKp.setCallback(boost::bind(updateCallbackPI));
	m_attEstTi.setCallback(boost::bind(updateCallbackPI));
	m_attEstKpQuick.setCallback(boost::bind(updateCallbackPI));
	m_attEstTiQuick.setCallback(boost::bind(updateCallbackPI));
	updateAttEstPIGains(); // Calls m_attitudeEstimator.setPIGains() internally
	boost::function<void()> updateCallbackMC = boost::bind(&RobotInterface::updateAttEstMagCalib, this);
	m_attEstMagCalibX.setCallback(boost::bind(updateCallbackMC));
	m_attEstMagCalibY.setCallback(boost::bind(updateCallbackMC));
	m_attEstMagCalibZ.setCallback(boost::bind(updateCallbackMC));
	updateAttEstMagCalib(); // Calls m_attitudeEstimator.setMagCalib() internally
	
	// Reset the button press config parameters just in case
	m_buttonPress0.set(false);
	m_buttonPress1.set(false);
	m_buttonPress2.set(false);

	// Advertise provided services
	m_srv_readOffsets = nh.advertiseService(CONFIG_PARAM_PATH + "readOffsets", &RobotInterface::handleReadOffsets, this);
	m_srv_readOffset = nh.advertiseService(CONFIG_PARAM_PATH + "readOffset", &RobotInterface::handleReadOffset, this);
	m_srv_attEstCalibrate = nh.advertiseService(CONFIG_PARAM_PATH + "attEstCalibrate", &RobotInterface::handleAttEstCalibrate, this);

	// Configure the plot manager variable names
	m_PM.setName(PM_ANGEST_PPITCH, "Estimation/AngleEstimator/ProjPitch");
	m_PM.setName(PM_ANGEST_PROLL,  "Estimation/AngleEstimator/ProjRoll");
	m_PM.setName(PM_ATTEST_FYAW,   "Estimation/AttitudeEstimator/FusedYaw");
	m_PM.setName(PM_ATTEST_FPITCH, "Estimation/AttitudeEstimator/FusedPitch");
	m_PM.setName(PM_ATTEST_FROLL,  "Estimation/AttitudeEstimator/FusedRoll");
	m_PM.setName(PM_ATTEST_FHEMI,  "Estimation/AttitudeEstimator/FusedHemi");
	m_PM.setName(PM_ATTEST_BIAS_X, "Estimation/AttitudeEstimator/GyroBiasX");
	m_PM.setName(PM_ATTEST_BIAS_Y, "Estimation/AttitudeEstimator/GyroBiasY");
	m_PM.setName(PM_ATTEST_BIAS_Z, "Estimation/AttitudeEstimator/GyroBiasZ");
	m_PM.setName(PM_ATTEST_NOMAG_FYAW,   "Estimation/AttitudeEstimatorNoMag/FusedYaw");
	m_PM.setName(PM_ATTEST_NOMAG_FPITCH, "Estimation/AttitudeEstimatorNoMag/FusedPitch");
	m_PM.setName(PM_ATTEST_NOMAG_FROLL,  "Estimation/AttitudeEstimatorNoMag/FusedRoll");
	m_PM.setName(PM_ATTEST_NOMAG_FHEMI,  "Estimation/AttitudeEstimatorNoMag/FusedHemi");
	m_PM.setName(PM_ATTEST_NOMAG_BIAS_X, "Estimation/AttitudeEstimatorNoMag/GyroBiasX");
	m_PM.setName(PM_ATTEST_NOMAG_BIAS_Y, "Estimation/AttitudeEstimatorNoMag/GyroBiasY");
	m_PM.setName(PM_ATTEST_NOMAG_BIAS_Z, "Estimation/AttitudeEstimatorNoMag/GyroBiasZ");
	m_PM.setName(PM_GYRO_X,     "Estimation/Gyro/x");
	m_PM.setName(PM_GYRO_Y,     "Estimation/Gyro/y");
	m_PM.setName(PM_GYRO_Z,     "Estimation/Gyro/z");
	m_PM.setName(PM_GYRO_N,     "Estimation/Gyro/norm");
	m_PM.setName(PM_ACC_XRAW,   "Estimation/Acc/Raw x");
	m_PM.setName(PM_ACC_YRAW,   "Estimation/Acc/Raw y");
	m_PM.setName(PM_ACC_ZRAW,   "Estimation/Acc/Raw z");
	m_PM.setName(PM_ACC_NRAW,   "Estimation/Acc/Raw norm");
	m_PM.setName(PM_ACC_X,      "Estimation/Acc/x");
	m_PM.setName(PM_ACC_Y,      "Estimation/Acc/y");
	m_PM.setName(PM_ACC_Z,      "Estimation/Acc/z");
	m_PM.setName(PM_ACC_N,      "Estimation/Acc/norm");
	m_PM.setName(PM_MAG_XRAW,   "Estimation/Mag/Raw x");
	m_PM.setName(PM_MAG_YRAW,   "Estimation/Mag/Raw y");
	m_PM.setName(PM_MAG_ZRAW,   "Estimation/Mag/Raw z");
	m_PM.setName(PM_MAG_XSPIKE, "Estimation/Mag/No spike x");
	m_PM.setName(PM_MAG_YSPIKE, "Estimation/Mag/No spike y");
	m_PM.setName(PM_MAG_ZSPIKE, "Estimation/Mag/No spike z");
	m_PM.setName(PM_MAG_XIRON,  "Estimation/Mag/Biased x");
	m_PM.setName(PM_MAG_YIRON,  "Estimation/Mag/Biased y");
	m_PM.setName(PM_MAG_ZIRON,  "Estimation/Mag/Biased z");
	m_PM.setName(PM_MAG_X,      "Estimation/Mag/Filt x");
	m_PM.setName(PM_MAG_Y,      "Estimation/Mag/Filt y");
	m_PM.setName(PM_MAG_Z,      "Estimation/Mag/Filt z");
	m_PM.setName(PM_MAG_N,      "Estimation/Mag/Filt norm");
	m_PM.setName(PM_SENSOR_DT,  "Estimation/Sensor dT");
}

/**
 * RobotInterface destructor.
 **/
RobotInterface::~RobotInterface()
{
}

/**
 * Factory method for DXLJoint structs.
 **/
boost::shared_ptr<Joint> RobotInterface::createJoint(const std::string& name)
{
	// Construct a DXL joint of the desired name
	boost::shared_ptr<DXLJoint> joint = boost::make_shared<DXLJoint>(name);

	// Wrap the RobotInterface updateJointSettings() function to specifically update this DXLJoint
	boost::function<void()> updateCb = boost::bind(&RobotInterface::updateJointSettings, this, joint.get());

	// If the config parameters change, call the update callback automatically
	joint->type.setCallback(boost::bind(updateCb));
	joint->readFeedback.setCallback(boost::bind(updateCb));

	// Initialize the joint parameters
	joint->commandGenerator = commandGenerator(joint->type());
	joint->voltage = (int)((15.0 / INT_TO_VOLTS) + 0.5); // Note: The 0.5 is for rounding purposes
	joint->temperature = 0;

	// Call the update callback once to begin with
	updateJointSettings(joint.get());

	// Return the constructed joint
	return joint;
}

/**
 * @return pointer to the DXLJoint corresponding to the given servo @p id,
 *    0 if not found.
 **/
RobotInterface::DXLJoint* RobotInterface::dxlJointForID(int id)
{
	// Search for the given ID in the RobotModel joint array
	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		DXLJoint* joint = dxlJoint(i);
		if(joint->id() == id)
			return joint;
	}

	// Return a null pointer if the ID isn't found
	return 0;
}

/**
 * Update the settings for a joint based on possible changes in config
 * server parameters (e.g. @c type, @c id, @c readFeedback).
 **/
void RobotInterface::updateJointSettings(RobotInterface::DXLJoint* joint)
{
	// The only thing that is not polled in every iteration is the command
	// generator, so update it here depending on the joint type.
	joint->commandGenerator = commandGenerator(joint->type());

	// Find the joint by id in the list of servos for the CM730 to query
	std::vector<int>::iterator it = std::find(
		m_cm730_queryset.begin(), m_cm730_queryset.end(), joint->id()
	);

	// Add the joint to the list of servos for the CM730 to query, or remove it.
	if(joint->readFeedback())
	{
		// Currently not in queryset => Append it
		if(it == m_cm730_queryset.end())
			m_cm730_queryset.push_back(joint->id());
	}
	else
	{
		// Currently in queryset => Remove it
		if(it != m_cm730_queryset.end())
			m_cm730_queryset.erase(it);
	}

	// Update the bulk read packet to reflect the changes to the query set
	if(m_board) m_board->updateTxBRPacket(m_cm730_queryset);
}

/**
 * Initialize the robot interface and connect to the robot.
 * It is assumed that `createJoint()` has already been called for all our joints.
 **/
bool RobotInterface::init(RobotModel* model)
{
	// Save the pointer to the passed RobotModel
	m_model = model;

	// Get a state reference for the relaxed state
	m_state_relaxed = m_model->registerState("relaxed");

	// Find the largest servo ID in the RobotModel joint struct array (needed to set the size of m_servoData below)
	int max_addr = 0;
	for(size_t i = 0; i < m_model->numJoints(); i++)
	{
		const DXLJoint* joint = dxlJoint(i);
		if(joint->id() <= 0)
		{
			ROS_ERROR("Invalid joint id %d for joint '%s', ignoring this joint for now", joint->id(), joint->name.c_str());
			continue;
		}
		if(joint->id() > max_addr)
			max_addr = joint->id();
	}
	m_servoData.resize(max_addr);

	// Sort the queryset (nice for debugging)
	std::sort(m_cm730_queryset.begin(), m_cm730_queryset.end());

	// Reset the golay filters
	m_fir_accX.setBuf(0.0);
	m_fir_accY.setBuf(0.0);
	m_fir_accZ.setBuf(9.81);

	// Initialise the LED command (RGB5 gets overridden anyway on the CM730 to display the USB connection state)
	m_ledCommand.mask = 0x07;
	m_ledCommand.state = 0x00;
	m_ledCommand.rgb5.r = 0.0;
	m_ledCommand.rgb5.g = 1.0;
	m_ledCommand.rgb5.b = 0.0;
	m_ledCommand.rgb6.r = 0.0;
	m_ledCommand.rgb6.g = 0.0;
	m_ledCommand.rgb6.b = 1.0;

	// Attempt to initialise the CM730
	if(!initCM730())
		return false;

	// Start the statistics timer for periodic reporting
	m_statisticsTimer.start();

	// Return success
	return true;
}

// Initialise the CM730 board
bool RobotInterface::initCM730()
{
	// Initialise the CM730
	m_board.reset(new CM730);
	m_board->updateTxBRPacket(m_cm730_queryset);

	// Stop here if connecting to the CM730 was unsuccessful
	if(m_board->connect() < 0)
	{
		ROS_ERROR("Could not initialize CM730");
		return false;
	}

	// Enable power for the arms (there is a power MOSFET on the CM730 board that controls this)
	m_board->setDynamixelPower(CM730::DYNPOW_ON);

	// Indicate that we have real hardware at our disposal
	m_haveHardware = true;

	// Return that the CM730 was successfully initialised
	return true;
}

// Function to set the feedback timestamps in the joint structs
void RobotInterface::setJointFeedbackTime(const ros::Time stamp)
{
	// Write to each of the joint struct feedback timestamp members
	for(size_t i = 0; i < m_model->numJoints(); i++)
		dxlJoint(i)->feedback.stamp = stamp;
}

/**
 * Read joint states and other data from the CM730.
 **/
bool RobotInterface::readJointStates()
{
	// Our initial intentions are to do a full bulk read, not just query the CM730 for its registers (and thereby leave the electrical dynamixel bus void of packets)
	bool onlyTryCM730 = false;
	
	// Skip one step if requested (helps to establish stable communication again after RX_CORRUPT or RX_TIMEOUT)
	if(m_skipStep)
	{
		m_skipStep = false;
		onlyTryCM730 = true;
	}

	// Don't bother trying to read anything if the CM730 is suspended
	if(m_haveHardware && m_board->isSuspended())
	{
		m_cm730Suspend = true;
		onlyTryCM730 = true;
	}
	else
	{
		if(m_cm730Suspend)
			ROS_ERROR("End of suspended servo comms!");
		m_cm730Suspend = false;
	}

	//
	// Bulk read from CM730
	//

	// Read feedback data from the CM730 and servos
	int ret = readFeedbackData(onlyTryCM730); // Should update m_servoData and m_boardData (only the latter however if onlyTryCM730 is true)
	ros::Time bulkReadTime = ros::Time::now();

	// If the bulk read failed then see what the reason was and react
	if(ret == CM730::RET_SUCCESS)
	{
		// Reset the consecutive failure count
		if(!onlyTryCM730)
			m_consecFailCount = 0;
	}
	else if(onlyTryCM730)
	{
		// Update the joint state timestamps to the current time in case we're about to return early from this function before we get a chance to do so (resolves joint state plotting timestamp anomalies)
		setJointFeedbackTime(bulkReadTime);
		
		// Print error messages specific to the type of read failure
		if(ret == CM730::RET_RX_CORRUPT) ROS_ERROR_THROTTLE(0.1, "CM730 read failed: RX_CORRUPT");
		else if(ret == CM730::RET_RX_FAIL) ROS_ERROR_THROTTLE(0.1, "CM730 read failed: RX_FAIL");
		else if(ret == CM730::RET_TX_FAIL) ROS_ERROR_THROTTLE(0.1, "CM730 read failed: TX_FAIL");
		else if(ret == CM730::RET_RX_TIMEOUT) ROS_ERROR_THROTTLE(0.1, "Read timeout of CM730!");
		else ROS_ERROR_THROTTLE(0.1, "CM730 read failed: Unknown error (%d)", ret);
		
		// Return as we have no data to process
		return false;
	}
	else
	{
		// Update the joint state timestamps to the current time in case we're about to return early from this function before we get a chance to do so (resolves joint state plotting timestamp anomalies)
		setJointFeedbackTime(bulkReadTime);

		// Increment the consecutive failure count
		m_consecFailCount++;

		// Retrieve the last failed ID
		int id = m_board->lastFailedID();    // Last failed ID is zero if failure was due to some reason other than servo comms
		DXLJoint* joint = dxlJointForID(id); // Note: Protect uses of this with 'if(joint){...}'

		// Process the servo failing
		if(id != 0 && id != CM730::ID_CM730)
		{
			// Retrieve the number of times that the last failed ID has now failed
			int failCount = m_board->servoFailCount(id);
			if(m_showServoFailures())
				ROS_WARN("ID %d failed => %d times now", id, failCount);

			// Increment the total servo fail count
			m_totalFailCount++;
			if(m_showServoFailures())
				ROS_ERROR_THROTTLE(0.5, "Total servo failure count at %d", m_totalFailCount);

			// Handle servo death cycles
#if PROD_DEATH_CYCLES
			if((bulkReadTime - m_timeLastFailCount).toSec() >= DEATH_CYCLE_TIME)
			{
				m_lastFailCount = m_totalFailCount - 1;
				m_timeLastFailCount = bulkReadTime;
			}
			if((m_totalFailCount - m_lastFailCount >= DEATH_CYCLE_COUNT) || (m_consecFailCount % CONSEC_CYCLE_COUNT == 0))
			{
#if PROD_DEATH_CYCLES_PREV
				int previd = 0;
				std::vector<int>::iterator it = std::find(m_cm730_queryset.begin(), m_cm730_queryset.end(), id);
				if(it != m_cm730_queryset.end() && it != m_cm730_queryset.begin()) // Our ID was found in the array and is not the first element...
					previd = *(--it);
				if(previd == CM730::ID_CM730 || previd < 1 || previd >= 0xFE)
					previd = 0;
				if((m_consecFailCount % CONSEC_CYCLE_COUNT) % 2 == 1 && previd > 0)
				{
					if(m_showServoFailures())
						ROS_ERROR("Servo death cycle (ID %d) => Prodding preceding servo ID %d with a write!", id, previd);
					m_board->writeByte(previd, DynamixelMX::P_ALARM_LED, 0x24); // Note: 0x24 is the default value of this register, and corresponds to the overheating and overload error bits (if these errors occur the servo LED lights up)
				}
				else
#endif
				{
					if(m_showServoFailures())
						ROS_ERROR("Servo death cycle (ID %d) => Prodding servo with a write!", id);
					m_board->writeByte(id, DynamixelMX::P_ALARM_LED, 0x24); // Note: 0x24 is the default value of this register, and corresponds to the overheating and overload error bits (if these errors occur the servo LED lights up)
				}
			}
#endif /* PROD_DEATH_CYCLES */

			// Handle severe servo death cycles
#if RESET_DEATH_CYCLES
			if((bulkReadTime - m_timeLastFailCountSev).toSec() >= SEV_DEATH_CYCLE_TIME)
			{
				m_lastFailCountSev = m_totalFailCount - 1;
				m_timeLastFailCountSev = bulkReadTime;
			}
			if((m_totalFailCount - m_lastFailCountSev >= SEV_DEATH_CYCLE_COUNT) || (m_consecFailCount % SEV_CONSEC_CYCLE_COUNT == 0))
			{
				m_commsSuspCount++;
				if(!m_enableCommsSusp && (bulkReadTime - m_timeLastCommsSusp).toSec() >= COMMS_SUSPEND_RECOV_TIME)
					m_enableCommsSusp = true;
				if((bulkReadTime - m_timeLastSuspCheck).toSec() >= COMMS_SUSPEND_CHECK_TIME)
				{
					m_lastCommsSuspCount = m_commsSuspCount - 1;
					m_timeLastSuspCheck = bulkReadTime;
				}
				if(m_commsSuspCount - m_lastCommsSuspCount >= COMMS_SUSPEND_CHECK_COUNT)
				{
					m_enableCommsSusp = false;
				}
				if(m_enableCommsSusp)
				{
					ROS_ERROR("Severe servo death cycle (ID %d) => Suspending all servo comms for %.0fms!", id, 1000.0*SEV_COMMS_SUSPEND_TIME);
					if(m_showServoFailures())
						ROS_ERROR("Total severe servo death cycle count at %d", m_commsSuspCount);
					m_board->suspend(SEV_COMMS_SUSPEND_TIME);
				}
				else
				{
					ROS_ERROR_THROTTLE(2.0, "Severe servo death cycle (ID %d) => NOT suspending servo comms as this is happening too often!", id);
					if(m_showServoFailures())
						ROS_ERROR_THROTTLE(2.0, "Total severe servo death cycle count at %d", m_commsSuspCount);
				}
				m_timeLastCommsSusp = bulkReadTime;
			}
#endif /* RESET_DEATH_CYCLES */
			
			// Handle non-responding servo repeat-offenders
#if DYNAMIC_BRPACKET_REORDER
			if(failCount % BR_REORDER_FAIL_COUNT == 0)
			{
				// Display a message to the user
				if(m_showServoFailures())
					ROS_WARN("Dynamic servo reordering triggered on ID %d => Prodding servo with a write!", id);
				
				// Prod the servo that has been moved to the back of the list to avoid the subsequent servo in the bulk read packet (before re-ordering) failing in the next bulk read and never receiving a packet from the reordered servo because failure always happens before it's the reordered servo's turn
				m_board->writeByte(id, DynamixelMX::P_ALARM_LED, 0x24); // Note: 0x24 is the default value of this register, and corresponds to the overheating and overload error bits (if these errors occur the servo LED lights up)

				// Move the ID to the back of the query set
				std::vector<int>::iterator it = std::find(m_cm730_queryset.begin(), m_cm730_queryset.end(), id);
				if(it != m_cm730_queryset.end()) // Our ID was found in the array...
				{
					if(m_cm730_queryset.end() - it > 1) // Our ID isn't at the back of the array already...
					{
						m_cm730_queryset.erase(it);
						m_cm730_queryset.push_back(id);
						m_board->updateTxBRPacket(m_cm730_queryset);
					}
				}

				// Display the new query set
				if(m_showServoFailures())
				{
					std::stringstream ss("Query set:");
					BOOST_FOREACH(int tmp, m_cm730_queryset) { ss << " " << tmp; }
					ROS_INFO("%s", ss.str().c_str());
				}
			}
#endif
		}

		// Perform handling specific to the type of bulk read failure
		switch(ret)
		{
			case CM730::RET_RX_CORRUPT:
			{
				ROS_ERROR_THROTTLE(0.1, "Bulk read failed: RX_CORRUPT");
				m_skipStep = true;
				if(joint) joint->diag.checksum_errors++;
				return false;
				break;
			}
			case CM730::RET_RX_FAIL:
				ROS_ERROR_THROTTLE(0.1, "Bulk read failed: RX_FAIL");
				return false;
				break;
			case CM730::RET_TX_FAIL:
				ROS_ERROR_THROTTLE(0.1, "Bulk read failed: TX_FAIL");
				return false;
				break;
			case CM730::RET_RX_TIMEOUT:
			{
				m_skipStep = true;
				if(id == 0)
					ROS_ERROR_THROTTLE(0.1, "Bulk read timeout of unknown ID!");
				else if(id == CM730::ID_CM730)
				{
					if(m_showServoFailures())
						ROS_ERROR_THROTTLE(0.1, "Bulk read timeout of CM730!");
				}
				else
				{
					if(joint)
						joint->diag.timeouts++;
				}
				break;
			}
			default:
				ROS_ERROR_THROTTLE(0.1, "Bulk read failed: Unknown error (%d)", ret);
				return false;
		}
	}

	//
	// Servo/joint feedback
	//

	// Write everything into the joint structs
	if(onlyTryCM730)
		setJointFeedbackTime(bulkReadTime);
	else
	{
		for(size_t i = 0; i < m_model->numJoints(); i++)
		{
			// Retrieve pointers to joint and bulk read data
			DXLJoint* joint = dxlJoint(i);
			int idx = joint->id() - 1;
			const BRData& data = m_servoData[idx];

			// Set the feedback time stamp
			joint->feedback.stamp = bulkReadTime; // Note: This line replaces the need to call setJointFeedbackTime()

			// Write either the real feedback or just the commanded values into the joint, depending on the readFeedback config parameter
			if(joint->readFeedback())
			{
				// Convert the feedback to an angular position
				if(data.position >= 0) // Valid position data is always non-negative in terms of ticks
					joint->feedback.pos = (data.position - joint->tickOffset()) / TICKS_PER_RAD;
				else
					joint->feedback.pos = 0.0;

				// Invert the joint position if required
				if(joint->invert())
					joint->feedback.pos = -joint->feedback.pos;

				// Give the command generator the information it needs
				int pValue = joint->realEffort * FULL_EFFORT;
				if(pValue < 2) pValue = 2;
				joint->commandGenerator->setPValue(pValue);
				joint->commandGenerator->setVoltage(INT_TO_VOLTS * m_boardData.voltage); // Note: This could use m_statVoltage, but as statistics are disabled this is a decent drop-in replacement

				// Estimate the produced torque using the position displacement
				joint->feedback.torque = joint->commandGenerator->servoTorqueFromCommand(joint->cmd.rawPos, joint->feedback.pos, joint->cmd.vel);
			}
			else
			{
				// Produce simulated feedback based on the commanded values
				joint->feedback.pos = joint->cmd.pos;
				joint->feedback.torque = joint->feedback.modelTorque;
			}
		}
	}

	//
	// Timing
	//
	
	// Retrieve the nominal time step
	const double nominaldT = m_model->timerDuration();

	// Calculate the time since sensor data was last processed
	double dT;
	if(m_lastSensorTime.isZero())
		dT = nominaldT;
	else
	{
		dT = (bulkReadTime - m_lastSensorTime).toSec();
		if(dT < MIN_SENSOR_DT * nominaldT)
			dT = MIN_SENSOR_DT * nominaldT;
		else if(dT > MAX_SENSOR_DT * nominaldT)
			dT = MAX_SENSOR_DT * nominaldT;
	}
	m_lastSensorTime = bulkReadTime;
	
	// Reset certain data if no data has been received from the CM730 for a while
	bool firstCM730Data = m_lastCM730Time.isZero();
	double cm730dT = (firstCM730Data ? nominaldT : (bulkReadTime - m_lastCM730Time).toSec());
	if(!m_haveHardware || m_board->gotCM730Data()) m_lastCM730Time = bulkReadTime;
	if(cm730dT > MAX_CM730_DT * nominaldT)
	{
		m_boardData.gyroX = 0.0;
		m_boardData.gyroY = 0.0;
		m_boardData.gyroZ = 0.0;
	}

	//
	// Gyroscope sensor
	//

	// Retrieve and correct the gyroscope data to be in rad/s
	double gyroX = m_boardData.gyroX * GYRO_SCALE;
	double gyroY = m_boardData.gyroY * GYRO_SCALE;
	double gyroZ = m_boardData.gyroZ * GYRO_SCALE;

	// Update the robot angular velocity in the robot model
	m_model->setRobotAngularVelocity(Eigen::Vector3d(gyroX, gyroY, gyroZ));

	//
	// Accelerometer sensor
	//

	// Retrieve and scale the accelerometer data to be in m/s^2 (inertial acceleration convention, so nominal is (0,0,9.81))
	double rawAccX = m_boardData.accX * ACC_SCALE;
	double rawAccY = m_boardData.accY * ACC_SCALE;
	double rawAccZ = m_boardData.accZ * ACC_SCALE;

	// Filter the accelerometer data
	m_fir_accX.put(rawAccX);
	m_fir_accY.put(rawAccY);
	m_fir_accZ.put(rawAccZ);
	double accX = m_fir_accX.value();
	double accY = m_fir_accY.value();
	double accZ = m_fir_accZ.value();

	// Update the measured acceleration vector in the robot model
	m_model->setAccelerationVector(Eigen::Vector3d(accX, accY, accZ));

	//
	// Magnetometer sensor
	//

	// Retrieve and scale the magnetometer data to be in gauss
	double magX = m_boardData.magX * MAG_SCALE;
	double magY = m_boardData.magY * MAG_SCALE;
	double magZ = m_boardData.magZ * MAG_SCALE;

	// Initialise the spike filter values if this is the first time we are getting data
	if(firstCM730Data)
	{
		m_magSpikeFilterX.setValue(magX);
		m_magSpikeFilterY.setValue(magY);
		m_magSpikeFilterZ.setValue(magZ);
	}

	// Pass spike-filtered magnetometer data through the magnetometer filter to account for hard iron effects
	m_magHardIronFilter.update(m_magSpikeFilterX.put(magX), m_magSpikeFilterY.put(magY), m_magSpikeFilterZ.put(magZ));

	// Apply an averaging FIR filter to the corrected magnetometer measurements
	m_magFirFilterX.put(m_magHardIronFilter.valueX());
	m_magFirFilterY.put(m_magHardIronFilter.valueY());
	m_magFirFilterZ.put(m_magHardIronFilter.valueZ());
	Eigen::Vector3d mag(m_magFirFilterX.value(), m_magFirFilterY.value(), m_magFirFilterZ.value());

	// Zero out the magnetometer measurements if they should not be used
	if(!m_useMagnetometer()) mag.setZero();

	// Make available the corrected magnetometer data
	m_model->setMagneticFieldVector(mag);

	//
	// Attitude estimation
	//

	// Update the attitude estimator
	m_attitudeEstimator.update(dT, gyroX, gyroY, gyroZ, accX, accY, accZ, mag.x(), mag.y(), mag.z()); // We use the filtered magnetometer values here...
	m_attEstNoMag.update(dT, gyroX, gyroY, gyroZ, accX, accY, accZ, 0.0, 0.0, 0.0);                   // We use no magnetometer values at all here...

	// Retrieve the current robot orientation (attitude) estimate
	double quatAtt[4], quatAttNoMag[4]; // Format is (w,x,y,z)
	m_attitudeEstimator.getAttitude(quatAtt);
	m_attEstNoMag.getAttitude(quatAttNoMag);

	// Make available the robot orientation estimates
	m_model->setRobotOrientation(Eigen::Quaterniond(quatAtt[0], quatAtt[1], quatAtt[2], quatAtt[3]), Eigen::Quaterniond(quatAttNoMag[0], quatAttNoMag[1], quatAttNoMag[2], quatAttNoMag[3]));

	//
	// Fused angle estimation
	//

	// Perform the prediction and correction cycles of the angle estimator
	m_angleEstimator.predict(dT, gyroX, gyroY);
	m_angleEstimator.update(accX, accY, accZ); // Note: After this line the fused angle can be retrieved using m_angleEstimator.projPitch() and m_angleEstimator.projRoll()

	//
	// Button presses
	//

	// Check which buttons have been pressed and publish appropriate messages
	for(int i = 0; i <= 2; i++)
	{
		// Determine the old and new button states
		bool state = (m_boardData.button & (1 << i));
		bool lastState = m_lastButtons & (1 << i);
		
		// Determine whether this button has been fake pressed by the config server
		bool fakePressed0 = (i == 0 && m_buttonPress0());
		bool fakePressed1 = (i == 1 && m_buttonPress1());
		bool fakePressed2 = (i == 2 && m_buttonPress2());

		// If the button was just released...
		if((!state && lastState) || fakePressed0 || fakePressed1 || fakePressed2)
		{
			// Publish on a ROS topic that the button was pressed
			Button btn;
			btn.button = i;
			btn.time = bulkReadTime;
			m_pub_buttons.publish(btn);

			// We react to some buttons directly ourselves
			handleButton(i);
			
			// Reset button press config variable
			if(fakePressed0) m_buttonPress0.set(false);
			if(fakePressed1) m_buttonPress1.set(false);
			if(fakePressed2) m_buttonPress2.set(false);
		}
	}

	// Save the new button states
	m_lastButtons = m_boardData.button;

	//
	// Plotting
	//

	// Plot sensory and estimation data if requested
	if(m_plotRobotInterfaceData())
	{
		m_PM.clear(bulkReadTime);
		m_PM.plotScalar(m_angleEstimator.projPitch(), PM_ANGEST_PPITCH);
		m_PM.plotScalar(m_angleEstimator.projRoll(), PM_ANGEST_PROLL);
		m_PM.plotScalar(m_attitudeEstimator.fusedYaw(), PM_ATTEST_FYAW);
		m_PM.plotScalar(m_attitudeEstimator.fusedPitch(), PM_ATTEST_FPITCH);
		m_PM.plotScalar(m_attitudeEstimator.fusedRoll(), PM_ATTEST_FROLL);
		m_PM.plotScalar((m_attitudeEstimator.fusedHemi() ? 1.0 : -1.0), PM_ATTEST_FHEMI);
		double attEstBias[3] = {0.0};
		m_attitudeEstimator.getGyroBias(attEstBias);
		m_PM.plotScalar(attEstBias[0], PM_ATTEST_BIAS_X);
		m_PM.plotScalar(attEstBias[1], PM_ATTEST_BIAS_Y);
		m_PM.plotScalar(attEstBias[2], PM_ATTEST_BIAS_Z);
		m_PM.plotScalar(m_attEstNoMag.fusedYaw(), PM_ATTEST_NOMAG_FYAW);
		m_PM.plotScalar(m_attEstNoMag.fusedPitch(), PM_ATTEST_NOMAG_FPITCH);
		m_PM.plotScalar(m_attEstNoMag.fusedRoll(), PM_ATTEST_NOMAG_FROLL);
		m_PM.plotScalar((m_attEstNoMag.fusedHemi() ? 1.0 : -1.0), PM_ATTEST_NOMAG_FHEMI);
		double attEstNoMagBias[3] = {0.0};
		m_attitudeEstimator.getGyroBias(attEstNoMagBias);
		m_PM.plotScalar(attEstNoMagBias[0], PM_ATTEST_NOMAG_BIAS_X);
		m_PM.plotScalar(attEstNoMagBias[1], PM_ATTEST_NOMAG_BIAS_Y);
		m_PM.plotScalar(attEstNoMagBias[2], PM_ATTEST_NOMAG_BIAS_Z);
		m_PM.plotScalar(gyroX, PM_GYRO_X);
		m_PM.plotScalar(gyroY, PM_GYRO_Y);
		m_PM.plotScalar(gyroZ, PM_GYRO_Z);
		m_PM.plotScalar(sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ), PM_GYRO_N);
		m_PM.plotScalar(rawAccX, PM_ACC_XRAW);
		m_PM.plotScalar(rawAccY, PM_ACC_YRAW);
		m_PM.plotScalar(rawAccZ, PM_ACC_ZRAW);
		m_PM.plotScalar(sqrt(rawAccX*rawAccX + rawAccY*rawAccY + rawAccZ*rawAccZ), PM_ACC_NRAW);
		m_PM.plotScalar(accX, PM_ACC_X);
		m_PM.plotScalar(accY, PM_ACC_Y);
		m_PM.plotScalar(accZ, PM_ACC_Z);
		m_PM.plotScalar(sqrt(accX*accX + accY*accY + accZ*accZ), PM_ACC_N);
		m_PM.plotScalar(magX, PM_MAG_XRAW);
		m_PM.plotScalar(magY, PM_MAG_YRAW);
		m_PM.plotScalar(magZ, PM_MAG_ZRAW);
		m_PM.plotScalar(m_magSpikeFilterX.value(), PM_MAG_XSPIKE);
		m_PM.plotScalar(m_magSpikeFilterY.value(), PM_MAG_YSPIKE);
		m_PM.plotScalar(m_magSpikeFilterZ.value(), PM_MAG_ZSPIKE);
		m_PM.plotScalar(m_magHardIronFilter.valueX(), PM_MAG_XIRON);
		m_PM.plotScalar(m_magHardIronFilter.valueY(), PM_MAG_YIRON);
		m_PM.plotScalar(m_magHardIronFilter.valueZ(), PM_MAG_ZIRON);
		m_PM.plotScalar(mag.x(), PM_MAG_X);
		m_PM.plotScalar(mag.y(), PM_MAG_Y);
		m_PM.plotScalar(mag.z(), PM_MAG_Z);
		m_PM.plotScalar(mag.norm(), PM_MAG_N);
		m_PM.plotScalar(dT, PM_SENSOR_DT);
		m_PM.publish();
	}

	// Return success
	return true;
}

// Read feedback data from the CM730 and servos (usually involves a bulk read, but if specified, only the CM730 should queried)
int RobotInterface::readFeedbackData(bool onlyTryCM730)
{
	// Perform the required read
	if(onlyTryCM730)
		return m_board->readCM730(&m_boardData);
	else
		return m_board->bulkRead(&m_servoData, &m_boardData);
}

/**
 * Get command generator for a given servo type.
 **/
CommandGeneratorPtr RobotInterface::commandGenerator(const std::string& type)
{
	// Return the existing generator if one already exists for this type
	if(m_generators.count(type) != 0)
		return m_generators[type];

	// Create a new command generator for this type based on the associated model parameters on the config server
	// Note: Currently all types use the DynamicCommandGenerator, but with varying internal coefficients
	CommandGeneratorPtr gen;
	gen.reset(new DynamicCommandGenerator(CONFIG_PARAM_PATH + "models/" + type + "/"));

	// Save the new command generator into our generators map for next time
	m_generators[type] = gen;

	// Return the required command generator
	return gen;
}

/**
 * Callback that processes commands for the LED display.
 **/
void RobotInterface::handleLEDCommand(const LEDCommand& cmd)
{
	// Update the individual LED states
	for(int i = 0; i < 3; i++)
	{
		// Only update the LED if the corresponding bit is set
		if(cmd.mask & (1 << i))
		{
			// Set/reset the bit so it matches the bit in m_ledCommand.state
			if(cmd.state & (1 << i))
				m_ledCommand.state |= (1 << i);
			else
				m_ledCommand.state &= ~(1 << i);
		}
	}

	// Update the RGB LED colors if requested
// 	if(cmd.mask & LEDCommand::LED5) m_ledCommand.rgb5 = cmd.rgb5; // Note: RGBLED5 is set by the CM730 firmware and will always be green as long as the PC is connected
	if(cmd.mask & LEDCommand::LED6) m_ledCommand.rgb6 = cmd.rgb6;
}

/**
 * @internal Perform a CM730 LED command write.
 **/
void RobotInterface::sendCM730LedCommand()
{
	// Only send the LED command to the CM730 if we have hardware
	if(m_haveHardware)
	{
		// Allocate memory for the packet data and map the CM730LedWriteData struct onto it
		std::vector<uint8_t> params(sizeof(CM730LedWriteData));
		CM730LedWriteData* paramData = (CM730LedWriteData*)&params[0];

		// Set thie ID of the target CM730
		paramData->id = CM730::ID_CM730;

		// Transcribe the current LED commands (on/off) to the paramData struct
		// LED_MANAGE => 0x01, LED_EDIT => 0x02, LED_PLAY => 0x04 (see 'cm730/firmware/CM730_HW/inc/led.h')
		paramData->led_panel = m_ledCommand.state & 0x07;

		// Set the desired colors of the RGB LEDs
		// The 16-bit rgbled value is <0-4> = Red, <5-9> = Green, <10-14> = Blue, <15> = 0
		paramData->rgbled5 =
			(((int)(m_ledCommand.rgb5.r * 31) & 0x1F) << 0)
			| (((int)(m_ledCommand.rgb5.g * 31) & 0x1F) << 5)
			| (((int)(m_ledCommand.rgb5.b * 31) & 0x1F) << 10);
		paramData->rgbled6 =
			(((int)(m_ledCommand.rgb6.r * 31) & 0x1F) << 0)
			| (((int)(m_ledCommand.rgb6.g * 31) & 0x1F) << 5)
			| (((int)(m_ledCommand.rgb6.b * 31) & 0x1F) << 10);

		// Perform a write of the LED data to the CM730
		if(m_board->writeData(paramData->id, CM730::P_LED_PANEL, &params[1], sizeof(CM730LedWriteData)-1) != CM730::RET_SUCCESS) // Note: We use &params[1] as we wish to skip the first byte that just contains the target ID
			ROS_ERROR("Write of LED command to CM730 failed!");
	}
	
	// Publish the current LED state on a topic
	nimbro_op_interface::LEDCommand ledState = m_ledCommand;
	ledState.mask = 0x7F; // MNG => 0x01, EDIT => 0x02, PLAY => 0x04, RGBLED5 => 0x08, RGBLED6 => 0x10, RX => 0x20, TX => 0x40
	ledState.state |= LEDCommand::LED0 | LEDCommand::LED1; // TODO: Just turn RX/TX LEDs on for now (can later be simulated based on what we know we're sending/receiving)
	ledState.state &= ledState.mask;
	m_pub_led_state.publish(ledState);
}

/**
 * @internal Perform a servo command sync write (via the CM730).
 **/
bool RobotInterface::sendJointTargets()
{
	// Allocate memory for the packet data and map an array of JointCmdSyncWriteData structs onto it
	std::vector<uint8_t> params(m_model->numJoints() * sizeof(JointCmdSyncWriteData));
	JointCmdSyncWriteData* paramData = (JointCmdSyncWriteData*) &params[0];

	// Fill in the sync write buffer with the servo commands
	for(size_t i = 0; i < m_model->numJoints(); i++)
	{
		// Retrieve a pointer to the corresponding DXL joint
		DXLJoint* joint = dxlJoint(i);

		// Retrieve a pointer to the required JointCmdSyncWriteData in the array
		JointCmdSyncWriteData* data = &paramData[i];

		// Slope-limit the servo effort to avoid large instantaneous steps
		joint->realEffort = slopeLimited<double>(joint->realEffort, joint->cmd.effort, m_effortSlopeLimit());
		if(joint->realEffort < 0.0) joint->realEffort = 0.0;
		else if(joint->realEffort > 4.0) joint->realEffort = 4.0;

		// Convert effort to a P gain of the servos
		uint8_t realpValue = (uint8_t)(joint->realEffort * FULL_EFFORT);

		// Give the command generator the information it needs
		int pValue = realpValue;
		if(pValue < 2) pValue = 2;
		joint->commandGenerator->setPValue(pValue);
		joint->commandGenerator->setVoltage(INT_TO_VOLTS * m_boardData.voltage); // Note: This could use m_statVoltage, but as statistics are disabled this is a decent drop-in replacement

		// If requested, use the servo model to generate the final position command
		// We create a linear mix between the raw command (goal position) and
		// the command generated by the servo model. The slope of the coefficient
		// is limited by the m_rawStateSlopeLimit().
		double rawGoal = ((!joint->cmd.raw && useModel()) ? 1 : 0);
		joint->rawState = slopeLimited<double>(joint->rawState, rawGoal, m_rawStateSlopeLimit()); // joint->rawState is a dimensionless parameter used to interpolate between a given raw command and servo model based command
		double modelCmd = joint->commandGenerator->servoCommandFor(joint->cmd.pos, joint->cmd.vel, joint->cmd.acc, joint->feedback.modelTorque);
		double rawPosCmd = joint->cmd.pos;
		rawPosCmd = joint->rawState * modelCmd + (1.0 - joint->rawState) * rawPosCmd;

		// Save the generated command for plotting and logging
		joint->cmd.rawPos = rawPosCmd;

		// Negate the command if the joint's invert flag is set
		if(joint->invert())
			rawPosCmd = -rawPosCmd;

		// Convert the command into a servo position in ticks
		int goal = TICKS_PER_RAD*rawPosCmd + joint->tickOffset();

		// Goal position saturation
		// TODO: Increase P if saturation occurs and provide some sort of statistics/warnings if this occurs
		if(goal > 4095) goal = 4095;
		else if(goal < 0) goal = 0;

		// Write the calculated values into the required JointCmdSyncWriteData struct
		data->id = joint->id();
		data->p_gain = realpValue;
		data->goal_position = (uint16_t) goal;
	}
	
	// Sync write the required joint targets
	if(!syncWriteJointTargets(m_model->numJoints(), &params[0]))
		return false;
	
	// Return success
	return true;
}

// Perform the actual sync write of a JointCmdSyncWriteData to the CM730
bool RobotInterface::syncWriteJointTargets(size_t numDevices, const uint8_t* data)
{
	// Relaxed robots won't do anything. They are lazy.
	if(m_relaxed || m_board->isSuspended()) return true;

	// Sync write the joint commands to the servos via the CM730
	if(m_board->syncWrite(DynamixelMX::P_P_GAIN, sizeof(JointCmdSyncWriteData)-1, numDevices, data) != CM730::RET_SUCCESS) // Note: Size minus 1 as the id byte doesn't count as a write data byte
	{
		ROS_ERROR("SyncWrite of joint commands failed!");
		return false;
	}
	else return true;
}

/**
 * @internal Perform a servo sync write (via the CM730) to configure the permitted servo torques.
 **/
bool RobotInterface::setStiffness(float torque)
{
	// Display a throttled info message that fading is active
	ROS_INFO_THROTTLE(0.4, "Fading is active (%.3f)", torque);

	// If zero torque has been requested, then send a packet to disable all the servo torques altogether (really relax everything)
	if(torque <= 0.0)
	{
		// Coerce the torque variable
		torque = 0.0;

		// Allocate memory for the packet data and map an array of TorqueEnableSyncWriteData structs onto it
		std::vector<uint8_t> params(m_model->numJoints() * sizeof(TorqueEnableSyncWriteData));
		TorqueEnableSyncWriteData* paramData = (TorqueEnableSyncWriteData*) &params[0];

		// Populate the struct array with the required data
		for(size_t i = 0; i < m_model->numJoints(); i++)
		{
			paramData[i].id = dxlJoint(i)->id();
			paramData[i].torque_enable = 0; // 0 => Off, 1 => On
		}

		// Perform a sync write of the generated packet to the servos via the CM730
		if(!syncWriteTorqueEnable(m_model->numJoints(), &params[0]))
			return false;

		// Update the relaxed flag
		m_relaxed = true;
	}
	else
	{
		// Update the relaxed flag
		m_relaxed = false;
	}

	// Coerce the torque variable
	if(torque > 1.0) torque = 1.0;

	// Allocate memory for the packet data and map an array of TorqueLimitSyncWriteData structs onto it
	std::vector<uint8_t> params(m_model->numJoints() * sizeof(TorqueLimitSyncWriteData));
	TorqueLimitSyncWriteData* paramData = (TorqueLimitSyncWriteData*) &params[0];

	// Populate the struct array with the required data
	for(size_t i = 0; i < m_model->numJoints(); i++)
	{
		paramData[i].id = dxlJoint(i)->id();
		paramData[i].torque_limit = (uint16_t)(torque * FULL_TORQUE); // 0 => Limit to 0% of maximum available torque, 1023 => Limit to 100% of maximum available torque
	}

	// Perform a sync write of the generated packet to the servos via the CM730
	if(!syncWriteTorqueLimit(m_model->numJoints(), &params[0]))
		return false;

	// Return success
	return true;
}

// Sync write a torque enable command to the CM730
bool RobotInterface::syncWriteTorqueEnable(size_t numDevices, const uint8_t* data)
{
	// Sync write the appropriate packet
	if(m_board->syncWrite(DynamixelMX::P_TORQUE_ENABLE, sizeof(TorqueEnableSyncWriteData)-1, numDevices, data) != CM730::RET_SUCCESS) // Note: Size minus 1 as the id byte doesn't count as a write data byte
	{
		ROS_ERROR("SyncWrite of torque-off packet failed!");
		return false;
	}
	else return true;
}

// Sync write a torque limit command to the CM730
bool RobotInterface::syncWriteTorqueLimit(size_t numDevices, const uint8_t* data)
{
	// Sync write the appropriate packet
	if(m_board->syncWrite(DynamixelMX::P_TORQUE_LIMIT_L, sizeof(TorqueLimitSyncWriteData)-1, numDevices, data) != CM730::RET_SUCCESS) // Note: Size minus 1 as the id byte doesn't count as a write data byte
	{
		ROS_ERROR("SyncWrite of torque limit packet failed");
		return false;
	}
	else return true;
}

/**
 * We query a single joint for its statistics data (voltage + temperature).
 * In the next callback, the next joint is asked, and so on.
 **/
void RobotInterface::handleStatistics(const ros::TimerEvent&)
{
	// Perform an action only if robotcontrol isn't paused
	if(!m_robPause() && (!m_haveHardware || !m_board->isSuspended()))
	{
		// Get statistics from the servos
		if(m_haveHardware)
		{
// 			// Note: This block has been commented out in an attempt to minimise communications issues!
// 			// Retrieve the joint that we are going to pick on this time
// 			DXLJoint* joint = dxlJoint(m_statIndex);
// 
// 			// Query the servo for the required statistics data
// 			StatisticsReadData statReadData;
// 			if(m_board->readData(joint->id(), DynamixelMX::P_PRESENT_VOLTAGE, &statReadData, sizeof(statReadData)) == CM730::SUCCESS)
// 			{
// 				joint->temperature = statReadData.temperature;
// 				joint->voltage = statReadData.voltage;
// 			}
// 
// 			// Average all the servo voltages to get a single 'current voltage'
// 			m_statVoltage = 0;
// 			for(size_t i = 0; i < m_model->numJoints(); i++)
// 				m_statVoltage += INT_TO_VOLTS * dxlJoint(i)->voltage;
// 			m_statVoltage /= m_model->numJoints();
// 
// 			// Increment (and if required wrap) the statistics servo ID
// 			if(++m_statIndex == m_model->numJoints())
// 				m_statIndex = 0;
		}

		// Send the latest LED commands
		sendCM730LedCommand();
	}
}

/**
 * Reports the battery voltage (mean of all servo voltages) and the maximum
 * temperature measured in the statistics callback over all servos.
 **/
void RobotInterface::getDiagnostics(robotcontrol::DiagnosticsPtr diag)
{
	// Retrieve the battery voltage as measured by the CM730
	diag->batteryVoltage = INT_TO_VOLTS * m_boardData.voltage; // Note: m_boardData.voltage must be initialised in the constructor, just in case, so this never references an indeterminate value

	// Retrieve the maximum servo temperature, and compile a vector of the individual joint statistics (e.g. timeouts, checksum errors, etc...)
	diag->servos.clear();
	diag->servoTemperature = 0;
	for(size_t i = 0; i < m_model->numJoints(); i++)
	{
		DXLJoint* joint = dxlJoint(i);
		if(joint->temperature > diag->servoTemperature)
			diag->servoTemperature = joint->temperature;
		diag->servos.push_back(joint->diag);
	}
}

/**
 * Handler function for the read offsets service. This saves all present
 * servo positions as the desired tick offsets, effectively making the
 * current pose the zero position.
 * @sa handleReadOffset()
 **/
bool RobotInterface::handleReadOffsets(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	// Ignore service call if we don't actually have any hardware
	if(!m_haveHardware)
	{
		ROS_ERROR("ReadOffsets service call was ignored as no hardware is actually present!");
		return false;
	}

	// Ignore service call if the CM730 is currently suspended
	if(m_board->isSuspended())
	{
		ROS_ERROR("ReadOffsets service call was ignored as the CM730 is currently suspended! Please try again.");
		return false;
	}

	// Go through each joint and save the present positions as the tick offsets
	for(size_t i = 0; i < m_model->numJoints(); i++)
	{
		// Retrieve a pointer to the joint
		DXLJoint* joint = dxlJoint(i);

		// Get the present servo position
		int present_pos;
		if(m_board->readWord(joint->id(), DynamixelMX::P_PRESENT_POSITION_L, &present_pos) != CM730::RET_SUCCESS) continue;

		// Set the present servo position as the tick offset
		joint->tickOffset.set(present_pos);                                              
	}

	// Return success
	return true;
}

/**
 * Handler function for the read offset service. This saves the present servo position
 * of a particular servo as the desired tick offset, effectively making the current
 * pose the zero position.
 * @sa handleReadOffsets()
 **/
bool RobotInterface::handleReadOffset(ReadOffsetRequest& req, ReadOffsetResponse& resp)
{
	// Ignore service call if we don't actually have any hardware
	if(!m_haveHardware)
	{
		ROS_ERROR("ReadOffsets service call was ignored as no hardware is actually present!");
		return false;
	}

	// Ignore service call if the CM730 is currently suspended
	if(m_board->isSuspended())
	{
		ROS_ERROR("ReadOffset service call was ignored as the CM730 is currently suspended! Please try again.");
		return false;
	}

	// Retrieve a pointer to the required joint by name (std::string)
	DXLJoint* joint = (DXLJoint*) m_model->getJoint(req.joint).get();
	if(!joint) return false;

	// Get the present servo position
	int present_pos;
	if(m_board->readWord(joint->id(), DynamixelMX::P_PRESENT_POSITION_L, &present_pos) != CM730::RET_SUCCESS)
		return false;

	// Display the measured present position
	ROS_INFO("%s present position: %d", req.joint.c_str(), present_pos);

	// Populate the service call response struct with the tick shift is being made through this service call
	resp.ticks = joint->tickOffset() - present_pos;

	// Set the present servo position as the tick offset
	joint->tickOffset.set(present_pos);

	// Return success
	return true;
}

/**
 * Transcribe the value of the acc-only method flag on the config server to the internals of the attitude estimator
 **/
void RobotInterface::updateAttEstMethod()
{
	// Update the acc-only resolution method
	m_attitudeEstimator.setAccMethod(m_attEstUseFusedMethod() ? stateestimation::AttitudeEstimator::ME_FUSED_YAW : stateestimation::AttitudeEstimator::ME_ZYX_YAW);
	m_attEstNoMag.setAccMethod(m_attEstUseFusedMethod() ? stateestimation::AttitudeEstimator::ME_FUSED_YAW : stateestimation::AttitudeEstimator::ME_ZYX_YAW);
}

/**
 * Transcribe the values from the attEstPIGains config server parameters to the internals of the attitude estimator
 **/
void RobotInterface::updateAttEstPIGains()
{
	// Update the attitude estimator parameters
	m_attitudeEstimator.setPIGains(m_attEstKp(), m_attEstTi(), m_attEstKpQuick(), m_attEstTiQuick());
	m_attEstNoMag.setPIGains(m_attEstKp(), m_attEstTi(), m_attEstKpQuick(), m_attEstTiQuick());
}

/**
 * Transcribe the values from the attEstMagCalib config server parameters to the internals of the attitude estimator
 **/
void RobotInterface::updateAttEstMagCalib()
{
	// Update the attitude estimator parameters
	m_attitudeEstimator.setMagCalib(m_attEstMagCalibX(), m_attEstMagCalibY(), m_attEstMagCalibZ());
	m_attEstNoMag.setMagCalib(m_attEstMagCalibX(), m_attEstMagCalibY(), m_attEstMagCalibZ());
}

/**
 * Handler function for the attitude estimation magnetometer calibration service.
 * Position the robot so that it is perfectly upright (the robot's local z-axis points in exactly the opposite
 * direction to gravity) and is facing in the direction that is to be declared the zero yaw direction.
 * Nominally this is parallel to the field and facing the positive goal. While the robot is in exactly this
 * position (with as much free air around it as possible too), fire off a call to this service using:
 * @code
 * rosservice call /robotcontrol/nopInterface/attEstCalibrate
 * @endcode
 **/
bool RobotInterface::handleAttEstCalibrate(nimbro_op_interface::AttEstMagCalibRequest& req, nimbro_op_interface::AttEstMagCalibResponse& resp)
{
	// Retrieve the current magnetic field vector
	Eigen::Vector3d mag = m_model->magneticFieldVector();

	// Update the config server parameters
	m_attEstMagCalibX.set((float) mag.x());
	m_attEstMagCalibY.set((float) mag.y());
	m_attEstMagCalibZ.set((float) mag.z());

	// Inform the user of the results of the calibration
	ROS_INFO("Attitude estimation calibration: attEstMagCalib vector has been updated to be (%10.7lf, %10.7lf, %10.7lf)", mag.x(), mag.y(), mag.z());

	// Return the result of the calibration
	resp.magCalibX = mag.x();
	resp.magCalibY = mag.y();
	resp.magCalibZ = mag.z();

	// Return that the service was successfully handled
	return true;
}

/**
 * Apply the config variable for the magnetometer spike filter max delta.
 **/
void RobotInterface::updateMagSpikeMaxDelta()
{
	// Update the spike filters
	m_magSpikeFilterX.setMaxDelta(m_magSpikeMaxDelta());
	m_magSpikeFilterY.setMaxDelta(m_magSpikeMaxDelta());
	m_magSpikeFilterZ.setMaxDelta(m_magSpikeMaxDelta());
}

/**
 * Handler function for when we receive notification of a new button press.
 **/
void RobotInterface::handleButton(int number) // Note: Calling this means that the number-th bit of the P_BUTTON byte (retrieved from the CM730) was set!
{
	// If the leftmost button, fade in/out depending on the current robot state
	if(number == 0) // Note: This corresponds to the leftmost button
	{
		if(m_fadingIsTriggered)
			ROS_INFO("Dismissing extra fade request (torque fading is already active)!");
		else
		{
			FadeTorqueGoal goal;
			goal.torque = (m_model->state() == m_state_relaxed ? 1.0 : 0.0);
			m_fadingIsTriggered = true;
			m_fadeTorqueClient.sendGoal(goal, boost::bind(&RobotInterface::resetFadingTriggered, this));
		}
	}
}

// Callback for monitoring when a sent fade goal has finished
void RobotInterface::resetFadingTriggered()
{
	// Reset the fading triggered flag
	m_fadingIsTriggered = false;
}

}

PLUGINLIB_EXPORT_CLASS(nimbro_op_interface::RobotInterface, robotcontrol::HardwareInterface);
// EOF