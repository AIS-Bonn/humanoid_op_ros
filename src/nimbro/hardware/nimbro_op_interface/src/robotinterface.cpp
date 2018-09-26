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
#include <robotcontrol/robotcontrol.h>

// Includes - Packages
#include <cm730/dynamixel.h>
#include <rot_conv/rot_conv.h>
#include <rc_utils/ros_time.h>
#include <rc_utils/slopelimited.h>
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
#define BUTTON0_RELAX_TIME         2.0   // The time until a long press of button 0 relaxes the robot
#define BUTTON0_UNRELAX_TIME       1.0   // The time after BUTTON0_RELAX_TIME until a continued long press of button 0 unrelaxes the robot
#define BUTTON0_RECOVER_PERIOD     0.1   // The regular time period after BUTTON0_RELAX_TIME (inclusive) at which CM730 recovery packets are sent
#define BUTTON1_LONG_TIME          2.0   // The time that button 1 needs to be pressed until it registers as a long press

// Namespaces
using namespace robotcontrol;
using namespace cm730;

// NimbRo-OP interface namespace
namespace nimbro_op_interface
{

// Constants
const std::string RobotInterface::RESOURCE_PATH = "nimbro_op_interface/";
const std::string RobotInterface::CONFIG_PARAM_PATH = "/nimbro_op_interface/";

// Constants
const std::string RobotInterface::DXLJoint::CONFIG_PARAM_PATH = RobotInterface::CONFIG_PARAM_PATH + "DXLJoint/";

/**
 * DXLJoint constructor. This function creates parameters on the config server
 * for the joint.
 **/
RobotInterface::DXLJoint::DXLJoint(const std::string& _name)
 : type(CONFIG_PARAM_PATH + "joints/" + _name + "/type", "default_type")
 , id(CONFIG_PARAM_PATH + "joints/" + _name + "/id", 1, 1, 253, 1)
 , tickOffset(CONFIG_PARAM_PATH + "offsets/" + _name, 0, 1, 4095, 2048)
 , invert(CONFIG_PARAM_PATH + "joints/" + _name + "/invert", false)
 , readFeedback(CONFIG_PARAM_PATH + "joints/" + _name + "/readFeedback", true)
 , enabled(CONFIG_PARAM_PATH + "joints/" + _name + "/enabled", true)
 , realEffort(cmd.effort)
 , rawState(1.0)
{
	// Initialize variables
	name = _name;
	diag.name = name;
}

// Scaling constants
const double RobotInterface::INT_TO_VOLTS = 0.1; // Multiply a voltage value read from the CM730 by this to convert it into volts (see 'cm730/firmware/CM730_HW/src/adc.c')
const double RobotInterface::GYRO_SCALE = (M_PI / 180) * (2 * 250) / 65536; // From control register CTRL_REG4 in "cm730/firmware/CM730_HW/src/gyro_acc.c": +-250dps is 65536 LSb
const double RobotInterface::ACC_SCALE = (2 * (4 * 9.81)) / 65536; // From control register CTRL_REG4 in "cm730/firmware/CM730_HW/src/gyro_acc.c": +-4g is 65536 LSb
const double RobotInterface::MAG_SCALE = 1 / 820.0; // From Config Register B in "cm730/firmware/CM730_APP/src/compass.c": 820 LSb/gauss

/**
 * RobotInterface constructor.
 **/
RobotInterface::RobotInterface()
 : m_nh("~")
 , m_model(NULL)
 , m_haveHardware(false)
 , m_buttonPress0(CONFIG_PARAM_PATH + "button/pressButton0", false)
 , m_buttonPress1(CONFIG_PARAM_PATH + "button/pressButton1", false)
 , m_buttonPress2(CONFIG_PARAM_PATH + "button/pressButton2", false)
 , m_relaxed(true)
 , m_enableTorque(false)
 , m_statIndex(0)
 , m_statVoltage(-1.0)
 , m_resetGyroAccOffset(CONFIG_PARAM_PATH + "imuOffsets/resetGyroAccOffset", false)
 , m_resetMagOffset(CONFIG_PARAM_PATH + "imuOffsets/resetMagOffset", false)
 , m_gyroAccFusedYaw(CONFIG_PARAM_PATH + "imuOffsets/gyroAccFusedYaw", -M_PI, 0.01, M_PI, 0.0)
 , m_gyroAccTiltAngle(CONFIG_PARAM_PATH + "imuOffsets/gyroAccTiltAngle", -M_PI, 0.01, M_PI, 0.0)
 , m_gyroAccTiltAxisAngle(CONFIG_PARAM_PATH + "imuOffsets/gyroAccTiltAxisAngle", -M_PI, 0.01, M_PI, 0.0)
 , m_magFlip(CONFIG_PARAM_PATH + "imuOffsets/magFlip", false)
 , m_magFusedYaw(CONFIG_PARAM_PATH + "imuOffsets/magFusedYaw", -M_PI, 0.01, M_PI, 0.0)
 , m_magTiltAngle(CONFIG_PARAM_PATH + "imuOffsets/magTiltAngle", -M_PI, 0.01, M_PI, 0.0)
 , m_magTiltAxisAngle(CONFIG_PARAM_PATH + "imuOffsets/magTiltAxisAngle", -M_PI, 0.01, M_PI, 0.0)
 , m_gyroAccMaxRelAccScale(CONFIG_PARAM_PATH + "imuOffsets/calibMaxRelAccScale", 0.0, 0.01, 0.9, 0.4)
 , m_gyroAccMinYawAgreement(CONFIG_PARAM_PATH + "imuOffsets/calibMinYawAgreement", 0.5, 0.01, 1.0, 0.8)
 , m_imuOffsetsGACalib(false)
 , m_attEstUseFusedMethod(CONFIG_PARAM_PATH + "attEstUseFusedMethod", true)
 , m_attEstKp(CONFIG_PARAM_PATH + "attEstPIGains/Kp", 0.05, 0.01, 30.0, 2.20)
 , m_attEstTi(CONFIG_PARAM_PATH + "attEstPIGains/Ti", 0.05, 0.01, 10.0, 2.65)
 , m_attEstKpQuick(CONFIG_PARAM_PATH + "attEstPIGains/KpQuick", 0.05, 0.01, 30.0, 10.0)
 , m_attEstTiQuick(CONFIG_PARAM_PATH + "attEstPIGains/TiQuick", 0.05, 0.01, 10.0, 1.25)
 , m_attEstKpYaw(CONFIG_PARAM_PATH + "attEstPIGains/KpYaw", 0.05, 0.01, 30.0, 2.0)
 , m_attEstKpYawQuick(CONFIG_PARAM_PATH + "attEstPIGains/KpYawQuick", 0.05, 0.01, 30.0, 2.0)
 , m_attEstMagCalibX(CONFIG_PARAM_PATH + "attEstMagCalib/x", -1.5, 0.01, 1.5, 1.0)
 , m_attEstMagCalibY(CONFIG_PARAM_PATH + "attEstMagCalib/y", -1.5, 0.01, 1.5, 0.0)
 , m_attEstMagCalibZ(CONFIG_PARAM_PATH + "attEstMagCalib/z", -1.5, 0.01, 1.5, 0.0)
 , m_attEstGyroBiasX(CONFIG_PARAM_PATH + "attEstGyroCalib/gyroBiasX", -0.15, 0.001, 0.15, 0.0)
 , m_attEstGyroBiasY(CONFIG_PARAM_PATH + "attEstGyroCalib/gyroBiasY", -0.15, 0.001, 0.15, 0.0)
 , m_attEstGyroBiasZ(CONFIG_PARAM_PATH + "attEstGyroCalib/gyroBiasZ", -0.15, 0.001, 0.15, 0.0)
 , m_attEstGyroBiasUpdate(CONFIG_PARAM_PATH + "attEstGyroCalib/forceUpdateGyroBias", false)
 , m_attEstGyroBiasSetFM(CONFIG_PARAM_PATH + "attEstGyroCalib/setGyroBiasFromMean", false)
 , m_attEstGyroBiasSetFSM(CONFIG_PARAM_PATH + "attEstGyroCalib/setGyroBiasFromSmoothMean", false)
 , m_attEstGyroBiasLast(Eigen::Vector3f::Zero())
 , m_temperatureLowPassTs(CONFIG_PARAM_PATH + "temperature/lowPassTs", 1.0, 0.2, 20.0, 10.0)
 , m_temperature(0.0)
 , m_voltageLowPassTs(CONFIG_PARAM_PATH + "voltage/lowPassTs", 5.0, 1.0, 50.0, 15.0)
 , m_voltage(0.0)
 , m_accLowPassMeanTs(CONFIG_PARAM_PATH + "acc/lowPassMeanTs", 0.05, 0.05, 5.0, 1.0)
 , m_accMean(Eigen::Vector3d::Zero())
 , m_fir_accX(rc_utils::FIRFilterType::FT_AVERAGE)
 , m_fir_accY(rc_utils::FIRFilterType::FT_AVERAGE)
 , m_fir_accZ(rc_utils::FIRFilterType::FT_AVERAGE)
 , m_gyroEnableAutoCalib(CONFIG_PARAM_PATH + "gyroAutoCalib/enable", false)
 , m_gyroScaleFactorHT(CONFIG_PARAM_PATH + "gyro/scaleFactorHT", 0.5, 0.01, 2.0, 1.0)
 , m_gyroScaleFactorLT(CONFIG_PARAM_PATH + "gyro/scaleFactorLT", 0.5, 0.01, 2.0, 1.0)
 , m_gyroTemperatureHigh(CONFIG_PARAM_PATH + "gyro/temperatureHigh", 15.0, 0.5, 80.0, 50.0)
 , m_gyroTemperatureLow(CONFIG_PARAM_PATH + "gyro/temperatureLow", 15.0, 0.5, 80.0, 30.0)
 , m_gyroLowPassMeanTs(CONFIG_PARAM_PATH + "gyro/lowPassMeanTs", 0.05, 0.05, 10.0, 2.0)
 , m_gyroLowPassMeanTsHigh(CONFIG_PARAM_PATH + "gyro/lowPassMeanTsHigh", 2.0, 0.2, 20.0, 8.0)
 , m_gyroStabilityBound(CONFIG_PARAM_PATH + "gyroAutoCalib/stabilityBound", 0.0, 0.0005, 0.04, 0.02)
 , m_gyroCalibFadeTimeStart(CONFIG_PARAM_PATH + "gyroAutoCalib/fadeTimeStart", 0.5, 0.05, 5.0, 1.0)
 , m_gyroCalibFadeTimeDur(CONFIG_PARAM_PATH + "gyroAutoCalib/fadeTimeDuration", 0.5, 0.02, 3.0, 1.2)
 , m_gyroCalibTsSlow(CONFIG_PARAM_PATH + "gyroAutoCalib/biasTsSlow", 0.1, 0.05, 5.0, 1.5)
 , m_gyroCalibTsFast(CONFIG_PARAM_PATH + "gyroAutoCalib/biasTsFast", 0.1, 0.05, 5.0, 0.2)
 , m_gyroMean(Eigen::Vector3d::Zero())
 , m_gyroMeanSmooth(Eigen::Vector3d::Zero())
 , m_gyroStableCount(0)
 , m_gyroBiasAdjusting(false)
 , m_gyroCalibrating(0)
 , m_gyroCalibType(0)
 , m_gyroCalibNumTurns(0)
 , m_gyroCalibLoopCount(0)
 , m_gyroCalibLoopCountFirst(0)
 , m_gyroCalibUpdateCount(0)
 , m_gyroCalibUpdateCountFirst(0)
 , m_gyroCalibScaleFactor(-1.0)
 , m_gyroCalibInitYaw(0.0)
 , m_gyroCalibInitTemp(35.0)
 , m_gyroCalibMiddleYaw(0.0)
 , m_gyroCalibMiddleTemp(35.0)
 , m_gyroCalibCurYaw(0.0)
 , m_useMagnetometer(CONFIG_PARAM_PATH + "useMagnetometer", false)
 , m_magSpikeMaxDelta(CONFIG_PARAM_PATH + "magSpikeMaxDelta", 0.0, 0.001, 0.20, 0.07)
 , m_magSpikeFilterX()
 , m_magSpikeFilterY()
 , m_magSpikeFilterZ()
 , m_magFirFilterX(rc_utils::FIRFilterType::FT_AVERAGE)
 , m_magFirFilterY(rc_utils::FIRFilterType::FT_AVERAGE)
 , m_magFirFilterZ(rc_utils::FIRFilterType::FT_AVERAGE)
 , m_magHardIronFilter(RESOURCE_PATH + "magFilter/", CONFIG_PARAM_PATH + "magFilter/", DEFAULT_TIMER_DURATION)
 , m_useModel(CONFIG_PARAM_PATH + "useModel", false)
 , m_effortSlopeLimit(CONFIG_PARAM_PATH + "effortSlopeLimit", 0.0, 0.01, 1.0, 0.02)
 , m_rawStateSlopeLimit(CONFIG_PARAM_PATH + "rawStateSlopeLimit", 0.0, 0.01, 1.0, 0.02)
 , m_useServos(CONFIG_PARAM_PATH + "useServos", true)
 , m_PM(PM_COUNT, RESOURCE_PATH)
 , m_plotRobotInterfaceData(CONFIG_PARAM_PATH + "plotData", false)
 , m_lastButtons(0)
 , m_buttonTime0(0, 0)
 , m_buttonTime1(0, 0)
 , m_buttonLastRec0(0, 0)
 , m_buttonState0(BPS_RELEASED)
 , m_buttonState1(BPS_RELEASED)
 , m_showButtonPresses(CONFIG_PARAM_PATH + "showButtonPresses", false)
 , m_commsOk(true)
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
 , m_hadRX(false)
 , m_hadTX(false)
 , m_robPause(RobotControl::CONFIG_PARAM_PATH + "pause", false) // See main() in robotcontrol.cpp for the main use of this config variable
 , m_fadeTorqueClient(m_nh, "fade_torque")
 , m_fadingIsTriggered(false)
{
	// Retrieve the node handle
	ros::NodeHandle nhs;

	// Retrieve the name and type of the robot
	nhs.param<std::string>("robot_name", m_robotNameStr, std::string());
	nhs.param<std::string>("robot_type", m_robotTypeStr, std::string());
	if(m_robotNameStr.empty())
		ROS_ERROR("Robot name is empty or unconfigured!");
	if(m_robotTypeStr.empty())
		ROS_ERROR("Robot type is empty or unconfigured!");

	// Initialise logger heartbeat message
	m_loggerMsg.sourceNode = "robotcontrol";
	m_loggerMsg.enableLogging = false;
	m_loggerStamp.fromNSec(0);

	// Initialize statistics timer
	m_statisticsTimer = m_nh.createTimer(ros::Duration(0.1), &RobotInterface::handleStatistics, this, false, false);

	// Advertise topics
	m_pub_buttons = nhs.advertise<nimbro_op_interface::Button>("button", 1);
	m_pub_led_state = nhs.advertise<nimbro_op_interface::LEDCommand>("led_state", 1);
	m_pub_logger = nhs.advertise<rrlogger::LoggerHeartbeat>("rclogger/heartbeat", 1);

	// Subscribe topics
	m_sub_led = nhs.subscribe("led", 5, &RobotInterface::handleLEDCommand, this);

	// Set up the buzzer
	m_sub_buzzer = nhs.subscribe(RESOURCE_PATH + "buzzer", 1, &RobotInterface::handleBuzzerCommand, this);
	m_buzzerData.playLength = 0xFF;
	m_buzzerData.data = 0;
	m_haveBuzzerData = false;

	// Configure the temperature processing
	m_temperatureLowPassTs.setCallback(boost::bind(&RobotInterface::updateTempLowPassTs, this), true);

	// Configure the voltage processing
	m_boardData.voltage = (unsigned char)((15.0 / INT_TO_VOLTS) + 0.5); // Note: The 0.5 is for rounding purposes
	m_voltageLowPassTs.setCallback(boost::bind(&RobotInterface::updateVoltageLowPassTs, this), true);
	m_voltageLowPass.setValue(INT_TO_VOLTS * m_boardData.voltage);
	m_initedVoltage = false;

	// Configure the IMU offsets
	m_resetGyroAccOffset.set(false);
	m_resetGyroAccOffset.setCallback(boost::bind(&RobotInterface::handleResetGyroAccOffset, this), false);
	m_resetMagOffset.set(false);
	m_resetMagOffset.setCallback(boost::bind(&RobotInterface::handleResetMagOffset, this), false);

	// Configure the acc data processing
	m_accLowPassMeanTs.setCallback(boost::bind(&RobotInterface::updateAccLowPassMeanTs, this), true);

	// Configure the gyro data processing
	m_gyroLowPassMeanTs.setCallback(boost::bind(&RobotInterface::updateGyroLowPassMeanTs, this), true);
	m_gyroLowPassMeanTsHigh.setCallback(boost::bind(&RobotInterface::updateGyroVeryLowPassMeanTs, this), true);

	// Configure the magnetometer data processing
	m_magSpikeMaxDelta.setCallback(boost::bind(&RobotInterface::updateMagSpikeMaxDelta, this), true);

	// Configure the attitude estimators
	boost::function<void()> updateCallbackME = boost::bind(&RobotInterface::updateAttEstMethod, this);
	m_attEstUseFusedMethod.setCallback(boost::bind(updateCallbackME));
	updateAttEstMethod(); // Calls AttitudeEstimator::setAccMethod() internally
	boost::function<void()> updateCallbackPI = boost::bind(&RobotInterface::updateAttEstPIGains, this);
	m_attEstKp.setCallback(boost::bind(updateCallbackPI));
	m_attEstTi.setCallback(boost::bind(updateCallbackPI));
	m_attEstKpQuick.setCallback(boost::bind(updateCallbackPI));
	m_attEstTiQuick.setCallback(boost::bind(updateCallbackPI));
	m_attEstKpYaw.setCallback(boost::bind(updateCallbackPI));
	m_attEstKpYawQuick.setCallback(boost::bind(updateCallbackPI));
	updateAttEstPIGains(); // Calls AttitudeEstimator::setPIGains() internally
	boost::function<void()> updateCallbackMC = boost::bind(&RobotInterface::updateAttEstMagCalib, this);
	m_attEstMagCalibX.setCallback(boost::bind(updateCallbackMC));
	m_attEstMagCalibY.setCallback(boost::bind(updateCallbackMC));
	m_attEstMagCalibZ.setCallback(boost::bind(updateCallbackMC));
	updateAttEstMagCalib(); // Calls AttitudeEstimator::setMagCalib() internally
	boost::function<void()> updateCallbackGC = boost::bind(&RobotInterface::updateAttEstGyroCalib, this);
	setAttEstGyroCalib(Eigen::Vector3f::Zero());
	m_attEstGyroBiasX.setCallback(boost::bind(updateCallbackGC));
	m_attEstGyroBiasY.setCallback(boost::bind(updateCallbackGC));
	m_attEstGyroBiasZ.setCallback(boost::bind(updateCallbackGC));
	m_attEstGyroBiasUpdate.setCallback(boost::bind(updateCallbackGC));
	updateAttEstGyroCalib(); // Calls AttitudeEstimator::setGyroBias() internally
	m_attEstGyroBiasSetFM.set(false);
	m_attEstGyroBiasSetFSM.set(false);

	// Reset the button press config parameters just in case
	m_buttonPress0.set(false);
	m_buttonPress1.set(false);
	m_buttonPress2.set(false);

	// Advertise provided services
	m_srv_listJoints = nhs.advertiseService(RESOURCE_PATH + "listJoints", &RobotInterface::handleListJoints, this);
	m_srv_readOffsets = nhs.advertiseService(RESOURCE_PATH + "readOffsets", &RobotInterface::handleReadOffsets, this);
	m_srv_readOffset = nhs.advertiseService(RESOURCE_PATH + "readOffset", &RobotInterface::handleReadOffset, this);
	m_srv_attEstCalibrate = nhs.advertiseService(RESOURCE_PATH + "attEstCalibrate", &RobotInterface::handleAttEstCalibrate, this);
	m_srv_calibrateGyroStart = nhs.advertiseService(RESOURCE_PATH + "calibrateGyroStart", &RobotInterface::handleCalibrateGyroStart, this);
	m_srv_calibrateGyroReturn = nhs.advertiseService(RESOURCE_PATH + "calibrateGyroReturn", &RobotInterface::handleCalibrateGyroReturn, this);
	m_srv_calibrateGyroStop = nhs.advertiseService(RESOURCE_PATH + "calibrateGyroStop", &RobotInterface::handleCalibrateGyroStop, this);
	m_srv_calibrateGyroAbort = nhs.advertiseService(RESOURCE_PATH + "calibrateGyroAbort", &RobotInterface::handleCalibrateGyroAbort, this);
	m_srv_imuOffsetsCalibGyroAccStart = nhs.advertiseService(RESOURCE_PATH + "imuOffsets/calibGyroAccStart", &RobotInterface::handleCalibGyroAccStart, this);
	m_srv_imuOffsetsCalibGyroAccNow = nhs.advertiseService(RESOURCE_PATH + "imuOffsets/calibGyroAccNow", &RobotInterface::handleCalibGyroAccNow, this);
	m_srv_imuOffsetsCalibGyroAccStop = nhs.advertiseService(RESOURCE_PATH + "imuOffsets/calibGyroAccStop", &RobotInterface::handleCalibGyroAccStop, this);

	// Configure the plot manager
	configurePlotManager();
}

/**
 * RobotInterface destructor.
 **/
RobotInterface::~RobotInterface()
{
}

/**
 * Deinitialisation function.
 **/
void RobotInterface::deinit()
{
	// Stop the statistics timer
	m_statisticsTimer.stop();

	// Send a final logger heartbeat
	sendLoggerHeartbeat(false);
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
	joint->enabled.setCallback(boost::bind(updateCb));

	// Initialize the joint parameters
	joint->commandGenerator = commandGenerator(joint->type());
	joint->voltage = -1;
	joint->temperature = -1;

	// Call the update callback once to begin with
	updateJointSettings(joint.get());

	// Return the constructed joint
	return joint;
}

/**
 * @return pointer to the DXLJoint corresponding to the given servo @p id,
 *    nullptr if not found.
 **/
RobotInterface::DXLJoint* RobotInterface::dxlJointForID(int id) const
{
	// Search for the given ID in the RobotModel joint array
	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		DXLJoint* joint = dxlJoint(i);
		if(joint->id() == id)
			return joint;
	}

	// Return a null pointer if the ID isn't found
	return nullptr;
}

/**
 * Returns the pointer to the DXLJoint corresponding to the given joint name, nullptr if not found.
 **/
RobotInterface::DXLJoint* RobotInterface::dxlJointForName(const std::string& name) const
{
	// Search for the given ID in the RobotModel joint array
	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		DXLJoint* joint = dxlJoint(i);
		if(joint->name == name)
			return joint;
	}

	// Return a null pointer if the ID isn't found
	return nullptr;
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
	if(joint->enabled() && joint->readFeedback())
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

// Joint command dependency function: Assign the command of another joint
void RobotInterface::jointCmdEqual(DXLJoint::Command* cmd, const DXLJoint::Command* cmdA)
{
	// Apply the required dependency
	cmd->pos = cmdA->pos;
	cmd->vel = cmdA->vel;
	cmd->acc = cmdA->acc;
	cmd->effort = cmdA->effort;
	cmd->raw = cmdA->raw;
}

// Joint command dependency function: Assign the negative command of another joint
void RobotInterface::jointCmdNegate(DXLJoint::Command* cmd, const DXLJoint::Command* cmdA)
{
	// Apply the required dependency
	cmd->pos = -cmdA->pos;
	cmd->vel = -cmdA->vel;
	cmd->acc = -cmdA->acc;
	cmd->effort = cmdA->effort;
	cmd->raw = cmdA->raw;
}

// Joint command dependency function: Assign the sum of two joint commands
void RobotInterface::jointCmdSum(DXLJoint::Command* cmd, const DXLJoint::Command* cmdA, const DXLJoint::Command* cmdB)
{
	// Apply the required dependency
	cmd->pos = cmdA->pos + cmdB->pos;
	cmd->vel = cmdA->vel + cmdB->vel;
	cmd->acc = cmdA->acc + cmdB->acc;
	cmd->effort = 0.5*(cmdA->effort + cmdB->effort);
	cmd->raw = (cmdA->raw || cmdB->raw);
}

// Joint command dependency function: Assign the difference between two joint commands
void RobotInterface::jointCmdDiff(DXLJoint::Command* cmd, const DXLJoint::Command* cmdA, const DXLJoint::Command* cmdB)
{
	// Apply the required dependency
	cmd->pos = cmdA->pos - cmdB->pos;
	cmd->vel = cmdA->vel - cmdB->vel;
	cmd->acc = cmdA->acc - cmdB->acc;
	cmd->effort = 0.5*(cmdA->effort + cmdB->effort);
	cmd->raw = (cmdA->raw || cmdB->raw);
}

// Joint torque dependency function: Assign the torque of another joint
void RobotInterface::jointTorqueEqual(double* torque, const double* torqueA)
{
	// Apply the required dependency
	*torque = *torqueA;
}

// Joint torque dependency function: Assign the negative torque of another joint
void RobotInterface::jointTorqueNegate(double* torque, const double* torqueA)
{
	// Apply the required dependency
	*torque = -(*torqueA);
}

// Joint torque dependency function: Assign the sum of two joint torques
void RobotInterface::jointTorqueSum(double* torque, const double* torqueA, const double* torqueB)
{
	// Apply the required dependency
	*torque = *torqueA + *torqueB;
}

// Joint torque dependency function: Assign the difference between two joint torques
void RobotInterface::jointTorqueDiff(double* torque, const double* torqueA, const double* torqueB)
{
	// Apply the required dependency
	*torque = *torqueA - *torqueB;
}

// Joint feedback dependency function: Assign the feedback of another joint
void RobotInterface::jointFeedEqual(DXLJoint::Feedback* feed, const DXLJoint::Feedback* feedA)
{
	// Apply the required dependency
	feed->pos = feedA->pos;
	feed->torque = feedA->torque;
}

// Joint feedback dependency function: Assign the negative feedback of another joint
void RobotInterface::jointFeedNegate(DXLJoint::Feedback* feed, const DXLJoint::Feedback* feedA)
{
	// Apply the required dependency
	feed->pos = -feedA->pos;
	feed->torque = -feedA->torque;
}

// Joint feedback dependency function: Assign the sum of two joint feedbacks
void RobotInterface::jointFeedSum(DXLJoint::Feedback* feed, const DXLJoint::Feedback* feedA, const DXLJoint::Feedback* feedB)
{
	// Apply the required dependency
	feed->pos = feedA->pos + feedB->pos;
	feed->torque = feedA->torque + feedB->torque;
}

// Joint feedback dependency function: Assign the difference between two joint feedbacks
void RobotInterface::jointFeedDiff(DXLJoint::Feedback* feed, const DXLJoint::Feedback* feedA, const DXLJoint::Feedback* feedB)
{
	// Apply the required dependency
	feed->pos = feedA->pos - feedB->pos;
	feed->torque = feedA->torque - feedB->torque;
}

/**
 * Add dependencies for all mimic joints
 **/
bool RobotInterface::addMimicJointDependencies()
{
	// Find all mimic joints and try to create dependencies for them
	for(size_t i = 0; i < m_model->numJoints(); i++)
	{
		DXLJoint* joint = dxlJoint(i);
		if(!joint->mimic) continue;
		if(joint->mimic->multiplier == 0.0)
		{
			ROS_WARN("Ignoring mimic joint '%s' because it has a multiplier of 0!", joint->name.c_str());
			continue;
		}
		const DXLJoint* source = dxlJointForName(joint->mimic->joint_name);
		if(!source)
		{
			ROS_WARN("Failed to find source joint '%s' of mimic joint '%s' => Not creating mimic dependency!", joint->mimic->joint_name.c_str(), joint->name.c_str());
			continue;
		}
		addJointAlias(joint, source, joint->mimic->multiplier, joint->mimic->offset);
	}

	// Return success in any case
	return true;
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
	m_state_setting_pose = m_model->registerState("setting_pose");

	// Initialise any joint aliases and dependencies
	ROS_INFO("Initialising joint dependencies...");
	clearJointDependencies();
	if(!initJointDependencies())
	{
		ROS_ERROR("Failed to initialise joint dependencies!");
		return false;
	}

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

	// Deduce the type of the servos (must all be of the same servo type, e.g. MX series, or X series)
	int isMX = 0, isX = 0, isNone = 0, isOther = 0;
	for(size_t i = 0; i < m_model->numJoints(); i++)
	{
		const DXLJoint* joint = dxlJoint(i);
		std::string jointType = joint->type();
		if(jointType == "NONE")
			isNone++;
		else if(jointType.compare(0, 2, "MX") == 0)
			isMX++;
		else if(jointType.compare(0, 2, "XL") == 0 || jointType.compare(0, 2, "XM") == 0 || jointType.compare(0, 2, "XH") == 0)
			isX++;
		else
			isOther++;
	}
	if(isOther > 0)
	{
		ROS_ERROR("Found %d joints that are neither MX nor X series (nor 'NONE') => Not sure what servo hardware to target!", isOther);
		return false;
	}
	else if(isMX > 0 && isX == 0)
		m_servos = boost::make_shared<DynamixelMX>();
	else if(isX > 0 && isMX == 0)
		m_servos = boost::make_shared<DynamixelX>();
	else if(isMX == 0 && isX == 0)
	{
		ROS_ERROR("Found neither MX nor X servo joints => Not sure what servo hardware to target!");
		return false;
	}
	else
	{
		ROS_ERROR("Found a mix of %d MX and %d X servo joints => Not sure what servo hardware to target!", isMX, isX);
		return false;
	}
	if(m_servos)
		ROS_INFO("Targeting the %s servo hardware", m_servos->name().c_str());
	else
	{
		ROS_ERROR("Something went wrong when deducing the servo hardware to target!");
		return false;
	}

	// Sort the queryset (nice for debugging)
	std::sort(m_cm730_queryset.begin(), m_cm730_queryset.end());

	// Set the cycle time of the magnetometer filter
	m_magHardIronFilter.setCycleTime(m_model->timerDuration());

	// Initialise the low pass mean filter Ts values (requires m_model to retrieve the true timer duration)
	updateTempLowPassTs();
	updateAccLowPassMeanTs();
	updateGyroLowPassMeanTs();
	updateGyroVeryLowPassMeanTs();

	// Reset the golay filters
	m_fir_accX.setBuf(0.0);
	m_fir_accY.setBuf(0.0);
	m_fir_accZ.setBuf(9.81);

	// Initialise the LED command (RGB6 gets overridden on the CM730 to display the USB connection state)
	m_ledCommand.mask = 0x07;
	m_ledCommand.state = 0x00;
	m_ledCommand.rgb5.r = 0.0;
	m_ledCommand.rgb5.g = 0.0;
	m_ledCommand.rgb5.b = 0.0;
	m_ledCommand.rgb5Blink = false;

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
	m_board.reset(new CM730(RESOURCE_PATH, CONFIG_PARAM_PATH, m_servos));
	m_board->updateTxBRPacket(m_cm730_queryset);

	// Stop here if connecting to the CM730 was unsuccessful
	if(m_board->connect() < 0)
	{
		ROS_ERROR("Could not initialize CM730");
		return false;
	}

	// Enable power for the arms (there is a power MOSFET on the CM730 board that controls this)
	m_board->setDynamixelPower(CM730::DYNPOW_ON);

	// Wait for the robot's servos to be relaxed
	if(m_robotNameStr == "xs0")
	{
		ROS_WARN("Robot is xs0 => Automatically disabling use of servos...");
		m_useServos.set(false);
	}
	else
		waitForRelaxedServos();

	// Indicate that we have real hardware at our disposal
	m_haveHardware = true;

	// Return that the CM730 was successfully initialised
	return true;
}

// Wait for relaxed servos (turns on and verifies the dynamixel power register of the CM730)
void RobotInterface::waitForRelaxedServos()
{
	// Constants
	static const double dT = 0.6;

	// Initialise variables
	ros::Time startTime = ros::Time::now(), lastBeep = startTime, now;
	bool hadSuccessfulRun = false, allowFadeButton = false, playedSound = false;
	int numCM730Reads = 0, numCM730ReadFail = 0, numCM730Writes = 0, numCM730WriteFail = 0;

	// Set up a vector to store how often we have heard from each joint
	int numAttempts = 0;
	std::vector<int> numHeard;
	numHeard.resize(m_model->numJoints(), 0);

	// Display an explanatory message to the user
	ROS_INFO("Waiting for all servos to be relaxed...");

	// Wait until no servo that we know about has its torque enabled
	do
	{
		// Sleep a little to give the CM730 and servos a chance
		usleep(10000);

		// Make a noise if this loop is taking too long
		now = ros::Time::now();
		if((now - lastBeep).toSec() > dT)
		{
			int ret = CM730::RET_SUCCESS;
			bool haveFailure = false, criticalFailure = false;
			if(numCM730ReadFail == numCM730Reads && numCM730Reads > 0)
			{
				criticalFailure = true;
				ROS_ERROR("Critical failure: All reads of the CM730 have failed!");
			}
			else if(numAttempts > 0)
			{
				size_t minHeardIndex = 0;
				int minHeard = numAttempts;
				std::ostringstream ss;
				for(size_t i = 0; i < m_model->numJoints(); i++)
				{
					if(numHeard[i] < 0) continue;
					if(numHeard[i] < minHeard)
					{
						minHeard = numHeard[i];
						minHeardIndex = i;
					}
					if(numHeard[i] == 0)
					{
						DXLJoint* joint = dxlJoint(i);
						const std::string& servoName = joint->name;
						int servoID = joint->id();
						if(haveFailure)
							ss << ", ";
						ss << servoName << " (" << servoID << ")";
						haveFailure = true;
						bool isArmServo = (servoName == "left_shoulder_pitch" || servoName == "left_shoulder_roll" || servoName == "left_elbow_pitch" || servoName == "right_shoulder_pitch" || servoName == "right_shoulder_roll" || servoName == "right_elbow_pitch");
						if(!isArmServo)
							criticalFailure = true;
					}
				}
				if(criticalFailure)
					ROS_ERROR("Critical failure: All reads have failed for %s", ss.str().c_str());
				else if(haveFailure)
					ROS_ERROR("Failure: All reads have failed for %s", ss.str().c_str());
				else if(minHeard < numAttempts)
				{
					DXLJoint* joint = dxlJoint(minHeardIndex);
					int numFail = numAttempts - minHeard;
					ROS_INFO("Every servo has been successfully read at least once => Worst servo is %s (%d) with %d of %d failures (%.0f%%)", joint->name.c_str(), joint->id(), numFail, numAttempts, (100.0 * numFail) / numAttempts);
				}
				else
					ROS_INFO("Every servo has been successfully read every time");
			}
			if(criticalFailure)
				ret = m_board->sound(7);
			else if(haveFailure)
				ret = m_board->sound(11);
			else
				ret = m_board->sound(14);
			numCM730Writes++;
			if(ret != CM730::RET_SUCCESS)
			{
				ROS_ERROR("Failed to write a sound command to the CM730! (Error: %d)", ret);
				numCM730WriteFail++;
			}
			lastBeep = now;
			playedSound = true;
		}

		// Read and verify the dynamixel power state
		CM730::DynPowState state = CM730::DYNPOW_UNKNOWN;
		int ret = m_board->getDynamixelPower(state);
		numCM730Reads++;
		if(ret != CM730::RET_SUCCESS)
		{
			ROS_ERROR_THROTTLE(dT, "Failed to read the CM730 dynamixel power register! (Error: %d)", ret);
			numCM730ReadFail++;
			continue;
		}
		else if(state != CM730::DYNPOW_ON)
		{
			ROS_WARN_THROTTLE(dT, "CM730 state is currently %d => Setting it to %d...", state, CM730::DYNPOW_ON);
			numCM730Writes++;
			if(!m_board->setDynamixelPower(CM730::DYNPOW_ON))
				numCM730WriteFail++;
			continue;
		}

		// Query the fade button and continue with robotcontrol if the user thereby says it is ok
		int button = 0;
		ret = m_board->getButtonState(button);
		numCM730Reads++;
		if(ret == CM730::RET_SUCCESS)
		{
			if((button & 0x01) == 0)
				allowFadeButton = true;
			else if(allowFadeButton) // Fade button is currently pressed
			{
				ROS_INFO("The user pressed the fade button to manually indicate that it is ok to continue...");
				break;
			}
		}
		else
		{
			ROS_ERROR_THROTTLE(dT, "Failed to read the CM730 button state! (Error %d)", ret);
			numCM730ReadFail++;
		}

		// Read the torque enabled state of each servo
		bool allServosRelaxed = true, readAtLeastOneServo = (m_model->numJoints() == 0);
		numAttempts++;
		std::ostringstream ss;
		ss << "Servos with torque enabled: ";
		for(size_t i = 0; i < m_model->numJoints(); i++)
		{
			const DXLJoint* joint = dxlJoint(i);
			int servoID = joint->id();
			if(joint->enabled() && servoID >= CM730::ID_MIN && servoID <= CM730::ID_MAX && servoID != CM730::ID_CM730)
			{
				int value = 0;
				if(m_board->readByte(servoID, m_servos->addressTorqueEnable(), &value) == CM730::RET_SUCCESS)
				{
					numHeard[i]++;
					readAtLeastOneServo = true;
					if(value != 0)
					{
						if(!allServosRelaxed)
							ss << ", ";
						ss << servoID;
						allServosRelaxed = false;
					}
				}
			}
			else
				numHeard[i] = -1;
		}

		// Display a warning if we have some servos that are not relaxed
		if(!allServosRelaxed)
			ROS_WARN_THROTTLE(dT, "%s", ss.str().c_str());
		if(!readAtLeastOneServo)
			ROS_WARN_THROTTLE(dT, "Failed to read from any servos just then...");

		// Check whether everything is ok and we can quit
		if(allServosRelaxed && readAtLeastOneServo)
		{
			if(hadSuccessfulRun)
				break;
			else
				hadSuccessfulRun = true;
		}
	}
	while(true);

	// Calculate the time that was waited
	double elapsed = (ros::Time::now() - startTime).toSec();

	// Display some diagnostics about dynamixel communications health
	if(playedSound)
	{
		if(numCM730WriteFail > 0)
		{
			if(numCM730WriteFail >= numCM730Writes)
				ROS_ERROR("%d of %d writes failed for the CM730", numCM730WriteFail, numCM730Writes);
			else
				ROS_WARN("%d of %d writes failed for the CM730", numCM730WriteFail, numCM730Writes);
		}
		if(numCM730ReadFail > 0)
		{
			if(numCM730ReadFail >= numCM730Reads)
				ROS_ERROR("%d of %d reads failed for the CM730", numCM730ReadFail, numCM730Reads);
			else
				ROS_WARN("%d of %d reads failed for the CM730", numCM730ReadFail, numCM730Reads);
		}
		for(size_t i = 0; i < m_model->numJoints(); i++)
		{
			if(numHeard[i] < 0) continue;
			int numFailed = numAttempts - numHeard[i];
			if(numFailed > 0)
			{
				RobotInterface::DXLJoint* joint = dxlJoint(i);
				if(numFailed >= numAttempts)
					ROS_ERROR("%d of %d reads failed for ID %d (%s)", numFailed, numAttempts, joint->id(), joint->name.c_str());
				else
					ROS_WARN("%d of %d reads failed for ID %d (%s)", numFailed, numAttempts, joint->id(), joint->name.c_str());
			}
		}
	}

	// Display an explanatory message to the user
	ROS_INFO("All servos are relaxed (took %.2fs)", elapsed);

	// Stop any currently playing buzzer sound
	int ret = m_board->stopSound();
	if(ret != CM730::RET_SUCCESS)
		ROS_ERROR("Failed to write to the CM730 to stop the buzzer! (Error: %d)", ret);
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

	// Only communicate with the CM730 if we are not using the servos
	if(!m_useServos())
		onlyTryCM730 = true;

	//
	// Bulk read from CM730
	//

	// Communications are occurring PC --> CM730
	m_hadTX = true;

	// Read feedback data from the CM730 and servos
	int ret = readFeedbackData(onlyTryCM730); // Should update m_servoData and m_boardData (only the latter however if onlyTryCM730 is true)
	ros::Time bulkReadTime = ros::Time::now();

	// If the bulk read failed then see what the reason was and react
	if(ret == CM730::RET_SUCCESS)
	{
		// Reset the consecutive failure count
		if(!onlyTryCM730)
			m_consecFailCount = 0;
		
		// Communications have occurred DXL --> CM730
		m_hadRX = !onlyTryCM730;
		m_commsOk = true;
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
					m_board->writeByte(previd, m_servos->prodAddress(), m_servos->prodValue());
				}
				else
#endif
				{
					if(m_showServoFailures())
						ROS_ERROR("Servo death cycle (ID %d) => Prodding servo with a write!", id);
					m_board->writeByte(id, m_servos->prodAddress(), m_servos->prodValue());
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
					m_commsOk = false;
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
				m_board->writeByte(id, m_servos->prodAddress(), m_servos->prodValue());

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
		// Retrieve the feedback from each joint
		for(size_t i = 0; i < m_model->numJoints(); i++)
		{
			// Retrieve pointers to joint and bulk read data
			DXLJoint* joint = dxlJoint(i);
			int idx = joint->id() - 1;
			const BRData& data = m_servoData[idx];

			// Set the feedback time stamp
			joint->feedback.stamp = bulkReadTime; // Note: This line replaces the need to call setJointFeedbackTime()

			// Write either the real feedback or just the commanded values into the joint, depending on the enabled and readFeedback config parameters
			if(joint->enabled() && joint->readFeedback())
			{
				// Convert the feedback to an angular position
				if(data.position >= 0) // Valid position data is always non-negative in terms of ticks
					joint->feedback.pos = (data.position - joint->tickOffset()) / joint->commandGenerator->ticksPerRad();
				else
					joint->feedback.pos = 0.0;

				// Invert the joint position if required
				if(joint->invert())
					joint->feedback.pos = -joint->feedback.pos;

				// Give the command generator the information it needs
				int pValue = (int)(joint->realEffort * m_servos->fullEffort());
				if(pValue < 2) pValue = 2;
				joint->commandGenerator->setPValue(pValue);
				joint->commandGenerator->setVoltage(m_voltage); // Note: This could use m_statVoltage (check if >0.0 before using it), but as statistics are disabled this is a decent drop-in replacement

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

		// Process the joint feedback
		processJointFeedback();
	}

	//
	// Timing
	//
	
	// Retrieve the nominal time step
	const double nominaldT = m_model->timerDuration();

	// Calculate the time since sensor data was last processed
	double dT;
	bool firstSensorData = m_lastSensorTime.isZero();
	if(firstSensorData)
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
	// Temperature
	//

	// Retrieve and filter the board temperature
	double temperature = m_boardData.temp;
	if(firstSensorData)
		m_temperatureLowPass.setValue(temperature);
	else
		m_temperatureLowPass.put(temperature);
	m_temperature = m_temperatureLowPass.value();
	m_model->setTemperature(m_temperature);

	//
	// Voltage
	//

	// Retrieve and filter the board voltage
	double voltage = INT_TO_VOLTS * m_boardData.voltage;
	if(!m_initedVoltage)
	{
		m_voltageLowPass.setValue(voltage);
		m_initedVoltage = (voltage != 0.0);
	}
	else
		m_voltageLowPass.put(voltage);
	m_voltage = m_voltageLowPass.value();
	m_model->setVoltage(m_voltage);

	//
	// IMU orientation offsets
	//

	// Gyroscope/accelerometer orientation offset
	Eigen::Quaterniond orientGyroAcc = rot_conv::QuatFromTilt(m_gyroAccFusedYaw(), m_gyroAccTiltAxisAngle(), m_gyroAccTiltAngle());
	Eigen::Quaterniond orientMag = rot_conv::QuatFromTilt(m_magFusedYaw(), m_magTiltAxisAngle(), m_magTiltAngle() + (m_magFlip() ? M_PI : 0.0));

	//
	// Gyroscope sensor
	//

	// Decide on an appropriate scale factor for the gyro
	double gyroScaleFactor = rc_utils::coerce(rc_utils::interpolateCoerced<double>(m_gyroTemperatureLow(), m_gyroTemperatureHigh(), m_gyroScaleFactorLT(), m_gyroScaleFactorHT(), m_temperature), 0.2, 5.0);
	if(m_gyroCalibrating != 0)
	{
		if(m_gyroCalibScaleFactor <= 0.0)
		{
			m_gyroCalibScaleFactor = gyroScaleFactor; // Note: scaleFactor cannot be zero or negative so this logic works!
			ROS_INFO("Keeping a constant gyro scale factor %.3f and disabling automatic gyro bias calibration...", m_gyroCalibScaleFactor);
		}
		else
			gyroScaleFactor = m_gyroCalibScaleFactor;
	}

	// Retrieve and correct the gyroscope data to be in rad/s
	double gyroX = m_boardData.gyroX * GYRO_SCALE * gyroScaleFactor;
	double gyroY = m_boardData.gyroY * GYRO_SCALE * gyroScaleFactor;
	double gyroZ = m_boardData.gyroZ * GYRO_SCALE * gyroScaleFactor;

	// Correct for the mounting orientation of the gyroscope
	Eigen::Vector3d gyro(gyroX, gyroY, gyroZ);
	gyro = orientGyroAcc * gyro;

	// Update the robot angular velocity in the robot model
	m_model->setRobotAngularVelocity(gyro);

	// Calculate the heavily smoothed mean of the gyro measurements
	if(firstSensorData)
	{
		m_gyroLowPassMean.setValue(gyro);
		m_gyroVeryLowPassMean.setValue(gyro);
	}
	else
	{
		m_gyroLowPassMean.put(gyro);
		m_gyroVeryLowPassMean.put(gyro);
	}
	m_gyroMean = m_gyroLowPassMean.value();
	m_gyroMeanSmooth = m_gyroVeryLowPassMean.value();

	// See for how many cycles the gyro value has been within close range of the gyro mean (i.e. more or less stable)
	double gyroMeanOffset = (gyro - m_gyroMean).norm();
	if(gyroMeanOffset > m_gyroStabilityBound() || firstSensorData)
		m_gyroStableCount = 0;
	else
		m_gyroStableCount++;
	double gyroStableTime = m_gyroStableCount * nominaldT; // We intentionally calculate this time based on cycles and not a difference in ROS time (as the latter doesn't necessitate that there were any measurements inbetween)

	// Set the gyro bias configs to the gyro mean if required
	if(m_attEstGyroBiasSetFM())
		setAttEstGyroBiasFromMean();
	if(m_attEstGyroBiasSetFSM())
		setAttEstGyroBiasFromSmoothMean();

	//
	// Accelerometer sensor
	//

	// Retrieve and scale the accelerometer data to be in m/s^2 (inertial acceleration convention, so nominal is (0,0,9.81))
	double rawAccX = m_boardData.accX * ACC_SCALE;
	double rawAccY = m_boardData.accY * ACC_SCALE;
	double rawAccZ = m_boardData.accZ * ACC_SCALE;
	Eigen::Vector3d rawAcc(rawAccX, rawAccY, rawAccZ);

	// Calculate the heavily smoothed mean of the raw acc measurements (for gyro/acc orientation offset calibration only)
	if(firstSensorData)
		m_accLowPassMean.setValue(rawAcc);
	else
		m_accLowPassMean.put(rawAcc);
	m_accMean = m_accLowPassMean.value();

	// Correct for the mounting orientation of the accelerometer
	rawAcc = orientGyroAcc * rawAcc;

	// Filter the accelerometer data
	m_fir_accX.put(rawAcc.x());
	m_fir_accY.put(rawAcc.y());
	m_fir_accZ.put(rawAcc.z());
	Eigen::Vector3d acc(m_fir_accX.value(), m_fir_accY.value(), m_fir_accZ.value());

	// Update the measured acceleration vector in the robot model
	m_model->setAccelerationVector(acc);

	//
	// Magnetometer sensor
	//

	// Retrieve and scale the magnetometer data to be in gauss
	double magRawX = m_boardData.magX * MAG_SCALE;
	double magRawY = m_boardData.magY * MAG_SCALE;
	double magRawZ = m_boardData.magZ * MAG_SCALE;

	// Correct for the mounting orientation of the magnetometer
	Eigen::Vector3d magRaw(magRawX, magRawY, magRawZ);
	magRaw = orientMag * magRaw; // Rotates the magnetometer vector into the robot frame

	// Initialise the spike filter values if this is the first time we are getting data
	if(firstCM730Data)
	{
		m_magSpikeFilterX.setValue(magRaw.x());
		m_magSpikeFilterY.setValue(magRaw.y());
		m_magSpikeFilterZ.setValue(magRaw.z());
	}

	// Update the last known robot orientation inside the magnetometer filter
	if(m_attEstYaw.QLActive())
		m_magHardIronFilter.clearOrientation();
	else
	{
		double quatTmp[4]; // Format is (w,x,y,z)
		m_attEstYaw.getAttitude(quatTmp);
		m_magHardIronFilter.setOrientation(Eigen::Quaterniond(quatTmp[0], quatTmp[1], quatTmp[2], quatTmp[3]));
	}

	// Pass spike-filtered magnetometer data through the magnetometer filter to account for hard iron effects
	m_magHardIronFilter.update(m_magSpikeFilterX.put(magRaw.x()), m_magSpikeFilterY.put(magRaw.y()), m_magSpikeFilterZ.put(magRaw.z()));

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

	// Auto-calibration of the gyro bias for attitude estimators without yaw feedback
	double gyroBiasTs = 0.0;
	double gyroBiasAlpha = 0.0;
	double gyroStableTimeSinceFade = gyroStableTime - m_gyroCalibFadeTimeStart();
	if(gyroStableTimeSinceFade >= 0.0 && m_gyroEnableAutoCalib() && m_gyroCalibrating == 0)
	{
		if(!m_gyroBiasAdjusting)
		{
			m_gyroVeryLowPassMean.setValue(m_gyroMean);
			m_gyroMeanSmooth = m_gyroVeryLowPassMean.value();
		}
		m_gyroBiasAdjusting = true;
		gyroBiasTs = rc_utils::interpolateCoerced<double>(0.0, m_gyroCalibFadeTimeDur(), m_gyroCalibTsSlow(), m_gyroCalibTsFast(), gyroStableTimeSinceFade);
		gyroBiasAlpha = rc_utils::LowPassFilter::computeAlpha(gyroBiasTs, nominaldT);
		double u = rc_utils::interpolateCoerced<double>(0.0, m_gyroLowPassMeanTsHigh(), 0.0, 1.0, gyroStableTimeSinceFade);
		Eigen::Vector3d gyroBiasTarget = u*m_gyroMeanSmooth + (1.0 - u)*m_gyroMean;
		double b[3] = {0.0};
		m_attEstYaw.getGyroBias(b);
		b[0] += gyroBiasAlpha*(gyroBiasTarget.x() - b[0]);
		b[1] += gyroBiasAlpha*(gyroBiasTarget.y() - b[1]);
		b[2] += gyroBiasAlpha*(gyroBiasTarget.z() - b[2]);
		m_attEstYaw.setGyroBias(b);
		m_attEstNoMag.getGyroBias(b);
		b[0] += gyroBiasAlpha*(gyroBiasTarget.x() - b[0]);
		b[1] += gyroBiasAlpha*(gyroBiasTarget.y() - b[1]);
		b[2] += gyroBiasAlpha*(gyroBiasTarget.z() - b[2]);
		m_attEstNoMag.setGyroBias(b);
	}
	else
		m_gyroBiasAdjusting = false;

	// Update the attitude estimator
	m_attitudeEstimator.update(dT, gyro.x(), gyro.y(), gyro.z(), acc.x(), acc.y(), acc.z(), mag.x(), mag.y(), mag.z()); // We use the filtered magnetometer values here...
	m_attEstNoMag.update(dT, gyro.x(), gyro.y(), gyro.z(), acc.x(), acc.y(), acc.z(), 0.0, 0.0, 0.0);                   // We use no magnetometer values at all here...
	m_attEstYaw.update(dT, gyro.x(), gyro.y(), gyro.z(), acc.x(), acc.y(), acc.z(), 0.0, 0.0, 0.0);                     // We use no magnetometer values at all here...

	// Retrieve the current robot orientation (attitude) estimate
	double quatAtt[4], quatAttNoMag[4], quatAttYaw[4]; // Format is (w,x,y,z)
	m_attitudeEstimator.getAttitude(quatAtt);
	m_attEstNoMag.getAttitude(quatAttNoMag);
	m_attEstYaw.getAttitude(quatAttYaw);

	// Make available the robot orientation estimates
	m_model->setRobotOrientation(Eigen::Quaterniond(quatAtt[0], quatAtt[1], quatAtt[2], quatAtt[3]), Eigen::Quaterniond(quatAttYaw[0], quatAttYaw[1], quatAttYaw[2], quatAttYaw[3])); // The choice of m_attEstYaw vs m_attEstNoMag here should be consistent with the same choice (above) in setting the quaternion orientation inside m_magHardIronFilter!

	//
	// Gyroscope calibration
	//

	// Update variables required for the gyroscope calibration
	if(m_gyroCalibrating != 0)
	{
		double curYaw = m_attEstYaw.fusedYaw();
		if(fabs(curYaw - m_gyroCalibCurYaw) >= M_PI)
		{
			if(curYaw > m_gyroCalibCurYaw)
				m_gyroCalibLoopCount--;
			else
				m_gyroCalibLoopCount++;
		}
		m_gyroCalibCurYaw = curYaw;
		m_gyroCalibUpdateCount++;
	}

	//
	// Fused angle estimation
	//

	// Perform the prediction and correction cycles of the angle estimator
	m_angleEstimator.predict(dT, gyro.x(), gyro.y());
	m_angleEstimator.update(acc.x(), acc.y(), acc.z());

	//
	// Button presses
	//

	// Check which buttons have been pressed and publish appropriate messages
	for(int i = 0; i < 3; i++)
	{
		// Determine the old and new button states
		bool state = (m_boardData.button & (1 << i));
		bool lastState = m_lastButtons & (1 << i);
		bool justReleased = (!state && lastState);
		bool ignorePress = false;
		bool longPress = false;

		// Detect long presses of button 0 to recover from a CM730 reset
		if(i == 0 && m_haveHardware)
		{
			if(state)
			{
				double timeElapsed = (m_buttonTime0.isZero() ? -1.0 : (bulkReadTime - m_buttonTime0).toSec());
				double timeData = ((m_buttonTime0.isZero() || m_lastCM730Time.isZero()) ? -1.0 : (m_lastCM730Time - m_buttonTime0).toSec());
				double timeSinceRec = (m_buttonLastRec0.isZero() ? BUTTON0_RECOVER_PERIOD : (bulkReadTime - m_buttonLastRec0).toSec());
				if(m_buttonState0 <= BPS_RELEASED || m_buttonState0 == BPS_LONG_PRESS || m_buttonState0 >= BPS_COUNT) // Note: BPS_LONG_PRESS is an invalid state for button 0, so it is treated like the other invalid or released states
				{
					m_buttonState0 = BPS_SHORT_PRESS;
					m_buttonTime0 = bulkReadTime;
				}
				else if(m_buttonState0 == BPS_SHORT_PRESS && timeElapsed >= BUTTON0_RELAX_TIME && timeData >= BUTTON0_RELAX_TIME)
				{
					m_buttonState0 = BPS_DO_RELAX;
					m_buttonTime0 = bulkReadTime;
					longPress = true;
				}
				else if(m_buttonState0 == BPS_DO_RELAX && timeElapsed >= BUTTON0_UNRELAX_TIME && timeData >= BUTTON0_UNRELAX_TIME)
				{
					m_buttonState0 = BPS_DO_UNRELAX;
					m_buttonTime0 = bulkReadTime;
					if(!m_model->relaxedWasSet())
					{
						m_model->setRelaxed(false);
						sendFadeTorqueGoal(1.0);
					}
				}
				if(m_buttonState0 == BPS_DO_RELAX || m_buttonState0 == BPS_DO_UNRELAX)
				{
					if(timeSinceRec >= BUTTON0_RECOVER_PERIOD)
					{
						m_board->setDynamixelPower(CM730::DYNPOW_ON);
						m_buttonLastRec0 = bulkReadTime;
					}
				}
				else
					rc_utils::zeroRosTime(m_buttonLastRec0);
				if(m_buttonState0 == BPS_DO_RELAX)
				{
					m_model->setRelaxed(true);
					if(m_model->state() != m_state_relaxed && m_model->state() != m_state_setting_pose)
						m_model->setState(m_state_relaxed);
				}
			}
			else if(justReleased)
			{
				if(m_buttonState0 == BPS_DO_RELAX || m_buttonState0 == BPS_DO_UNRELAX)
				{
					ignorePress = true;
					if(!m_model->relaxedWasSet())
						m_model->setRelaxed(false);
				}
				m_buttonState0 = BPS_RELEASED;
				rc_utils::zeroRosTime(m_buttonTime0);
				rc_utils::zeroRosTime(m_buttonLastRec0);
			}
			if(!longPress && (m_buttonState0 == BPS_SHORT_PRESS || m_buttonState0 == BPS_DO_RELAX || m_buttonState0 == BPS_DO_UNRELAX))
				ignorePress = true;
		}

		// Detect long presses of button 1
		if(i == 1 && m_haveHardware)
		{
			if(state)
			{
				double timeElapsed = (m_buttonTime1.isZero() ? -1.0 : (bulkReadTime - m_buttonTime1).toSec());
				if(m_buttonState1 != BPS_SHORT_PRESS && m_buttonState1 != BPS_LONG_PRESS)
				{
					m_buttonState1 = BPS_SHORT_PRESS;
					m_buttonTime1 = bulkReadTime;
				}
				else if(m_buttonState1 == BPS_SHORT_PRESS && timeElapsed >= BUTTON1_LONG_TIME)
				{
					m_buttonState1 = BPS_LONG_PRESS;
					m_buttonTime1 = bulkReadTime;
					longPress = true;
				}
			}
			else if(justReleased)
			{
				if(m_buttonState1 == BPS_LONG_PRESS)
					ignorePress = true;
				m_buttonState1 = BPS_RELEASED;
				rc_utils::zeroRosTime(m_buttonTime1);
			}
			if(!longPress && (m_buttonState1 == BPS_SHORT_PRESS || m_buttonState1 == BPS_LONG_PRESS))
				ignorePress = true;
		}

		// Determine whether this button has been fake pressed by the config server
		bool fakePressed0 = (i == 0 && m_buttonPress0());
		bool fakePressed1 = (i == 1 && m_buttonPress1());
		bool fakePressed2 = (i == 2 && m_buttonPress2());

		// If the button was just released...
		if((justReleased || longPress || fakePressed0 || fakePressed1 || fakePressed2) && !ignorePress)
		{
			// Publish on a ROS topic that the button was pressed
			Button btn;
			btn.button = i;
			btn.time = bulkReadTime;
			btn.longPress = longPress;
			m_pub_buttons.publish(btn);

			// We react to some buttons directly ourselves
			handleButton(i, longPress);

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
		m_attEstNoMag.getGyroBias(attEstNoMagBias);
		m_PM.plotScalar(attEstNoMagBias[0], PM_ATTEST_NOMAG_BIAS_X);
		m_PM.plotScalar(attEstNoMagBias[1], PM_ATTEST_NOMAG_BIAS_Y);
		m_PM.plotScalar(attEstNoMagBias[2], PM_ATTEST_NOMAG_BIAS_Z);
		m_PM.plotScalar(m_attEstYaw.fusedYaw(), PM_ATTEST_YAW_FYAW);
		m_PM.plotScalar(m_attEstYaw.fusedPitch(), PM_ATTEST_YAW_FPITCH);
		m_PM.plotScalar(m_attEstYaw.fusedRoll(), PM_ATTEST_YAW_FROLL);
		m_PM.plotScalar((m_attEstYaw.fusedHemi() ? 1.0 : -1.0), PM_ATTEST_YAW_FHEMI);
		double attEstYawBias[3] = {0.0};
		m_attEstYaw.getGyroBias(attEstYawBias);
		m_PM.plotScalar(attEstYawBias[0], PM_ATTEST_YAW_BIAS_X);
		m_PM.plotScalar(attEstYawBias[1], PM_ATTEST_YAW_BIAS_Y);
		m_PM.plotScalar(attEstYawBias[2], PM_ATTEST_YAW_BIAS_Z);
		m_PM.plotScalar(gyro.x(), PM_GYRO_X);
		m_PM.plotScalar(gyro.y(), PM_GYRO_Y);
		m_PM.plotScalar(gyro.z(), PM_GYRO_Z);
		m_PM.plotScalar(gyro.norm(), PM_GYRO_N);
		m_PM.plotScalar(m_gyroMean.x(), PM_GYROMEAN_X);
		m_PM.plotScalar(m_gyroMean.y(), PM_GYROMEAN_Y);
		m_PM.plotScalar(m_gyroMean.z(), PM_GYROMEAN_Z);
		m_PM.plotScalar(m_gyroMean.norm(), PM_GYROMEAN_N);
		m_PM.plotScalar(m_gyroMeanSmooth.x(), PM_GYROMEAN_SMOOTH_X);
		m_PM.plotScalar(m_gyroMeanSmooth.y(), PM_GYROMEAN_SMOOTH_Y);
		m_PM.plotScalar(m_gyroMeanSmooth.z(), PM_GYROMEAN_SMOOTH_Z);
		m_PM.plotScalar(m_gyroMeanSmooth.norm(), PM_GYROMEAN_SMOOTH_N);
		m_PM.plotScalar(gyroMeanOffset, PM_GYRO_MEAN_OFFSET);
		m_PM.plotScalar(gyroStableTime, PM_GYRO_STABLE_TIME);
		m_PM.plotScalar(gyroBiasTs, PM_GYRO_BIAS_TS);
		m_PM.plotScalar(gyroBiasAlpha, PM_GYRO_BIAS_ALPHA);
		m_PM.plotScalar(gyroScaleFactor, PM_GYRO_SCALE_FACTOR);
		m_PM.plotScalar(rawAcc.x(), PM_ACC_XRAW);
		m_PM.plotScalar(rawAcc.y(), PM_ACC_YRAW);
		m_PM.plotScalar(rawAcc.z(), PM_ACC_ZRAW);
		m_PM.plotScalar(rawAcc.norm(), PM_ACC_NRAW);
		m_PM.plotScalar(acc.x(), PM_ACC_X);
		m_PM.plotScalar(acc.y(), PM_ACC_Y);
		m_PM.plotScalar(acc.z(), PM_ACC_Z);
		m_PM.plotScalar(acc.norm(), PM_ACC_N);
		m_PM.plotScalar(m_accMean.x(), PM_ACCMEAN_X);
		m_PM.plotScalar(m_accMean.y(), PM_ACCMEAN_Y);
		m_PM.plotScalar(m_accMean.z(), PM_ACCMEAN_Z);
		m_PM.plotScalar(m_accMean.norm(), PM_ACCMEAN_N);
		m_PM.plotScalar(magRaw.x(), PM_MAG_XRAW);
		m_PM.plotScalar(magRaw.y(), PM_MAG_YRAW);
		m_PM.plotScalar(magRaw.z(), PM_MAG_ZRAW);
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
		m_PM.plotScalar(m_temperature * 0.01, PM_TEMPERATURE);
		m_PM.plotScalar(m_voltage, PM_VOLTAGE);
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
 * List the joints of the robot and their properties.
 **/
bool RobotInterface::handleListJoints(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	// Vector for joints sorted by ID
	std::vector<DXLJoint*> sorted;

	// List information about all joints in RobotModel
	ROS_INFO(" ");
	ROS_WARN("All joints in RobotModel:");
	for(size_t i = 0; i < m_model->numJoints(); i++)
	{
		DXLJoint* joint = dxlJoint(i);
		ROS_INFO("Joint %2d: ID %-3d %-33s ==> Ticks %-4d Type %s%s%s%s", (int) i, joint->id(), joint->name.c_str(), joint->tickOffset(), joint->type().c_str(), (joint->enabled() ? "" : " DISABLED"), (joint->readFeedback() ? "" : " NOREAD"), (joint->invert() ? " INVERT" : ""));
		sorted.push_back(joint);
	}
	ROS_INFO(" ");
	ROS_WARN("Sorted by ID:");
	std::sort(sorted.begin(), sorted.end(), [](DXLJoint* a, DXLJoint* b) -> bool { return (a->id() < b->id()); });
	for(std::size_t i = 0; i < sorted.size(); i++)
	{
		DXLJoint* joint = sorted[i];
		ROS_INFO("ID %-3d %-33s ==> Ticks %-4d Type %s%s%s%s", joint->id(), joint->name.c_str(), joint->tickOffset(), joint->type().c_str(), (joint->enabled() ? "" : " DISABLED"), (joint->readFeedback() ? "" : " NOREAD"), (joint->invert() ? " INVERT" : ""));
	}
	ROS_INFO(" ");

	// Return that the service was successfully handled
	return true;
}

/**
 * Callback that processes commands for the LED display.
 **/
void RobotInterface::handleLEDCommand(const LEDCommandConstPtr& cmd)
{
	// Update the individual LED states
	for(int i = 0; i < 3; i++)
	{
		// Only update the LED if the corresponding bit is set
		if(cmd->mask & (1 << i))
		{
			// Set/reset the bit so it matches the bit in m_ledCommand.state
			if(cmd->state & (1 << i))
				m_ledCommand.state |= (1 << i);
			else
				m_ledCommand.state &= ~(1 << i);
		}
	}

	// Update the RGBLED colors if requested (Note: RGBLED6 is set by the CM730 firmware and cannot be set in software)
	if(cmd->mask & LEDCommand::LED5)
	{
		m_ledCommand.rgb5 = cmd->rgb5;
		m_ledCommand.rgb5Blink = cmd->rgb5Blink;
	}
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

		// Set the ID of the target CM730
		paramData->id = CM730::ID_CM730;

		// Transcribe the current LED commands (on/off) to the paramData struct
		// LED_MANAGE => 0x01, LED_EDIT => 0x02, LED_PLAY => 0x04 (see 'cm730/firmware/CM730_HW/inc/led.h')
		paramData->led_panel = m_ledCommand.state & 0x07;

		// Set the desired colors of the RGB LEDs
		// The 16-bit rgbled value is <0-4> = Red, <5-9> = Green, <10-14> = Blue, <15> = 0
		paramData->rgbled5 = (((int)(m_ledCommand.rgb5.r * 31) & 0x1F) << 0)
		                   | (((int)(m_ledCommand.rgb5.g * 31) & 0x1F) << 5)
		                   | (((int)(m_ledCommand.rgb5.b * 31) & 0x1F) << 10)
		                   | (m_ledCommand.rgb5Blink != 0 ? 1 << 15 : 0);

		// Perform a write of the LED data to the CM730
		if(m_board->writeData(paramData->id, CM730::P_LED_PANEL, &params[1], sizeof(CM730LedWriteData)-1) != CM730::RET_SUCCESS) // Note: We use &params[1] as we wish to skip the first byte that just contains the target ID
			ROS_ERROR_THROTTLE(0.4, "Write of LED command to CM730 failed!");
	}
	
	// Publish the current LED state on a topic
	nimbro_op_interface::LEDCommand ledState = m_ledCommand;
	ledState.mask = 0x3F; // MNG => 0x01, EDIT => 0x02, PLAY => 0x04, RGBLED5 => 0x08, RX => 0x10, TX => 0x20
	ledState.state &= ~(LEDCommand::LED0 | LEDCommand::LED1);
	if(m_hadRX) ledState.state |= LEDCommand::LED0;
	if(m_hadTX) ledState.state |= LEDCommand::LED1;
	m_hadRX = m_hadTX = false;
	ledState.state &= ledState.mask;
	m_pub_led_state.publish(ledState);
}

/**
 * Process joint commands right after the motion modules generate them
 **/
void RobotInterface::processJointCommands()
{
	// Apply the joint command dependencies
	for(const boost::function<void ()>& executeDependency : m_jointCommandDependencies)
		executeDependency();

	// Apply joint aliases to the commands
	for(const JointAliasType& JA : m_jointAliases)
	{
		JA.alias->cmd.pos = JA.source->cmd.pos * JA.multiplier + JA.offset;
		JA.alias->cmd.vel = JA.source->cmd.vel * JA.multiplier;
		JA.alias->cmd.acc = JA.source->cmd.acc * JA.multiplier;
		JA.alias->cmd.effort = JA.source->cmd.effort;
		JA.alias->cmd.raw = JA.source->cmd.raw;
	}
}

/**
 * Process joint torques right after the inverse dynamics generate them
 **/
void RobotInterface::processJointTorques()
{
	// Apply the joint torque dependencies
	for(const boost::function<void ()>& executeDependency : m_jointTorqueDependencies)
		executeDependency();

	// Apply joint aliases to the torques
	for(const JointAliasType& JA : m_jointAliases)
		JA.alias->feedback.modelTorque = JA.source->feedback.modelTorque / JA.multiplier;
}

/**
 * Process joint feedback right after it is collected
 **/
void RobotInterface::processJointFeedback()
{
	// Apply the joint feedback dependencies
	for(const boost::function<void ()>& executeDependency : m_jointFeedbackDependencies)
		executeDependency();

	// Apply joint aliases to the read feedback
	for(const JointAliasType& JA : m_jointAliases)
	{
		JA.alias->feedback.pos = JA.source->feedback.pos * JA.multiplier + JA.offset;
		JA.alias->feedback.torque = JA.source->feedback.torque / JA.multiplier;
	}
}

/**
 * @internal Perform a servo command sync write (via the CM730).
 **/
bool RobotInterface::sendJointTargets()
{
	// Send the buzzer command if we have one
	if(m_haveBuzzerData && m_haveHardware)
	{
		if(m_board->writeData(CM730::ID_CM730, CM730::P_BUZZER_PLAY_LENGTH, &m_buzzerData, sizeof(m_buzzerData)) != CM730::RET_SUCCESS)
			ROS_ERROR_THROTTLE(0.4, "Write of buzzer command to CM730 failed!");
		m_haveBuzzerData = false;
	}

	// Don't send anything if we aren't using the servos
	if(!m_useServos())
		return true;

	// Communications are occurring PC --> CM730
	m_hadTX = true;

	// Compile the required joint commands
	std::vector<JointCmdData> jointCmdData;
	for(size_t i = 0; i < m_model->numJoints(); i++)
	{
		// Retrieve a pointer to the corresponding DXL joint
		DXLJoint* joint = dxlJoint(i);

		// Do not sync write to servos that are not enabled
		if(!joint->enabled()) continue;

		// Slope-limit the servo effort to avoid large instantaneous steps
		joint->realEffort = rc_utils::slopeLimited<double>(joint->realEffort, joint->cmd.effort, m_effortSlopeLimit());
		if(joint->realEffort < 0.0) joint->realEffort = 0.0;
		else if(joint->realEffort > 4.0) joint->realEffort = 4.0;

		// Convert effort to a P gain of the servos
		int realpValue = (int)(joint->realEffort * m_servos->fullEffort());

		// Give the command generator the information it needs
		int pValue = realpValue;
		if(pValue < 2) pValue = 2;
		joint->commandGenerator->setPValue(pValue);
		joint->commandGenerator->setVoltage(m_voltage); // Note: This could use m_statVoltage (check if >0.0 before using it), but as statistics are disabled this is a decent drop-in replacement

		// If requested, use the servo model to generate the final position command
		// We create a linear mix between the raw command (goal position) and
		// the command generated by the servo model. The slope of the coefficient
		// is limited by the m_rawStateSlopeLimit().
		double rawGoal = ((!joint->cmd.raw && useModel()) ? 1 : 0);
		joint->rawState = rc_utils::slopeLimited<double>(joint->rawState, rawGoal, m_rawStateSlopeLimit()); // joint->rawState is a dimensionless parameter used to interpolate between a given raw command and servo model based command
		double modelCmd = joint->commandGenerator->servoCommandFor(joint->cmd.pos, joint->cmd.vel, joint->cmd.acc, joint->feedback.modelTorque);
		double rawPosCmd = joint->cmd.pos;
		rawPosCmd = joint->rawState * modelCmd + (1.0 - joint->rawState) * rawPosCmd;

		// Save the generated command for plotting and logging
		joint->cmd.rawPos = rawPosCmd;

		// Negate the command if the joint's invert flag is set
		if(joint->invert())
			rawPosCmd = -rawPosCmd;

		// Convert the command into a servo position in ticks
		int goal = (int) (joint->commandGenerator->ticksPerRad()*rawPosCmd + joint->tickOffset() + 0.5);

		// Goal position saturation
		int minTicks = joint->commandGenerator->minTickValue();
		int maxTicks = joint->commandGenerator->maxTickValue();
		if(goal > maxTicks)
			goal = maxTicks;
		else if(goal < minTicks)
			goal = minTicks;

		// Save the required joint command
		jointCmdData.push_back(JointCmdData(joint->id(), realpValue, goal));
	}

	// Sync write the generated joint commands via the CM730
	if(!syncWriteJointTargets(jointCmdData))
		return false;

	// Return success
	return true;
}

// Sync write the required joint commands via the CM730
bool RobotInterface::syncWriteJointTargets(const std::vector<JointCmdData>& jointCmdData)
{
	// Relaxed robots won't do anything. They are lazy.
	if(m_relaxed || m_board->isSuspended()) return true;

	// Enable the torque on the servos if required
	if(m_enableTorque && m_servos->type == DynamixelBase::X_SERVOS)
	{
		// Allocate memory for the packet data and map an array of TorqueEnableSyncWriteData structs onto it
		std::vector<uint8_t> params(m_model->numJoints() * sizeof(TorqueEnableSyncWriteData));
		TorqueEnableSyncWriteData* paramData = (TorqueEnableSyncWriteData*) &params[0];

		// Populate the struct array with the required data
		size_t count = 0;
		for(size_t i = 0; i < m_model->numJoints(); i++)
		{
			const DXLJoint* joint = dxlJoint(i);
			if(!joint->enabled()) continue;
			paramData[count].id = joint->id();
			paramData[count].torque_enable = 1; // 0 => Off, 1 => On
			count++;
		}

		// Perform a sync write of the generated packet to the servos via the CM730
		if(syncWriteTorqueEnable(count, &params[0]))
			m_enableTorque = false;
	}

	// Sync write the required joint commands
	if(m_servos->type == DynamixelBase::MX_SERVOS)
	{
		// Allocate memory for the packet data and map an array of structs onto it
		std::vector<uint8_t> params(jointCmdData.size() * sizeof(DynamixelMX::JointCmdSyncWriteData));
		DynamixelMX::JointCmdSyncWriteData* paramData = (DynamixelMX::JointCmdSyncWriteData*) &params[0];

		// Populate the sync write data as required
		for(size_t i = 0; i < jointCmdData.size(); i++)
		{
			paramData[i].id = (uint8_t) jointCmdData[i].id;
			paramData[i].p_gain = (uint8_t) jointCmdData[i].p_gain;
			paramData[i].goal_position = (uint16_t) jointCmdData[i].goal_position;
		}

		// Sync write the required joint commands
		if(m_board->syncWrite(DynamixelMX::P_P_GAIN, sizeof(DynamixelMX::JointCmdSyncWriteData)-1, jointCmdData.size(), &params[0]) != CM730::RET_SUCCESS) // Note: Size minus 1 as the id byte doesn't count as a write data byte
		{
			ROS_ERROR("SyncWrite of MX servo joint commands failed!");
			return false;
		}
	}
	else if(m_servos->type == DynamixelBase::X_SERVOS)
	{
		// Allocate memory for the packet data and map arrays of structs onto it
		std::vector<uint8_t> paramsPG(jointCmdData.size() * sizeof(DynamixelX::JointCmdSyncWriteDataPG));
		std::vector<uint8_t> paramsGP(jointCmdData.size() * sizeof(DynamixelX::JointCmdSyncWriteDataGP));
		DynamixelX::JointCmdSyncWriteDataPG* paramDataPG = (DynamixelX::JointCmdSyncWriteDataPG*) &paramsPG[0];
		DynamixelX::JointCmdSyncWriteDataGP* paramDataGP = (DynamixelX::JointCmdSyncWriteDataGP*) &paramsGP[0];

		// Populate the sync write data as required
		for(size_t i = 0; i < jointCmdData.size(); i++)
		{
			paramDataPG[i].id = paramDataGP[i].id = (uint8_t) jointCmdData[i].id;
			paramDataPG[i].p_gain = (uint16_t) jointCmdData[i].p_gain;
			paramDataGP[i].goal_position = (uint32_t) jointCmdData[i].goal_position;
		}

		// Sync write the required joint commands
		if(m_board->syncWrite(DynamixelX::P_POSITION_P_GAIN_L, sizeof(DynamixelX::JointCmdSyncWriteDataPG)-1, jointCmdData.size(), &paramsPG[0]) != CM730::RET_SUCCESS) // Note: Size minus 1 as the id byte doesn't count as a write data byte
		{
			ROS_ERROR("SyncWrite of X servo P gain joint commands failed!");
			return false;
		}
		if(m_board->syncWrite(DynamixelX::P_GOAL_POSITION_0, sizeof(DynamixelX::JointCmdSyncWriteDataGP)-1, jointCmdData.size(), &paramsGP[0]) != CM730::RET_SUCCESS) // Note: Size minus 1 as the id byte doesn't count as a write data byte
		{
			ROS_ERROR("SyncWrite of X servo goal position joint commands failed!");
			return false;
		}
	}
	else
		return false;

	// Return success
	return true;
}

/**
 * @internal Perform a servo sync write (via the CM730) to configure the permitted servo torques.
 **/
bool RobotInterface::setStiffness(float torque)
{
	// Display a throttled info message that fading is active
	if(!(m_relaxed && torque == 0.0))
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
		size_t count = 0;
		for(size_t i = 0; i < m_model->numJoints(); i++)
		{
			const DXLJoint* joint = dxlJoint(i);
			if(!joint->enabled()) continue;
			paramData[count].id = joint->id();
			paramData[count].torque_enable = 0; // 0 => Off, 1 => On
			count++;
		}

		// Perform a sync write of the generated packet to the servos via the CM730
		syncWriteTorqueEnable(count, &params[0]);

		// Update the relaxed flag
		m_relaxed = true;
		m_enableTorque = false;
	}
	else
	{
		// Write the status return level of X servos at the beginning of fade in
		if(m_relaxed && m_servos->type == DynamixelBase::X_SERVOS)
		{
			// Allocate memory for the packet data and map an array of ReturnLevelSyncWriteData structs onto it
			std::vector<uint8_t> params(m_model->numJoints() * sizeof(ReturnLevelSyncWriteData));
			ReturnLevelSyncWriteData* paramData = (ReturnLevelSyncWriteData*) &params[0];

			// Populate the struct array with the required data
			size_t count = 0;
			for(size_t i = 0; i < m_model->numJoints(); i++)
			{
				const DXLJoint* joint = dxlJoint(i);
				if(!joint->enabled()) continue;
				paramData[count].id = joint->id();
				paramData[count].return_level = 1; // 1 => Respond to READ and PING only
				count++;
			}

			// Perform a sync write of the generated packet to the servos via the CM730
			syncWriteReturnLevel(count, &params[0]);

			// Set that the torque should be enabled on the next write of joint commands
			m_enableTorque = true;
		}

		// Update the relaxed flag
		m_relaxed = false;
	}

	// Coerce the torque variable
	if(torque > 1.0) torque = 1.0;

	// Allocate memory for the packet data and map an array of TorqueLimitSyncWriteData structs onto it
	std::vector<uint8_t> params(m_model->numJoints() * sizeof(TorqueLimitSyncWriteData));
	TorqueLimitSyncWriteData* paramData = (TorqueLimitSyncWriteData*) &params[0];

	// Populate the struct array with the required data
	size_t count = 0;
	for(size_t i = 0; i < m_model->numJoints(); i++)
	{
		const DXLJoint* joint = dxlJoint(i);
		if(!joint->enabled()) continue;
		paramData[count].id = joint->id();
		paramData[count].torque_limit = (uint16_t)(torque * m_servos->fullTorque()); // 0 => Limit to 0% of maximum available torque, fullTorque => Limit to 100% of maximum available torque
		count++;
	}

	// Perform a sync write of the generated packet to the servos via the CM730
	if(!syncWriteTorqueLimit(count, &params[0]))
		return false;

	// Return success
	return true;
}

// Sync write a status return level command to the CM730
bool RobotInterface::syncWriteReturnLevel(size_t numDevices, const uint8_t* data)
{
	// Sync write the appropriate packet
	if(m_board->syncWrite(m_servos->addressReturnLevel(), sizeof(ReturnLevelSyncWriteData)-1, numDevices, data) != CM730::RET_SUCCESS) // Note: Size minus 1 as the id byte doesn't count as a write data byte
	{
		ROS_ERROR("SyncWrite of status return level packet failed!");
		return false;
	}
	else return true;
}

// Sync write a torque enable command to the CM730
bool RobotInterface::syncWriteTorqueEnable(size_t numDevices, const uint8_t* data)
{
	// Sync write the appropriate packet
	if(m_board->syncWrite(m_servos->addressTorqueEnable(), sizeof(TorqueEnableSyncWriteData)-1, numDevices, data) != CM730::RET_SUCCESS) // Note: Size minus 1 as the id byte doesn't count as a write data byte
	{
		ROS_ERROR("SyncWrite of torque enable/disable packet failed!");
		return false;
	}
	else return true;
}

// Sync write a torque limit command to the CM730
bool RobotInterface::syncWriteTorqueLimit(size_t numDevices, const uint8_t* data)
{
	// Sync write the appropriate packet
	if(m_board->syncWrite(m_servos->addressTorqueLimit(), sizeof(TorqueLimitSyncWriteData)-1, numDevices, data) != CM730::RET_SUCCESS) // Note: Size minus 1 as the id byte doesn't count as a write data byte
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
		if(m_haveHardware && m_model->numJoints() >= 1)
		{
// 			// Note: This block has been commented out in an attempt to minimise communications issues, and would need to be adjusted to deal with MX vs. X servos!
// 
// 			// Increment (and if required wrap) the statistics servo ID
// 			if(++m_statIndex >= m_model->numJoints())
// 				m_statIndex = 0;
// 
// 			// Retrieve the joint that we are going to pick on this time
// 			DXLJoint* joint = dxlJoint(m_statIndex);
// 
// 			// Query the servo for the required statistics data
// 			if(joint->enabled() && joint->readFeedback())
// 			{
// 				StatisticsReadData statReadData;
// 				if(m_board->readData(joint->id(), DynamixelMX::P_PRESENT_VOLTAGE, &statReadData, sizeof(statReadData)) == CM730::SUCCESS)
// 				{
// 					joint->voltage = statReadData.voltage;
// 					joint->temperature = statReadData.temperature;
// 				}
// 			}
// 
// 			// Apply joint aliases to the read statistics
// 			for(const JointAliasType& JA : m_jointAliases)
// 			{
// 				JA.alias->voltage = JA.source->voltage;
// 				JA.alias->temperature = JA.source->temperature;
// 			}
// 
// 			// Average all the servo voltages to get a single 'current voltage'
// 			int count = 0;
// 			m_statVoltage = 0.0;
// 			for(size_t i = 0; i < m_model->numJoints(); i++)
// 			{
// 				DXLJoint* jnt = dxlJoint(i);
// 				if(jnt->enabled() && jnt->readFeedback() && jnt->voltage > 0)
// 				{
// 					m_statVoltage += INT_TO_VOLTS * jnt->voltage;
// 					count++;
// 				}
// 			}
// 			if(count > 0)
// 				m_statVoltage /= count;
// 			else
// 				m_statVoltage = -1.0;
		}

		// Send the latest LED commands
		sendCM730LedCommand();
	}
	else // Robotcontrol is paused or we have hardware and the board is suspended
	{
		// Turn off the simulated RX/TX led states
		nimbro_op_interface::LEDCommand ledState;
		ledState.mask = 0x30; // MNG => 0x01, EDIT => 0x02, PLAY => 0x04, RGBLED5 => 0x08, RX => 0x10, TX => 0x20
		ledState.state = 0x00;
		m_pub_led_state.publish(ledState);
	}

	// Update the logger heartbeat
	sendLoggerHeartbeat();
}

/**
 * Reports the battery voltage (mean of all servo voltages) and the maximum
 * temperature measured in the statistics callback over all servos.
 **/
void RobotInterface::getDiagnostics(robotcontrol::DiagnosticsPtr diag)
{
	// Retrieve the battery voltage as measured by the CM730
	diag->batteryVoltage = m_voltage; // Note: m_voltage must be initialised in the constructor, just in case, so this never references an indeterminate value
	diag->commsOk = m_commsOk;

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

		// Do not attempt to read offsets from servos that are not enabled
		if(!joint->enabled()) continue;

		// Get the present servo position
		int present_pos;
		if(m_board->readWord(joint->id(), m_servos->addressPresentPos(), &present_pos) != CM730::RET_SUCCESS) continue;

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
	if(!joint || !joint->enabled()) return false;

	// Get the present servo position
	int present_pos;
	if(m_board->readWord(joint->id(), m_servos->addressPresentPos(), &present_pos) != CM730::RET_SUCCESS)
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

// Handle reset of gyro/acc orientation offset
void RobotInterface::handleResetGyroAccOffset()
{
	// Reset the offset if required
	if(m_resetGyroAccOffset())
	{
		m_resetGyroAccOffset.set(false);
		m_gyroAccFusedYaw.set(0.0);
		m_gyroAccTiltAngle.set(0.0);
		m_gyroAccTiltAxisAngle.set(0.0);
	}
}

// Handle reset of mag orientation offset
void RobotInterface::handleResetMagOffset()
{
	// Reset the offset if required
	if(m_resetMagOffset())
	{
		m_resetMagOffset.set(false);
		m_magFlip.set(false);
		m_magFusedYaw.set(0.0);
		m_magTiltAngle.set(0.0);
		m_magTiltAxisAngle.set(0.0);
	}
}

/**
 * Handler for the calibrate gyro/acc IMU offsets start service.
 **/
bool RobotInterface::handleCalibGyroAccStart(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	// Inform the user that a calibration has started
	if(m_imuOffsetsGACalib)
		ROS_WARN("A gyro/acc offsets calibration was already running => Aborting and starting again...");
	else
		ROS_WARN("Starting a gyro/acc offsets calibration...");

	// Set the calibrating flag
	m_imuOffsetsGACalib = true;

	// Reset the calibration data points
	m_imuOffsetsGAData.clear();

	// Return that the service was successfully handled
	return true;
}

/**
 * Handler for the calibrate gyro/acc IMU offsets now service.
 **/
bool RobotInterface::handleCalibGyroAccNow(CalibGyroAccNowRequest& req, CalibGyroAccNowResponse& resp)
{
	// Don't do anything if a calibration is not running
	if(!m_imuOffsetsGACalib)
	{
		ROS_ERROR("Ignoring CalibGyroAccNow service call: No gyro/acc calibration is running!");
		return false;
	}

	// Error checking
	if(req.poseType < 0 || req.poseType >= GAOMT_COUNT)
	{
		ROS_ERROR("Ignoring CalibGyroAccNow service call: Invalid pose type (%d)!", (int)req.poseType);
		return false;
	}

	// Add the data point to our list
	GyroAccOffsetMeas gaom;
	gaom.type = (GyroAccOffsetMeasType) req.poseType;
	gaom.acc = m_accMean;
	m_imuOffsetsGAData.push_back(gaom);

	// Inform the user that a data point was received
	ROS_INFO("Added gyro/acc calib data point: Type %d, Acc(%.2f, %.2f, %.2f)", (int)gaom.type, gaom.acc.x(), gaom.acc.y(), gaom.acc.z());

	// Populate the service response
	resp.numDataPoints = m_imuOffsetsGAData.size();
	resp.accX = gaom.acc.x();
	resp.accY = gaom.acc.y();
	resp.accZ = gaom.acc.z();

	// Return that the service was successfully handled
	return true;
}

/**
 * Handler for the calibrate gyro/acc IMU offsets stop service.
 **/
bool RobotInterface::handleCalibGyroAccStop(CalibGyroAccStopRequest& req, CalibGyroAccStopResponse& resp)
{
	// Don't do anything if a calibration is not running
	if(!m_imuOffsetsGACalib)
	{
		ROS_ERROR("Ignoring CalibGyroAccStop service call: No gyro/acc calibration is running!");
		return false;
	}

	// Reset the calibrating flag
	m_imuOffsetsGACalib = false;

	// Indicate to the user that this service call has been received
	ROS_INFO("Stopping gyro/acc calibration with %d data points...", (int)m_imuOffsetsGAData.size());

	// Average out the readings of each type
	size_t numData[GAOMT_COUNT] = {0};
	Eigen::Vector3d meanData[GAOMT_COUNT];
	for(size_t i = 0; i < GAOMT_COUNT; i++)
		meanData[i].setZero();
	for(size_t i = 0; i < m_imuOffsetsGAData.size(); i++)
	{
		const GyroAccOffsetMeas& data = m_imuOffsetsGAData[i];
		if(data.type < 0 || data.type >= GAOMT_COUNT || data.acc.isZero()) continue;
		meanData[data.type] += (data.acc - meanData[data.type]) / (++numData[data.type]);
	}

	// Check the validity of the averaged upright measurement
	double uprightAccNorm = meanData[GAOMT_UPRIGHT].norm();
	if(numData[GAOMT_UPRIGHT] <= 0 || uprightAccNorm <= 0.0)
	{
		if(numData[GAOMT_UPRIGHT] <= 0)
			ROS_ERROR("Computation of gyro/acc calibration failed: No data for the upright pose!");
		else
			ROS_ERROR("Computation of gyro/acc calibration failed: Upright data points are invalid!");
		return false;
	}

	// Calibrate the gyro/acc tilt angle and tilt axis angle
	Eigen::Vector3d SzB = meanData[GAOMT_UPRIGHT] / uprightAccNorm; // Positive z axis of the robot (body-fixed frame B) in the coordinates of the sensor (frame S)
	double tiltAxisAngle, tiltAngle;                                // Tilt axis angle and tilt angle of the rotation from B to S
	rot_conv::TiltFromZVec(SzB, tiltAxisAngle, tiltAngle);

	// Compute the purported x directions as supported by the various types of data
	size_t dirnCount = 0;
	Eigen::Vector3d SxB = Eigen::Vector3d::Zero();
	double minNorm = uprightAccNorm*(1.0 - m_gyroAccMaxRelAccScale());
	double maxNorm = uprightAccNorm*(1.0 + m_gyroAccMaxRelAccScale());
	for(size_t type = GAOMT_UPRIGHT + 1; type < GAOMT_COUNT; type++)
	{
		// Ignore this data type if no measurements of that type were made
		if(numData[type] <= 0) continue;

		// Normalise the averaged data for this data type
		double norm = meanData[type].norm();
		if(norm <= 0.0 || norm < minNorm || norm > maxNorm)
		{
			ROS_WARN("Ignored measurements of type %d as the AVERAGED measurement value had a norm %.2f out of the range [%.2f,%.2f]!", (int)type, norm, minNorm, maxNorm);
			continue;
		}

		// Project the averaged data to the plane perpendicular to the accepted robot "up" direction
		double dotProd = meanData[type].dot(SzB);
		Eigen::Vector3d projData = meanData[type] - dotProd*SzB;

		// Adjust the projected vector to point to the purported positive x axis
		if(type == GAOMT_FRONT)
			projData = -projData;
		else if(type == GAOMT_RIGHT)
			projData = projData.cross(SzB);
		else if(type == GAOMT_LEFT)
			projData = SzB.cross(projData);
		else if(type != GAOMT_BACK)
		{
			ROS_WARN("Encountered unexpected type %d!", (int)type);
			continue;
		}

		// Normalise the purported x direction calculated for this data type
		double projNorm = projData.norm();
		if(projNorm <= 0.0 || projNorm < minNorm || projNorm > maxNorm)
		{
			ROS_WARN("Ignored measurements of type %d as the PROJECTED averaged measurement value had a norm %.2f out of the range [%.2f,%.2f] (projection angle %.3f)!", (int)type, projNorm, minNorm, maxNorm, fabs(asin(rc_utils::coerceAbs(dotProd/norm, 1.0))));
			continue;
		}
		projData /= projNorm; // Unit vector pointing in the purported x direction

		// Sum up the projected data
		SxB += projData;
		dirnCount++;
	}

	// Calibrate the gyro/acc fused yaw
	double fusedYaw = 0.0;
	if(dirnCount <= 0)
		ROS_WARN("Could not calibrate a value for the gyro/acc fused yaw orientation because only type 0 upright data was available => Setting it to zero!");
	else
	{
		// Compute the circular mean of the purported x directions
		SxB /= dirnCount;

		// Calculate the fused yaw orientation of the gyro/acc sensors
		double SxBnorm = SxB.norm();
		if(SxBnorm < m_gyroAccMinYawAgreement() || SxBnorm <= 0.0)
			ROS_WARN("Could not calibrate a value for the gyro/acc fused yaw orientation because the %d directions disagreed too much (%.3f < %.2f) => Setting it to zero!", (int)dirnCount, SxBnorm, m_gyroAccMinYawAgreement());
		else
		{
			ROS_INFO("Direction agreement for fused yaw (%d directions): %.1f%%", (int)dirnCount, 100.0*SxBnorm);
			SxB /= SxBnorm;
			Eigen::Vector3d SyB = SzB.cross(SxB);
			Eigen::Matrix3d BRS;
			BRS << SxB.x(), SxB.y(), SxB.z(), SyB.x(), SyB.y(), SyB.z(), SzB.x(), SzB.y(), SzB.z(); // BRS = Matrix with rows SxB, SyB, SzB
			double tmpTiltAxisAngle = 0.0, tmpTiltAngle = 0.0;
			rot_conv::TiltFromRotmat(BRS, fusedYaw, tmpTiltAxisAngle, tmpTiltAngle);
			double tiltAxisAngleErr = fabs(tmpTiltAxisAngle - tiltAxisAngle);
			double tiltAngleErr = fabs(tmpTiltAngle - tiltAngle);
			if(tiltAxisAngleErr > 1e-14 || tiltAngleErr > 1e-14)
				ROS_WARN("Disagreement in calculation of the tilt rotation (%.2g, %.2g) => Should never happen!", tiltAxisAngleErr, tiltAngleErr); 
		}
	}

	// Update the parameters on the config server
	m_gyroAccFusedYaw.set(fusedYaw);
	m_gyroAccTiltAxisAngle.set(tiltAxisAngle);
	m_gyroAccTiltAngle.set(tiltAngle);

	// Populate the service response
	resp.numDataPoints = m_imuOffsetsGAData.size();
	resp.numDirections = dirnCount;
	resp.gyroAccFusedYaw = fusedYaw;
	resp.gyroAccTiltAxisAngle = tiltAxisAngle;
	resp.gyroAccTiltAngle = tiltAngle;

	// Report the results of the calibration
	ROS_INFO("Calibrated tilt angles orientation of the gyro/acc: (%.4f, %.4f, %.4f)", fusedYaw, tiltAxisAngle, tiltAngle);

	// Return that the service was successfully handled
	return true;
}

/**
 * Transcribe the value of the acc-only method flag on the config server to the internals of the attitude estimator
 **/
void RobotInterface::updateAttEstMethod()
{
	// Update the acc-only resolution method
	stateestimation::AttitudeEstimator::AccMethodEnum method = (m_attEstUseFusedMethod() ? stateestimation::AttitudeEstimator::ME_FUSED_YAW : stateestimation::AttitudeEstimator::ME_ZYX_YAW);
	m_attitudeEstimator.setAccMethod(method);
	m_attEstNoMag.setAccMethod(method);
	m_attEstYaw.setAccMethod(method);
}

/**
 * Transcribe the values from the attEstPIGains config server parameters to the internals of the attitude estimator
 **/
void RobotInterface::updateAttEstPIGains()
{
	// Update the attitude estimator parameters
	m_attitudeEstimator.setPIGains(m_attEstKp(), m_attEstTi(), m_attEstKpQuick(), m_attEstTiQuick());
	m_attEstNoMag.setPIGains(m_attEstKp(), m_attEstTi(), m_attEstKpQuick(), m_attEstTiQuick());
	m_attEstYaw.setPIGains(m_attEstKpYaw(), 1e12, m_attEstKpYawQuick(), 1e12);
}

/**
 * Transcribe the values from the attEstMagCalib config server parameters to the internals of the attitude estimator
 **/
void RobotInterface::updateAttEstMagCalib()
{
	// Update the attitude estimator parameters
	m_attitudeEstimator.setMagCalib(m_attEstMagCalibX(), m_attEstMagCalibY(), m_attEstMagCalibZ());
	m_attEstNoMag.setMagCalib(m_attEstMagCalibX(), m_attEstMagCalibY(), m_attEstMagCalibZ());
	m_attEstYaw.setMagCalib(m_attEstMagCalibX(), m_attEstMagCalibY(), m_attEstMagCalibZ());
}

/**
 * Update the attitude estimator gyro bias based on the config parameters, but only if this will set it
 * to a value that is different to the last time the attitude estimator's gyro bias was set.
 **/
void RobotInterface::updateAttEstGyroCalib()
{
	// Update the attitude estimator parameters only if something has changed
	Eigen::Vector3f configBias(m_attEstGyroBiasX(), m_attEstGyroBiasY(), m_attEstGyroBiasZ());
	if(configBias != m_attEstGyroBiasLast || m_attEstGyroBiasUpdate())
	{
		setAttEstGyroCalib(configBias);
		m_attEstGyroBiasUpdate.set(false);
	}
}

/**
 * Set the gyro biases of the attitude estimators to @p bias.
 **/
void RobotInterface::setAttEstGyroCalib(const Eigen::Vector3f& bias)
{
	// Update the attitude estimator parameters
	m_attitudeEstimator.setGyroBias(bias.x(), bias.y(), bias.z());
	m_attEstNoMag.setGyroBias(bias.x(), bias.y(), bias.z());
	m_attEstYaw.setGyroBias(bias.x(), bias.y(), bias.z());
	m_attEstGyroBiasLast = bias;
}

/**
 * Writes the current gyro mean into the attitude estimator gyro bias config variables.
 **/
void RobotInterface::setAttEstGyroBiasFromMean()
{
	// Update the required config variables
	m_attEstGyroBiasX.set(m_gyroMean.x());
	m_attEstGyroBiasY.set(m_gyroMean.y());
	m_attEstGyroBiasZ.set(m_gyroMean.z());
	m_attEstGyroBiasSetFM.set(false);
}

/**
 * Writes the current gyro smooth mean into the attitude estimator gyro bias config variables.
 **/
void RobotInterface::setAttEstGyroBiasFromSmoothMean()
{
	// Update the required config variables
	m_attEstGyroBiasX.set(m_gyroMeanSmooth.x());
	m_attEstGyroBiasY.set(m_gyroMeanSmooth.y());
	m_attEstGyroBiasZ.set(m_gyroMeanSmooth.z());
	m_attEstGyroBiasSetFSM.set(false);
}

/**
 * Handler function for the attitude estimation magnetometer calibration service.
 * Position the robot so that it is perfectly upright (the robot's local z-axis points in exactly the opposite
 * direction to gravity) and is facing in the direction that is to be declared the zero yaw direction.
 * Nominally this is parallel to the field and facing the positive goal. While the robot is in exactly this
 * position (with as much free air around it as possible too), fire off a call to this service using:
 * @code
 * rosservice call /nimbro_op_interface/attEstCalibrate
 * @endcode
 **/
bool RobotInterface::handleAttEstCalibrate(nimbro_op_interface::AttEstCalibRequest& req, nimbro_op_interface::AttEstCalibResponse& resp)
{
	// Retrieve the current magnetic field vector
	Eigen::Vector3d mag = m_model->magneticFieldVector();

	// Update the config server parameters
	m_attEstMagCalibX.set(mag.x());
	m_attEstMagCalibY.set(mag.y());
	m_attEstMagCalibZ.set(mag.z());
	setAttEstGyroBiasFromMean();

	// Inform the user of the results of the calibration
	ROS_INFO("Attitude estimation calibration: attEstMagCalib vector has been updated to be (%10.7lf, %10.7lf, %10.7lf)", mag.x(), mag.y(), mag.z());
	ROS_INFO("Attitude estimation calibration: attEstGyroBias vector has been updated to be (%10.7lf, %10.7lf, %10.7lf)", m_gyroMean.x(), m_gyroMean.y(), m_gyroMean.z());

	// Return the result of the calibration
	resp.magCalibX = mag.x();
	resp.magCalibY = mag.y();
	resp.magCalibZ = mag.z();
	resp.gyroBiasX = m_gyroMean.x();
	resp.gyroBiasY = m_gyroMean.y();
	resp.gyroBiasZ = m_gyroMean.z();

	// Return that the service was successfully handled
	return true;
}

/**
 * @brief Start a scale factor calibration of the gyroscope.
 **/
bool RobotInterface::handleCalibrateGyroStart(nimbro_op_interface::CalibGyroStartRequest& req, nimbro_op_interface::CalibGyroStartResponse& resp)
{
	// Warn if we just aborted a previous calibration
	if(m_gyroCalibrating != 0)
	{
		ROS_WARN("Aborting the running gyroscope calibration before starting the new one!");
		std_srvs::Empty empty;
		handleCalibrateGyroAbort(empty.request, empty.response);
	}

	// Inform the user that the gyro calibration has started
	ROS_WARN("Starting gyroscope calibration...");
	ROS_INFO("If the gyro bias is not currently accurate, and the yaw-estimating attitude estimator thereby has non-negligible drift, this calibration will not be accurate!");

	// Update the gyro calibration state
	m_gyroCalibrating = 1;

	// Process the request
	m_gyroCalibType = (req.type >= 0 && req.type <= 2 ? req.type : 0);
	m_gyroCalibNumTurns = (req.turns >= 1 ? req.turns : 1);

	// Capture the initial state
	m_gyroCalibInitYaw = m_attEstYaw.fusedYaw();
	m_gyroCalibInitTemp = m_temperature;

	// Initialise the remaining variables
	m_gyroCalibLoopCount = 0;
	m_gyroCalibLoopCountFirst = 0;
	m_gyroCalibUpdateCount = 0;
	m_gyroCalibUpdateCountFirst = 0;
	m_gyroCalibScaleFactor = -1.0;
	m_gyroCalibMiddleYaw = m_gyroCalibInitYaw;
	m_gyroCalibMiddleTemp = m_gyroCalibInitTemp;
	m_gyroCalibCurYaw = m_gyroCalibInitYaw;

	// More information for the user
	std::string updateText;
	if(m_gyroCalibType == 1)
		updateText = "will update the low temperature scale factor";
	else if(m_gyroCalibType == 2)
		updateText = "will update the high temperature scale factor";
	else
		updateText = "will not automatically update any scale factors";
	ROS_INFO("Required actions (%s):\n- Rotate the robot by exactly %d complete revolutions around its yaw axis\n- Call the calibrateGyroReturn service\n- Rotate the robot by exactly %d complete revolutions in the other direction\n- Call the calibrateGyroStop service\n- Call the calibrateGyroAbort service at any time to abort", updateText.c_str(), m_gyroCalibNumTurns, m_gyroCalibNumTurns);
	ROS_INFO("Start state: Yaw %.4f, Temperature %.1f", m_gyroCalibInitYaw, m_gyroCalibInitTemp);

	// Populate the service response
	resp.initialYaw = m_gyroCalibInitYaw;
	resp.initialTemp = m_gyroCalibInitTemp;

	// Return that the service call was successfully handled
	return true;
}

/**
 * @brief Proceed to the return phase of a scale factor calibration of the gyroscope.
 **/
bool RobotInterface::handleCalibrateGyroReturn(nimbro_op_interface::CalibGyroReturnRequest& req, nimbro_op_interface::CalibGyroReturnResponse& resp)
{
	// Make sure this service call is expected
	if(m_gyroCalibrating == 2)
		ROS_WARN("The middle point was already set by calling this service: Updating the existing middle point values...");
	else if(m_gyroCalibrating != 1)
	{
		ROS_WARN("Unexpected service call to calibrateGyroReturn: No calibration is running!");
		return false;
	}

	// Update the gyro calibration state
	m_gyroCalibrating = 2;

	// Capture the middle state
	m_gyroCalibMiddleYaw = m_gyroCalibCurYaw; // We use this variable to retrieve the yaw as it is guaranteed to be consistent with our loop count
	m_gyroCalibMiddleTemp = m_temperature;
	m_gyroCalibLoopCountFirst = m_gyroCalibLoopCount;
	m_gyroCalibLoopCount = 0;
	m_gyroCalibUpdateCountFirst = m_gyroCalibUpdateCount;
	m_gyroCalibUpdateCount = 0;

	// Inform the user as to the middle state
	ROS_INFO("Middle state: Yaw %.4f, Temperature %.1f, %d loops detected, %d estimator updates made", m_gyroCalibMiddleYaw, m_gyroCalibMiddleTemp, m_gyroCalibLoopCountFirst, m_gyroCalibUpdateCountFirst);

	// Populate the service response
	resp.middleYaw = m_gyroCalibMiddleYaw;
	resp.middleTemp = m_gyroCalibMiddleTemp;
	resp.middleLoops = m_gyroCalibLoopCountFirst;

	// Return that the service call was successfully handled
	return true;
}

/**
 * @brief Stop a scale factor calibration of the gyroscope.
 **/
bool RobotInterface::handleCalibrateGyroStop(nimbro_op_interface::CalibGyroStopRequest& req, nimbro_op_interface::CalibGyroStopResponse& resp)
{
	// Make sure this service call is expected
	if(m_gyroCalibrating != 2)
	{
		if(m_gyroCalibrating == 1)
			ROS_WARN("Unexpected service call to calibrateGyroStop: A calibration is running but calibrateGyroReturn hasn't been called yet to set the middle point!");
		else
			ROS_WARN("Unexpected service call to calibrateGyroStop: No calibration is running!");
		return false;
	}

	// Update the gyro calibration state
	m_gyroCalibrating = 0;

	// Capture the final state
	double gyroCalibFinalYaw = m_gyroCalibCurYaw; // We use this variable to retrieve the yaw as it is guaranteed to be consistent with our loop count
	double gyroCalibFinalTemp = m_temperature;

	// Inform the user as to the final state
	ROS_INFO("Final state: Yaw %.4f, Temperature %.1f, %d loops detected, %d estimator updates made", gyroCalibFinalYaw, gyroCalibFinalTemp, m_gyroCalibLoopCount, m_gyroCalibUpdateCount);

	// Compute the average temperature and the linear amount of change in yaw in the two calibration phases
	double temperature = (m_gyroCalibInitTemp + m_gyroCalibMiddleTemp + gyroCalibFinalTemp)/3.0; // Average temperature during the calibration
	double An = M_2PI*m_gyroCalibLoopCountFirst + m_gyroCalibMiddleYaw - m_gyroCalibInitYaw; // Linear change in yaw in the first calibration phase
	double Am = M_2PI*m_gyroCalibLoopCount + gyroCalibFinalYaw - m_gyroCalibMiddleYaw;       // Linear change in yaw in the second calibration phase
	double N = m_gyroCalibUpdateCountFirst; // Number of yaw updates in the first calibration phase
	double M = m_gyroCalibUpdateCount;      // Number of yaw updates in the second calibration phase

	// Error checking
	if(m_gyroCalibNumTurns <= 0)
	{
		ROS_WARN("The required number of turns for the calibration was zero or negative, which should be impossible. How did that happen?");
		return false;
	}
	if(m_gyroCalibScaleFactor <= 0.0)
	{
		ROS_WARN("The gyro scale factor used during the calibration has a mystical value of %.3f. How did that happen?", m_gyroCalibScaleFactor);
		return false;
	}
	if(An*Am >= 0.0)
	{
		ROS_WARN("The robot was rotated in the same direction both times, so the calibration is not valid!");
		return false;
	}
	if(N <= 0 || M <= 0)
	{
		ROS_WARN("One or both of the update counts are zero or negative, so the calibration is not valid!");
		return false;
	}

	// Compute the expected linear amount of change in yaw for the two calibration phases
	double Dn = rc_utils::sign(An) * m_gyroCalibNumTurns * M_2PI; // Desired linear change in yaw in the first calibration phase
	double Dm = rc_utils::sign(Am) * m_gyroCalibNumTurns * M_2PI; // Desired linear change in yaw in the second calibration phase

	// Calculate the scale factor and constant yaw drift rate that best explains the observed measurements
	double yawDriftPerUpdate = (Dn*Am - Dm*An)/(Dn*M - Dm*N);       // Note: The denominator cannot be zero due to the error checking above
	double yawDrift = yawDriftPerUpdate / m_model->timerDuration(); // Note: This is just a very broad estimate, and only meant for getting an idea of the order of magnitude of the assumed yaw drift
	double scaleFactor = (Dn*M - Dm*N)/(An*M - Am*N);               // Note: The denominator cannot be zero due to the error checking above
	scaleFactor *= m_gyroCalibScaleFactor;                          // While the calibration was being done the gyro measurements were already being scaled by a constant scale factor, so we must account for that

	// Present the calibration results to the user
	ROS_INFO("Calibration results: Temperature %.1f requires scale factor %.3f", temperature, scaleFactor);
	ROS_INFO("                     A yaw drift of %.4f rad/s (%.3f deg/s) was approximated", yawDrift, yawDrift*180.0/M_PI);

	// Warn if the scale factor is indicative of some kind of mistake
	if(scaleFactor < 0.5 || scaleFactor > 2.0)
		ROS_WARN("The calculated scale factor is very different to 1.0, so there is probably a problem with your gyro, the units it is expressed in, or your calibration procedure!");

	// Update the config server if the type says so
	if(m_gyroCalibType == 1) // Update low temperature calibration
	{
		m_gyroTemperatureLow.set(temperature);
		m_gyroScaleFactorLT.set(scaleFactor);
		ROS_INFO("Updated the low temperature gyro scale factor on the config server!");
	}
	else if(m_gyroCalibType == 2) // Update high temperature calibration
	{
		m_gyroTemperatureHigh.set(temperature);
		m_gyroScaleFactorHT.set(scaleFactor);
		ROS_INFO("Updated the high temperature gyro scale factor on the config server!");
	}
	else // Don't update any calibration automatically
		ROS_INFO("No gyro scale factors were automatically updated by this calibration!");

	// Indicate that gyroscope calibration is now over
	ROS_WARN("End gyroscope calibration");

	// Populate the service response
	resp.finalYaw = gyroCalibFinalYaw;
	resp.finalTemp = gyroCalibFinalTemp;
	resp.finalLoops = m_gyroCalibLoopCount;
	resp.temperature = temperature;
	resp.type = m_gyroCalibType;
	resp.scaleFactor = scaleFactor;
	resp.yawDrift = yawDrift;

	// Return that the service call was successfully handled
	return true;
}

/**
 * @brief Abort a scale factor calibration of the gyroscope.
 **/
bool RobotInterface::handleCalibrateGyroAbort(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	// Inform the user that the gyro calibration has been aborted
	ROS_INFO("Aborting gyroscope calibration...");

	// Update the gyro calibration state
	m_gyroCalibrating = 0;

	// Return that the service call was successfully handled
	return true;
}

/**
 * Apply the config variable for the temperature low pass settling time.
 **/
void RobotInterface::updateTempLowPassTs()
{
	// Update the low pass filter
	float dT = (m_model ? m_model->timerDuration() : DEFAULT_TIMER_DURATION);
	m_temperatureLowPass.setTs(m_temperatureLowPassTs() / dT);
}

/**
 * Apply the config variable for the voltage low pass settling time.
 **/
void RobotInterface::updateVoltageLowPassTs()
{
	// Update the low pass filter
	float dT = (m_model ? m_model->timerDuration() : DEFAULT_TIMER_DURATION);
	m_voltageLowPass.setTs(m_voltageLowPassTs() / dT);
}

/**
 * Apply the config variable for the acc low pass mean settling time.
 **/
void RobotInterface::updateAccLowPassMeanTs()
{
	// Update the low pass mean filter
	float dT = (m_model ? m_model->timerDuration() : DEFAULT_TIMER_DURATION);
	m_accLowPassMean.setTs(m_accLowPassMeanTs() / dT);
}

/**
 * Apply the config variable for the gyro low pass mean settling time.
 **/
void RobotInterface::updateGyroLowPassMeanTs()
{
	// Update the low pass mean filter
	float dT = (m_model ? m_model->timerDuration() : DEFAULT_TIMER_DURATION);
	m_gyroLowPassMean.setTs(m_gyroLowPassMeanTs() / dT);
}

/**
 * Apply the config variable for the gyro very low pass mean settling time.
 **/
void RobotInterface::updateGyroVeryLowPassMeanTs()
{
	// Update the very low pass mean filter
	float dT = (m_model ? m_model->timerDuration() : DEFAULT_TIMER_DURATION);
	m_gyroVeryLowPassMean.setTs(m_gyroLowPassMeanTsHigh() / dT);
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
void RobotInterface::handleButton(int number, bool longPress) // Note: Calling this means that the number-th bit of the P_BUTTON byte (retrieved from the CM730) was set!
{
	// Indicate that a button was pressed if required
	if(m_showButtonPresses())
		ROS_INFO("Button %d was %s", number, (longPress ? "long-pressed" : "pressed"));

	// If the leftmost button was pressed, fade in/out depending on the current robot state
	if(number == 0 && !longPress)
		sendFadeTorqueGoal(m_model->state() == m_state_relaxed ? 1.0 : 0.0);
}

// Configure the plot manager
void RobotInterface::configurePlotManager()
{
	// Configure the plot manager variable names
	m_PM.setName(PM_ANGEST_PPITCH,       "Estimation/AngleEstimator/ProjPitch");
	m_PM.setName(PM_ANGEST_PROLL,        "Estimation/AngleEstimator/ProjRoll");
	m_PM.setName(PM_ATTEST_FYAW,         "Estimation/AttitudeEstimator/FusedYaw");
	m_PM.setName(PM_ATTEST_FPITCH,       "Estimation/AttitudeEstimator/FusedPitch");
	m_PM.setName(PM_ATTEST_FROLL,        "Estimation/AttitudeEstimator/FusedRoll");
	m_PM.setName(PM_ATTEST_FHEMI,        "Estimation/AttitudeEstimator/FusedHemi");
	m_PM.setName(PM_ATTEST_BIAS_X,       "Estimation/AttitudeEstimator/GyroBiasX");
	m_PM.setName(PM_ATTEST_BIAS_Y,       "Estimation/AttitudeEstimator/GyroBiasY");
	m_PM.setName(PM_ATTEST_BIAS_Z,       "Estimation/AttitudeEstimator/GyroBiasZ");
	m_PM.setName(PM_ATTEST_NOMAG_FYAW,   "Estimation/AttitudeEstimatorNoMag/FusedYaw");
	m_PM.setName(PM_ATTEST_NOMAG_FPITCH, "Estimation/AttitudeEstimatorNoMag/FusedPitch");
	m_PM.setName(PM_ATTEST_NOMAG_FROLL,  "Estimation/AttitudeEstimatorNoMag/FusedRoll");
	m_PM.setName(PM_ATTEST_NOMAG_FHEMI,  "Estimation/AttitudeEstimatorNoMag/FusedHemi");
	m_PM.setName(PM_ATTEST_NOMAG_BIAS_X, "Estimation/AttitudeEstimatorNoMag/GyroBiasX");
	m_PM.setName(PM_ATTEST_NOMAG_BIAS_Y, "Estimation/AttitudeEstimatorNoMag/GyroBiasY");
	m_PM.setName(PM_ATTEST_NOMAG_BIAS_Z, "Estimation/AttitudeEstimatorNoMag/GyroBiasZ");
	m_PM.setName(PM_ATTEST_YAW_FYAW,     "Estimation/AttitudeEstimatorYaw/FusedYaw");
	m_PM.setName(PM_ATTEST_YAW_FPITCH,   "Estimation/AttitudeEstimatorYaw/FusedPitch");
	m_PM.setName(PM_ATTEST_YAW_FROLL,    "Estimation/AttitudeEstimatorYaw/FusedRoll");
	m_PM.setName(PM_ATTEST_YAW_FHEMI,    "Estimation/AttitudeEstimatorYaw/FusedHemi");
	m_PM.setName(PM_ATTEST_YAW_BIAS_X,   "Estimation/AttitudeEstimatorYaw/GyroBiasX");
	m_PM.setName(PM_ATTEST_YAW_BIAS_Y,   "Estimation/AttitudeEstimatorYaw/GyroBiasY");
	m_PM.setName(PM_ATTEST_YAW_BIAS_Z,   "Estimation/AttitudeEstimatorYaw/GyroBiasZ");
	m_PM.setName(PM_GYRO_X,              "Estimation/Gyro/x");
	m_PM.setName(PM_GYRO_Y,              "Estimation/Gyro/y");
	m_PM.setName(PM_GYRO_Z,              "Estimation/Gyro/z");
	m_PM.setName(PM_GYRO_N,              "Estimation/Gyro/norm");
	m_PM.setName(PM_GYROMEAN_X,          "Estimation/GyroMean/x");
	m_PM.setName(PM_GYROMEAN_Y,          "Estimation/GyroMean/y");
	m_PM.setName(PM_GYROMEAN_Z,          "Estimation/GyroMean/z");
	m_PM.setName(PM_GYROMEAN_N,          "Estimation/GyroMean/norm");
	m_PM.setName(PM_GYROMEAN_SMOOTH_X,   "Estimation/GyroMeanSmooth/x");
	m_PM.setName(PM_GYROMEAN_SMOOTH_Y,   "Estimation/GyroMeanSmooth/y");
	m_PM.setName(PM_GYROMEAN_SMOOTH_Z,   "Estimation/GyroMeanSmooth/z");
	m_PM.setName(PM_GYROMEAN_SMOOTH_N,   "Estimation/GyroMeanSmooth/norm");
	m_PM.setName(PM_GYRO_MEAN_OFFSET,    "Estimation/GyroAutoCalib/GyroToMeanOffset");
	m_PM.setName(PM_GYRO_STABLE_TIME,    "Estimation/GyroAutoCalib/GyroStableTime");
	m_PM.setName(PM_GYRO_BIAS_TS,        "Estimation/GyroAutoCalib/GyroBiasTs");
	m_PM.setName(PM_GYRO_BIAS_ALPHA,     "Estimation/GyroAutoCalib/GyroBiasAlpha");
	m_PM.setName(PM_GYRO_SCALE_FACTOR,   "Estimation/GyroScaleFactor");
	m_PM.setName(PM_ACC_XRAW,            "Estimation/Acc/Raw x");
	m_PM.setName(PM_ACC_YRAW,            "Estimation/Acc/Raw y");
	m_PM.setName(PM_ACC_ZRAW,            "Estimation/Acc/Raw z");
	m_PM.setName(PM_ACC_NRAW,            "Estimation/Acc/Raw norm");
	m_PM.setName(PM_ACC_X,               "Estimation/Acc/x");
	m_PM.setName(PM_ACC_Y,               "Estimation/Acc/y");
	m_PM.setName(PM_ACC_Z,               "Estimation/Acc/z");
	m_PM.setName(PM_ACC_N,               "Estimation/Acc/norm");
	m_PM.setName(PM_ACCMEAN_X,           "Estimation/AccMean/x");
	m_PM.setName(PM_ACCMEAN_Y,           "Estimation/AccMean/y");
	m_PM.setName(PM_ACCMEAN_Z,           "Estimation/AccMean/z");
	m_PM.setName(PM_ACCMEAN_N,           "Estimation/AccMean/norm");
	m_PM.setName(PM_MAG_XRAW,            "Estimation/Mag/Raw x");
	m_PM.setName(PM_MAG_YRAW,            "Estimation/Mag/Raw y");
	m_PM.setName(PM_MAG_ZRAW,            "Estimation/Mag/Raw z");
	m_PM.setName(PM_MAG_XSPIKE,          "Estimation/Mag/No spike x");
	m_PM.setName(PM_MAG_YSPIKE,          "Estimation/Mag/No spike y");
	m_PM.setName(PM_MAG_ZSPIKE,          "Estimation/Mag/No spike z");
	m_PM.setName(PM_MAG_XIRON,           "Estimation/Mag/Biased x");
	m_PM.setName(PM_MAG_YIRON,           "Estimation/Mag/Biased y");
	m_PM.setName(PM_MAG_ZIRON,           "Estimation/Mag/Biased z");
	m_PM.setName(PM_MAG_X,               "Estimation/Mag/Filt x");
	m_PM.setName(PM_MAG_Y,               "Estimation/Mag/Filt y");
	m_PM.setName(PM_MAG_Z,               "Estimation/Mag/Filt z");
	m_PM.setName(PM_MAG_N,               "Estimation/Mag/Filt norm");
	m_PM.setName(PM_TEMPERATURE,         "Estimation/Temperature");
	m_PM.setName(PM_VOLTAGE,             "Estimation/Voltage");
	m_PM.setName(PM_SENSOR_DT,           "Estimation/Sensor dT");

	// Check that we have been thorough
	if(!m_PM.checkNames())
		ROS_ERROR("Please review any warnings above that are related to the naming of plotter variables!");
}

// Callback for receiving buzzer commands
void RobotInterface::handleBuzzerCommand(const BuzzerConstPtr& cmd)
{
	// Process the commanded buzzer tone
	if(cmd->soundType == Buzzer::TONE)
	{
		if(cmd->toneFreq < 52 && cmd->toneDuration >= 0.0 && cmd->toneDuration <= 5.0)
		{
			uint8_t durTick = (uint8_t)(10.0*cmd->toneDuration + 0.5);
			if(durTick > 0 && durTick <= 50)
			{
				m_buzzerData.playLength = durTick;
				m_buzzerData.data = cmd->toneFreq;
				m_haveBuzzerData = true;
			}
		}
	}
	else if(cmd->soundType == Buzzer::MUSIC)
	{
		if(cmd->musicIndex < 26)
		{
			m_buzzerData.playLength = 0xFF;
			m_buzzerData.data = cmd->musicIndex;
			m_haveBuzzerData = true;
		}
	}
}

// Send a fade torque goal
void RobotInterface::sendFadeTorqueGoal(float torque)
{
	// Send the fade torque goal if we haven't just already sent one
	if(m_fadingIsTriggered)
		ROS_INFO("Dismissing extra fade request (torque fading is already active)!");
	else
	{
		FadeTorqueGoal goal;
		goal.torque = rc_utils::coerce(torque, 0.0f, 1.0f);
		m_fadingIsTriggered = true;
		m_fadeTorqueClient.sendGoal(goal, boost::bind(&RobotInterface::resetFadingTriggered, this));
	}
}

// Callback for monitoring when a sent fade goal has finished
void RobotInterface::resetFadingTriggered()
{
	// Reset the fading triggered flag
	m_fadingIsTriggered = false;
}

// Publish the default calculated logger state
void RobotInterface::sendLoggerHeartbeat()
{
	// We should log if the robot is not in the relaxed state
	bool log = (m_model->state() != m_state_relaxed && m_model->state() != m_state_setting_pose);
	sendLoggerHeartbeat(log);
}

// Send a heartbeat to the logger
void RobotInterface::sendLoggerHeartbeat(bool log)
{
	// Get the current ROS time
	ros::Time now = ros::Time::now();

	// See whether we have new information to publish
	bool newState = (log != m_loggerMsg.enableLogging);
	bool newTime = ((now - m_loggerStamp).toSec() >= 0.4);

	// Publish the required logger state if we have reason to
	if(newState || newTime)
	{
		m_loggerStamp = now;
		m_loggerMsg.enableLogging = log;
		m_pub_logger.publish(m_loggerMsg);
	}
}

}

PLUGINLIB_EXPORT_CLASS(nimbro_op_interface::RobotInterface, robotcontrol::HardwareInterface);
// EOF
