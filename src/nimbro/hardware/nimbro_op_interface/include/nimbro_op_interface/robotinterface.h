// NimbRo-OP robot hardware interface
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//         Sebastian Schueller <schuell1@cs.uni-bonn.de>
//         Max Schwarz <max.schwarz@uni-bonn.de>

// Ensure header is only included once
#ifndef RC_ROBOTINTERFACE_H
#define RC_ROBOTINTERFACE_H

// Includes
#include <robotcontrol/model/robotmodel.h>
#include <robotcontrol/hw/hardwareinterface.h>
#include <robotcontrol/hw/magfilter.h>
#include <robotcontrol/FadeTorqueAction.h>
#include <rc_utils/attitude_estimator.h>
#include <rc_utils/angle_estimator.h>
#include <rc_utils/firfilter.h>
#include <rc_utils/golay.h>
#include <robotcontrol/ServoDiag.h>
#include <rc_utils/low_pass_filter.h>
#include <rc_utils/spike_filter.h>
#include <nimbro_op_interface/Buzzer.h>
#include <nimbro_op_interface/LEDCommand.h>
#include <nimbro_op_interface/ReadOffset.h>
#include <nimbro_op_interface/AttEstCalib.h>
#include <nimbro_op_interface/CalibGyroStop.h>
#include <nimbro_op_interface/CalibGyroStart.h>
#include <nimbro_op_interface/CalibGyroReturn.h>
#include <nimbro_op_interface/CalibGyroAccNow.h>
#include <nimbro_op_interface/CalibGyroAccStop.h>
#include <actionlib/client/simple_action_client.h>
#include <rrlogger/LoggerHeartbeat.h>
#include <config_server/parameter.h>
#include <plot_msgs/plot_manager.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>
#include <cm730/CM730.h>
#include <ros/timer.h>

// Class forward declarations
namespace urdf { class Joint; }
namespace robotcontrol
{
	class RobotModel;
	class DynamicCommandGenerator;
	typedef boost::shared_ptr<DynamicCommandGenerator> CommandGeneratorPtr;
}

// NimbRo-OP interface namespace
namespace nimbro_op_interface
{

/**
 * @brief Real robot interface
 *
 * Connects to a NimbRo-OP robot directly connected to the machine.
 *
 * Parameters on the config_server (under the robotcontrol group):
 *
 * Name                | Meaning
 * ------------------- | ---------------------------------
 * useModel            | Enable the servo model for command generation
 * effortSlopeLimit    | Maximum delta of the joint effort in a cycle
 * joints/X/type       | Actuator type of joint X
 * joints/X/id         | Address of joint X on the Dynamixel bus
 * joints/X/invert     | Invert the joint direction
 * offsets/X           | Offset (in servo ticks) of joint X
 *
 * The RobotInterface also advertises a single service called *read_offsets*
 * which resets the joint offsets so that the current measured robot pose
 * becomes the zero pose. This is useful for initial calibration (with a
 * relaxed robot) before fine-tuning the offsets.
 **/
class RobotInterface : public robotcontrol::HardwareInterface
{
public:
	// Constructor/destructor
	RobotInterface();
	virtual ~RobotInterface();

	// Virtual function overrides
	virtual bool init(robotcontrol::RobotModel* model);
	virtual void deinit();
	virtual boost::shared_ptr<robotcontrol::Joint> createJoint(const std::string& name);
	virtual void processJointCommands();
	virtual void processJointTorques();
	virtual void processJointFeedback();
	virtual bool sendJointTargets();
	virtual bool readJointStates();
	virtual bool setStiffness(float torque);
	virtual void getDiagnostics(robotcontrol::DiagnosticsPtr ptr);

	// Constants
	static const std::string RESOURCE_PATH;
	static const std::string CONFIG_PARAM_PATH;

protected:
	// Joint command data (not for direct use with sync writes)
	struct JointCmdData
	{
		JointCmdData() = default;
		JointCmdData(int id, int p_gain, int goal_position) : id(id), p_gain(p_gain), goal_position(goal_position) {}
		int id;
		int p_gain;
		int goal_position;
	};

	// Virtual functions for hardware abstraction
	virtual bool initCM730();
	virtual int  readFeedbackData(bool onlyTryCM730);
	virtual bool syncWriteJointTargets(const std::vector<JointCmdData>& jointCmdData);
	virtual bool syncWriteReturnLevel(size_t numDevices, const uint8_t* data);
	virtual bool syncWriteTorqueEnable(size_t numDevices, const uint8_t* data);
	virtual bool syncWriteTorqueLimit(size_t numDevices, const uint8_t* data);
	virtual bool useModel() const { return m_useModel(); }

	// Scaling constants
	static const double INT_TO_VOLTS;
	static const double GYRO_SCALE;
	static const double ACC_SCALE;
	static const double MAG_SCALE;

	//! Node handle
	ros::NodeHandle m_nh;

	//! The robot model
	robotcontrol::RobotModel* m_model;

	//! Our servo hardware type
	boost::shared_ptr<cm730::DynamixelBase> m_servos;

	//! Bulk read buffer for servo feedback
	std::vector<cm730::BRData> m_servoData;

	//! Bulk read buffer for CM730 feedback (e.g. IMU)
	cm730::BRBoard m_boardData;

	//! Robot name
	std::string m_robotNameStr;

	//! Robot type
	std::string m_robotTypeStr;

	/**
	* @internal Helper struct that reflects the layout of an individual LED command write.
	* This struct is for use with the sendCM730LedData() function. RGBLED6 is not written
	* to because it is used by the CM730 to display the USB connection status, and is as
	* such write protected (attempting to write to the RGBLED6 register results in the
	* entire write packet being discarded).
	**/
	struct CM730LedWriteData
	{
		uint8_t id;        //!< CM730 ID
		uint8_t led_panel; //!< P_LED_PANEL
		uint16_t rgbled5;  //!< P_RGBLED5_L / P_RGBLED5_H
	} __attribute__((packed));

	/**
	* @internal Layout of the buzzer data command to the CM730.
	**/
	struct CM730BuzzerWriteData
	{
		uint8_t playLength; //!< P_BUZZER_PLAY_LENGTH
		uint8_t data;       //!< P_BUZZER_DATA
	} __attribute__((packed));

	/**
	* @internal Helper struct that reflects the layout of an individual status return level sync write command.
	**/
	struct ReturnLevelSyncWriteData
	{
		uint8_t id;           //!< Servo ID
		uint8_t return_level; //!< Status return level
	} __attribute__((packed));

	/**
	* @internal Helper struct that reflects the layout of an individual torque enable sync write command.
	* This struct is for use with the setStiffness() function, and allows for the enabling and disabling
	* of individual servo torques.
	**/
	struct TorqueEnableSyncWriteData
	{
		uint8_t id;            //!< Servo ID
		uint8_t torque_enable; //!< Torque enable
	} __attribute__((packed));

	/**
	* @internal Helper struct that reflects the layout of an individual torque limit sync write command.
	* This struct is for use with the setStiffness() function, and allows for torque fading.
	**/
	struct TorqueLimitSyncWriteData
	{
		uint8_t id;            //!< Servo ID
		uint16_t torque_limit; //!< Torque limit (MX) OR Goal PWM (X)
	} __attribute__((packed));

// 	/**
// 	* @internal Layout of the statistics data requested from the servo.
// 	**/
// 	struct StatisticsReadData
// 	{
// 		uint8_t voltage;     //!< P_PRESENT_VOLTAGE (Note: Current incompatibility => X servos voltage is 2 bytes!)
// 		uint8_t temperature; //!< P_PRESENT_TEMPERATURE
// 	} __attribute__((packed));

	/**
	 * @brief Joint class with hardware information
	 **/
	struct DXLJoint : public robotcontrol::Joint
	{
		explicit DXLJoint(const std::string& name);

		//! Servo command generator appropiate for the servo type
		robotcontrol::CommandGeneratorPtr commandGenerator;

		// Constants
		static const std::string CONFIG_PARAM_PATH;

		//! @name config_server joint parameters
		//@{
		config_server::Parameter<std::string> type;  //!< Actuator type
		config_server::Parameter<int> id;            //!< Actuator address
		config_server::Parameter<int> tickOffset;    //!< Joint offset in ticks
		config_server::Parameter<bool> invert;       //!< Invert the joint direction
		config_server::Parameter<bool> readFeedback; //!< Flag whether reading of feedback is enabled
		config_server::Parameter<bool> enabled;      //!< Flag whether joint is globally enabled
		//@}

		//! @name Statistics
		//@{
		int voltage;
		int temperature;
		//@}

		//! @name Control values smoothing
		//@{
		double realEffort;
		double rawState;
		//@}

		//! Statistics message
		robotcontrol::ServoDiag diag;
	};

	/**
	 * @brief Get the DXLJoint for an index
	 *
	 * @param idx Index in the RobotModel joint struct array (not servo ID)
	 */
	inline DXLJoint* dxlJoint(size_t idx) const { return (DXLJoint*) m_model->joint(idx).get(); }
	DXLJoint* dxlJointForID(int id) const;
	DXLJoint* dxlJointForName(const std::string& name) const;

	// Joint alias type struct
	struct JointAliasType
	{
		JointAliasType(DXLJoint* alias, const DXLJoint* source, double multiplier = 1.0, double offset = 0.0) : alias(alias), source(source), multiplier(multiplier), offset(offset) {}
		DXLJoint* alias;
		const DXLJoint* source;
		double multiplier;
		double offset;
	};

	// Joint aliases
	void clearJointAliases() { m_jointAliases.clear(); }
	void addJointAlias(const JointAliasType& JA) { m_jointAliases.push_back(JA); }
	void addJointAlias(DXLJoint* alias, const DXLJoint* source, double multiplier = 1.0, double offset = 0.0) { m_jointAliases.emplace_back(alias, source, multiplier, offset); }

	// Joint command dependencies
	void clearJointCommandDependencies() { m_jointCommandDependencies.clear(); }
	void addJointCommandDependency(const boost::function<void (DXLJoint::Command*, const DXLJoint::Command*)>& func, DXLJoint::Command* cmd, const DXLJoint::Command* cmdA) { m_jointCommandDependencies.push_back(boost::bind(func, cmd, cmdA)); }
	void addJointCommandDependency(const boost::function<void (DXLJoint::Command*, const DXLJoint::Command*, const DXLJoint::Command*)>& func, DXLJoint::Command* cmd, const DXLJoint::Command* cmdA, const DXLJoint::Command* cmdB) { m_jointCommandDependencies.push_back(boost::bind(func, cmd, cmdA, cmdB)); }
	void addJointCommandDependency(const boost::function<void (DXLJoint::Command*, const DXLJoint::Command*, const DXLJoint::Command*, const DXLJoint::Command*)>& func, DXLJoint::Command* cmd, const DXLJoint::Command* cmdA, const DXLJoint::Command* cmdB, const DXLJoint::Command* cmdC) { m_jointCommandDependencies.push_back(boost::bind(func, cmd, cmdA, cmdB, cmdC)); }

	// Joint torque dependencies
	void clearJointTorqueDependencies() { m_jointTorqueDependencies.clear(); }
	void addJointTorqueDependency(const boost::function<void (double*, const double*)>& func, double* torque, const double* torqueA) { m_jointTorqueDependencies.push_back(boost::bind(func, torque, torqueA)); }
	void addJointTorqueDependency(const boost::function<void (double*, const double*, const double*)>& func, double* torque, const double* torqueA, const double* torqueB) { m_jointTorqueDependencies.push_back(boost::bind(func, torque, torqueA, torqueB)); }
	void addJointTorqueDependency(const boost::function<void (double*, const double*, const double*, const double*)>& func, double* torque, const double* torqueA, const double* torqueB, const double* torqueC) { m_jointTorqueDependencies.push_back(boost::bind(func, torque, torqueA, torqueB, torqueC)); }

	// Joint feedback dependencies
	void clearJointFeedbackDependencies() { m_jointFeedbackDependencies.clear(); }
	void addJointFeedbackDependency(const boost::function<void (DXLJoint::Feedback*, const DXLJoint::Feedback*)>& func, DXLJoint::Feedback* feed, const DXLJoint::Feedback* feedA) { m_jointFeedbackDependencies.push_back(boost::bind(func, feed, feedA)); }
	void addJointFeedbackDependency(const boost::function<void (DXLJoint::Feedback*, const DXLJoint::Feedback*, const DXLJoint::Feedback*)>& func, DXLJoint::Feedback* feed, const DXLJoint::Feedback* feedA, const DXLJoint::Feedback* feedB) { m_jointFeedbackDependencies.push_back(boost::bind(func, feed, feedA, feedB)); }
	void addJointFeedbackDependency(const boost::function<void (DXLJoint::Feedback*, const DXLJoint::Feedback*, const DXLJoint::Feedback*, const DXLJoint::Feedback*)>& func, DXLJoint::Feedback* feed, const DXLJoint::Feedback* feedA, const DXLJoint::Feedback* feedB, const DXLJoint::Feedback* feedC) { m_jointFeedbackDependencies.push_back(boost::bind(func, feed, feedA, feedB, feedC)); }

	// Joint command dependency functions
	static void jointCmdEqual(DXLJoint::Command* cmd, const DXLJoint::Command* cmdA);
	static void jointCmdNegate(DXLJoint::Command* cmd, const DXLJoint::Command* cmdA);
	static void jointCmdSum(DXLJoint::Command* cmd, const DXLJoint::Command* cmdA, const DXLJoint::Command* cmdB);
	static void jointCmdDiff(DXLJoint::Command* cmd, const DXLJoint::Command* cmdA, const DXLJoint::Command* cmdB);

	// Joint torque dependency functions
	static void jointTorqueEqual(double* torque, const double* torqueA);
	static void jointTorqueNegate(double* torque, const double* torqueA);
	static void jointTorqueSum(double* torque, const double* torqueA, const double* torqueB);
	static void jointTorqueDiff(double* torque, const double* torqueA, const double* torqueB);

	// Joint feedback dependency functions
	static void jointFeedEqual(DXLJoint::Feedback* feed, const DXLJoint::Feedback* feedA);
	static void jointFeedNegate(DXLJoint::Feedback* feed, const DXLJoint::Feedback* feedA);
	static void jointFeedSum(DXLJoint::Feedback* feed, const DXLJoint::Feedback* feedA, const DXLJoint::Feedback* feedB);
	static void jointFeedDiff(DXLJoint::Feedback* feed, const DXLJoint::Feedback* feedA, const DXLJoint::Feedback* feedB);

	//! Add mimic joint dependencies
	bool addMimicJointDependencies();

	//! Configure joint dependencies
	void clearJointDependencies() { clearJointAliases(); clearJointCommandDependencies(); clearJointTorqueDependencies(); clearJointFeedbackDependencies(); }
	virtual bool initJointDependencies() { return addMimicJointDependencies(); } // Overriders should use addJointAlias(), addJointCommandDependency(), addJointTorqueDependency(), addJointFeedbackDependency() and addMimicJointDependencies() as required

private:
	// Flag whether the real hardware is present (assumes no unless otherwise told)
	bool m_haveHardware;

	//! Joint alias and dependency variables
	std::vector<JointAliasType> m_jointAliases;
	std::vector<boost::function<void ()>> m_jointCommandDependencies;
	std::vector<boost::function<void ()>> m_jointTorqueDependencies;
	std::vector<boost::function<void ()>> m_jointFeedbackDependencies;

	//! Process changed joint settings
	void updateJointSettings(DXLJoint* joint);
	void setJointFeedbackTime(const ros::Time stamp);

	//! Get an appropiate command generator for a servo type
	robotcontrol::CommandGeneratorPtr commandGenerator(const std::string& type);

	//! Service to list all joints and their configurations
	ros::ServiceServer m_srv_listJoints;
	bool handleListJoints(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

	// Topic handlers
	void handleStatistics(const ros::TimerEvent&);
	void handleLEDCommand(const LEDCommandConstPtr& cmd);

	//! Handle button press
	void handleButton(int number, bool longPress);
	config_server::Parameter<bool> m_buttonPress0;
	config_server::Parameter<bool> m_buttonPress1;
	config_server::Parameter<bool> m_buttonPress2;

	void sendCM730LedCommand();

	void waitForRelaxedServos();

	//! Our hardware driver
	boost::shared_ptr<cm730::CM730> m_board;

	//! All servo types and generators we know about
	std::map<std::string, robotcontrol::CommandGeneratorPtr> m_generators;

	//! All joints relaxed?
	bool m_relaxed;
	bool m_enableTorque;

	//! Timer used for statistics callback
	ros::Timer m_statisticsTimer;

	//! Servo to ask for statistics next
	size_t m_statIndex;

	//! Mean servo voltage
	double m_statVoltage;

	//! Service to read offsets from hardware
	ros::ServiceServer m_srv_readOffsets;
	ros::ServiceServer m_srv_readOffset;
	bool handleReadOffsets(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
	bool handleReadOffset(ReadOffsetRequest& req, ReadOffsetResponse& resp);

	// IMU orientation offsets
	enum GyroAccOffsetMeasType
	{
		GAOMT_UPRIGHT = 0,
		GAOMT_FRONT,
		GAOMT_RIGHT,
		GAOMT_BACK,
		GAOMT_LEFT,
		GAOMT_COUNT
	};
	struct GyroAccOffsetMeas
	{
		GyroAccOffsetMeasType type;
		Eigen::Vector3d acc;
	};
	config_server::Parameter<bool>  m_resetGyroAccOffset;     // Trigger to reset the gyro/acc orientation offset to the identity rotation
	config_server::Parameter<bool>  m_resetMagOffset;         // Trigger to reset the magnetometer orientation offset to the identity rotation
	config_server::Parameter<float> m_gyroAccFusedYaw;        // The fused yaw of the mounting orientation of the gyro/acc sensors
	config_server::Parameter<float> m_gyroAccTiltAngle;       // The tilt angle of the mounting orientation of the gyro/acc sensors
	config_server::Parameter<float> m_gyroAccTiltAxisAngle;   // The tilt axis angle of the mounting orientation of the gyro/acc sensors
	config_server::Parameter<bool>  m_magFlip;                // A boolean flag whether the configured mounting orientation of the magnetometer sensor should be flipped by 180 degrees to ensure real CW rotations produce measured CW rotations, and the same for CCW rotations
	config_server::Parameter<float> m_magFusedYaw;            // The fused yaw of the mounting orientation of the magnetometer sensor
	config_server::Parameter<float> m_magTiltAngle;           // The tilt angle of the mounting orientation of the magnetometer sensor
	config_server::Parameter<float> m_magTiltAxisAngle;       // The tilt axis angle of the mounting orientation of the magnetometer sensor
	config_server::Parameter<float> m_gyroAccMaxRelAccScale;  // The maximum relative error in the norm of the gyro/acc calibration acc values relative to the norm of the upright measured value (0 => Norms from 100% to 100% are allowed, 1 => Norms from 0% to 200% are allowed) 
	config_server::Parameter<float> m_gyroAccMinYawAgreement; // The minimum level of required agreement in the fused yaw for the gyro/acc calibration (0 => No agreement required, 1 => Every data type average must point in the exact same direction)
	ros::ServiceServer m_srv_imuOffsetsCalibGyroAccStart;
	ros::ServiceServer m_srv_imuOffsetsCalibGyroAccNow;
	ros::ServiceServer m_srv_imuOffsetsCalibGyroAccStop;
	void handleResetGyroAccOffset();
	void handleResetMagOffset();
	bool handleCalibGyroAccStart(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
	bool handleCalibGyroAccNow(CalibGyroAccNowRequest& req, CalibGyroAccNowResponse& resp);
	bool handleCalibGyroAccStop(CalibGyroAccStopRequest& req, CalibGyroAccStopResponse& resp);
	bool m_imuOffsetsGACalib;
	std::vector<GyroAccOffsetMeas> m_imuOffsetsGAData;

	//! Attitude estimator
	stateestimation::AttitudeEstimator m_attitudeEstimator; // Attitude estimator instance
	stateestimation::AttitudeEstimator m_attEstNoMag;       // Attitude estimator instance (no magnetometer)
	stateestimation::AttitudeEstimator m_attEstYaw;         // Attitude estimator instance (yaw without magnetometer)
	config_server::Parameter<bool> m_attEstUseFusedMethod;  // Boolean flag specifying which acc-only resolution method to use
	void updateAttEstMethod();                              // Transcribes the value from the config server parameters to the internals of the attitude estimator
	config_server::Parameter<float> m_attEstKp;             // Kp parameter (P gain) to use for the attitude estimator
	config_server::Parameter<float> m_attEstTi;             // Ti parameter (integral time constant) to use for the attitude estimator
	config_server::Parameter<float> m_attEstKpQuick;        // Maximum Kp parameter (P gain) to use for the attitude estimator during quick learning
	config_server::Parameter<float> m_attEstTiQuick;        // Maximum Ti parameter (integral time constant) to use for the attitude estimator during quick learning
	config_server::Parameter<float> m_attEstKpYaw;          // Kp parameter (P gain) to use for the yaw-integrating attitude estimator (Ti is essentially infinite)
	config_server::Parameter<float> m_attEstKpYawQuick;     // Maximum Kp parameter (P gain) to use for the yaw-integrating attitude estimator during quick learning (Ti is essentially infinite)
	void updateAttEstPIGains();                             // Transcribes the values from the config server parameters to the internals of the attitude estimator
	config_server::Parameter<float> m_attEstMagCalibX;      // m_attEstMagCalib_xyz should be equal to the vector value of m_magFilter.value() [i.e. RobotModel::magneticFieldVector()]
	config_server::Parameter<float> m_attEstMagCalibY;      // when the robot is completely upright and with its coordinate axes perfectly aligned with the global coordinate frame.
	config_server::Parameter<float> m_attEstMagCalibZ;      // Nominally this is when the robot has zero pitch/roll and is facing in the direction of the positive goal.
	void updateAttEstMagCalib();                            // Transcribes the values from the config server parameters to the internals of the attitude estimator
	config_server::Parameter<float> m_attEstGyroBiasX;      // An estimate of the gyro bias along the x axis (only modifies the attitude estimator whenever the config server parameter is updated in value)
	config_server::Parameter<float> m_attEstGyroBiasY;      // An estimate of the gyro bias along the y axis (only modifies the attitude estimator whenever the config server parameter is updated in value)
	config_server::Parameter<float> m_attEstGyroBiasZ;      // An estimate of the gyro bias along the z axis (only modifies the attitude estimator whenever the config server parameter is updated in value)
	config_server::Parameter<bool>  m_attEstGyroBiasUpdate; // Boolean config parameter to force a write of the config parameter gyro biases to the attitude estimators
	config_server::Parameter<bool>  m_attEstGyroBiasSetFM;  // Boolean config parameter to trigger setting of the gyro bias estimate to the current gyro mean
	config_server::Parameter<bool>  m_attEstGyroBiasSetFSM; // Boolean config parameter to trigger setting of the gyro bias estimate to the current gyro smooth mean
	Eigen::Vector3f m_attEstGyroBiasLast;                   // Variable used to check whether the gyro bias config server parameters have changed
	void updateAttEstGyroCalib();                           // Checks whether the value of the gyro bias on the config server has changed, and if so transcribes the gyro bias to the internals of the attitude estimator
	void setAttEstGyroCalib(const Eigen::Vector3f& bias);   // Writes the given gyro bias to the internals of the attitude estimator
	void setAttEstGyroBiasFromMean();                       // Writes the current gyro mean into the attitude estimator gyro bias config variables
	void setAttEstGyroBiasFromSmoothMean();                 // Writes the current gyro smooth mean into the attitude estimator gyro bias config variables

	//! Service to set the attitude estimation gyro and magnetometer calibration values
	ros::ServiceServer m_srv_attEstCalibrate;
	bool handleAttEstCalibrate(nimbro_op_interface::AttEstCalibRequest& req, nimbro_op_interface::AttEstCalibResponse& resp);

	//! Simple 2D fused angle estimator
	stateestimation::AngleEstimator m_angleEstimator;

	// Temperature processing
	config_server::Parameter<float> m_temperatureLowPassTs;
	rc_utils::LowPassFilter m_temperatureLowPass;
	double m_temperature;
	void updateTempLowPassTs();

	// Voltage processing
	config_server::Parameter<float> m_voltageLowPassTs;
	rc_utils::LowPassFilter m_voltageLowPass;
	bool m_initedVoltage;
	double m_voltage;
	void updateVoltageLowPassTs();

	// Acc data processing
	config_server::Parameter<float> m_accLowPassMeanTs;
	rc_utils::LowPassFilterT<Eigen::Vector3d> m_accLowPassMean;
	Eigen::Vector3d m_accMean;
	void updateAccLowPassMeanTs();

	// FIR filters for accelerometer data smoothing
	rc_utils::FIRFilter<9> m_fir_accX;
	rc_utils::FIRFilter<9> m_fir_accY;
	rc_utils::FIRFilter<11> m_fir_accZ;

	// Gyro data processing
	config_server::Parameter<bool>  m_gyroEnableAutoCalib;
	config_server::Parameter<float> m_gyroScaleFactorHT;
	config_server::Parameter<float> m_gyroScaleFactorLT;
	config_server::Parameter<float> m_gyroTemperatureHigh;
	config_server::Parameter<float> m_gyroTemperatureLow;
	config_server::Parameter<float> m_gyroLowPassMeanTs;
	config_server::Parameter<float> m_gyroLowPassMeanTsHigh;
	config_server::Parameter<float> m_gyroStabilityBound;
	config_server::Parameter<float> m_gyroCalibFadeTimeStart;
	config_server::Parameter<float> m_gyroCalibFadeTimeDur;
	config_server::Parameter<float> m_gyroCalibTsSlow;
	config_server::Parameter<float> m_gyroCalibTsFast;
	rc_utils::LowPassFilterT<Eigen::Vector3d> m_gyroLowPassMean;
	rc_utils::LowPassFilterT<Eigen::Vector3d> m_gyroVeryLowPassMean;
	Eigen::Vector3d m_gyroMean;
	Eigen::Vector3d m_gyroMeanSmooth;
	unsigned int m_gyroStableCount;
	bool m_gyroBiasAdjusting;
	void updateGyroLowPassMeanTs();
	void updateGyroVeryLowPassMeanTs();

	// Gyro calibration
	bool handleCalibrateGyroStart(nimbro_op_interface::CalibGyroStartRequest& req, nimbro_op_interface::CalibGyroStartResponse& resp);
	bool handleCalibrateGyroReturn(nimbro_op_interface::CalibGyroReturnRequest& req, nimbro_op_interface::CalibGyroReturnResponse& resp);
	bool handleCalibrateGyroStop(nimbro_op_interface::CalibGyroStopRequest& req, nimbro_op_interface::CalibGyroStopResponse& resp);
	bool handleCalibrateGyroAbort(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
	ros::ServiceServer m_srv_calibrateGyroStart;
	ros::ServiceServer m_srv_calibrateGyroReturn;
	ros::ServiceServer m_srv_calibrateGyroStop;
	ros::ServiceServer m_srv_calibrateGyroAbort;
	int m_gyroCalibrating;
	int m_gyroCalibType;
	int m_gyroCalibNumTurns;
	int m_gyroCalibLoopCount;
	int m_gyroCalibLoopCountFirst;
	int m_gyroCalibUpdateCount;
	int m_gyroCalibUpdateCountFirst;
	double m_gyroCalibScaleFactor;
	double m_gyroCalibInitYaw;
	double m_gyroCalibInitTemp;
	double m_gyroCalibMiddleYaw;
	double m_gyroCalibMiddleTemp;
	double m_gyroCalibCurYaw;

	// Magnetometer data processing
	config_server::Parameter<bool> m_useMagnetometer;
	config_server::Parameter<float> m_magSpikeMaxDelta;
	rc_utils::SpikeFilter m_magSpikeFilterX;
	rc_utils::SpikeFilter m_magSpikeFilterY;
	rc_utils::SpikeFilter m_magSpikeFilterZ;
	rc_utils::FIRFilter<9> m_magFirFilterX;
	rc_utils::FIRFilter<9> m_magFirFilterY;
	rc_utils::FIRFilter<9> m_magFirFilterZ;
	stateestimation::MagFilter m_magHardIronFilter;
	void updateMagSpikeMaxDelta();

	//! Use servo model for command generation or just send position commands?
	config_server::Parameter<bool> m_useModel;

	//! Maximum delta in one period for effort setting
	config_server::Parameter<float> m_effortSlopeLimit;

	//! Maximum delta in one period for raw setting
	config_server::Parameter<float> m_rawStateSlopeLimit;

	//! Attempt communication with the servos?
	config_server::Parameter<bool> m_useServos;

	//! Publish estimated angle plots
	enum PMIDs
	{
		PM_ANGEST_PPITCH = 0,
		PM_ANGEST_PROLL,
		PM_ATTEST_FYAW,
		PM_ATTEST_FPITCH,
		PM_ATTEST_FROLL,
		PM_ATTEST_FHEMI,
		PM_ATTEST_BIAS_X,
		PM_ATTEST_BIAS_Y,
		PM_ATTEST_BIAS_Z,
		PM_ATTEST_NOMAG_FYAW,
		PM_ATTEST_NOMAG_FPITCH,
		PM_ATTEST_NOMAG_FROLL,
		PM_ATTEST_NOMAG_FHEMI,
		PM_ATTEST_NOMAG_BIAS_X,
		PM_ATTEST_NOMAG_BIAS_Y,
		PM_ATTEST_NOMAG_BIAS_Z,
		PM_ATTEST_YAW_FYAW,
		PM_ATTEST_YAW_FPITCH,
		PM_ATTEST_YAW_FROLL,
		PM_ATTEST_YAW_FHEMI,
		PM_ATTEST_YAW_BIAS_X,
		PM_ATTEST_YAW_BIAS_Y,
		PM_ATTEST_YAW_BIAS_Z,
		PM_GYRO_X,
		PM_GYRO_Y,
		PM_GYRO_Z,
		PM_GYRO_N,
		PM_GYROMEAN_X,
		PM_GYROMEAN_Y,
		PM_GYROMEAN_Z,
		PM_GYROMEAN_N,
		PM_GYROMEAN_SMOOTH_X,
		PM_GYROMEAN_SMOOTH_Y,
		PM_GYROMEAN_SMOOTH_Z,
		PM_GYROMEAN_SMOOTH_N,
		PM_GYRO_MEAN_OFFSET,
		PM_GYRO_STABLE_TIME,
		PM_GYRO_BIAS_TS,
		PM_GYRO_BIAS_ALPHA,
		PM_GYRO_SCALE_FACTOR,
		PM_ACC_XRAW,
		PM_ACC_YRAW,
		PM_ACC_ZRAW,
		PM_ACC_NRAW,
		PM_ACC_X,
		PM_ACC_Y,
		PM_ACC_Z,
		PM_ACC_N,
		PM_ACCMEAN_X,
		PM_ACCMEAN_Y,
		PM_ACCMEAN_Z,
		PM_ACCMEAN_N,
		PM_MAG_XRAW,
		PM_MAG_YRAW,
		PM_MAG_ZRAW,
		PM_MAG_XSPIKE,
		PM_MAG_YSPIKE,
		PM_MAG_ZSPIKE,
		PM_MAG_XIRON,
		PM_MAG_YIRON,
		PM_MAG_ZIRON,
		PM_MAG_X,
		PM_MAG_Y,
		PM_MAG_Z,
		PM_MAG_N,
		PM_TEMPERATURE,
		PM_VOLTAGE,
		PM_SENSOR_DT,
		PM_COUNT
	};
	plot_msgs::PlotManagerFS m_PM;
	config_server::Parameter<bool> m_plotRobotInterfaceData;
	void configurePlotManager();

	//! Publish button events
	ros::Publisher m_pub_buttons;
	
	//! Publish LED state
	ros::Publisher m_pub_led_state;

	// Button variables
	enum ButtonPressState
	{
		BPS_RELEASED = 0,
		BPS_SHORT_PRESS,
		BPS_LONG_PRESS,
		BPS_DO_RELAX,
		BPS_DO_UNRELAX,
		BPS_COUNT
	};
	uint8_t m_lastButtons;
	ros::Time m_buttonTime0;
	ros::Time m_buttonTime1;
	ros::Time m_buttonLastRec0;
	ButtonPressState m_buttonState0;
	ButtonPressState m_buttonState1;
	config_server::Parameter<bool> m_showButtonPresses;

	// Query set for the CM730
	std::vector<int> m_cm730_queryset;

	// Servo failure management
	bool m_commsOk;
	int m_totalFailCount;
	int m_consecFailCount;
	int m_commsSuspCount;
	int m_lastFailCount;
	int m_lastFailCountSev;
	int m_lastCommsSuspCount;
	bool m_enableCommsSusp;
	ros::Time m_timeLastFailCount;
	ros::Time m_timeLastFailCountSev;
	ros::Time m_timeLastCommsSusp;
	ros::Time m_timeLastSuspCheck;
	config_server::Parameter<bool> m_showServoFailures;

	// CM730 variables
	bool m_skipStep;
	bool m_cm730Suspend;
	ros::Time m_lastSensorTime;
	ros::Time m_lastCM730Time;

	// LED variables
	LEDCommand m_ledCommand;
	bool m_hadRX;
	bool m_hadTX;
	ros::Subscriber m_sub_led;
	config_server::Parameter<bool> m_robPause; // This shadows the robotcontrol pause configuration to pause the statistics timer when robotcontrol is paused

	// Buzzer variables
	ros::Subscriber m_sub_buzzer;
	bool m_haveBuzzerData;
	CM730BuzzerWriteData m_buzzerData;
	void handleBuzzerCommand(const BuzzerConstPtr& cmd);

	// Fade in/out on button press
	robotcontrol::RobotModel::State m_state_relaxed;
	robotcontrol::RobotModel::State m_state_setting_pose;
	actionlib::SimpleActionClient<robotcontrol::FadeTorqueAction> m_fadeTorqueClient;
	bool m_fadingIsTriggered;
	void sendFadeTorqueGoal(float torque);
	void resetFadingTriggered();

	// Logger heartbeat
	void sendLoggerHeartbeat();
	void sendLoggerHeartbeat(bool log);
	ros::Publisher m_pub_logger;
	rrlogger::LoggerHeartbeat m_loggerMsg;
	ros::Time m_loggerStamp;
};

}

#endif
// EOF
