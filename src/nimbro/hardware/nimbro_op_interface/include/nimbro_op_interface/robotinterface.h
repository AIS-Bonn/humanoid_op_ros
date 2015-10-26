// NimbRo-OP robot hardware interface
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//         Sebastian Schueller <schuell1@cs.uni-bonn.de>
//         Max Schwarz <max.schwarz@uni-bonn.de>

// Ensure header is only included once
#ifndef RC_ROBOTINTERFACE_H
#define RC_ROBOTINTERFACE_H

// Includes
#include <robotcontrol/model/robotmodel.h>
#include <robotcontrol/model/firfilter.h>
#include <robotcontrol/model/golay.h>
#include <robotcontrol/hw/hardwareinterface.h>
#include <robotcontrol/hw/angleestimator.h>
#include <robotcontrol/hw/magfilter.h>
#include <robotcontrol/hw/attitude_estimator.h>
#include <robotcontrol/FadeTorqueAction.h>
#include <robotcontrol/ServoDiag.h>
#include <nimbro_utils/spike_filter.h>
#include <nimbro_op_interface/LEDCommand.h>
#include <nimbro_op_interface/ReadOffset.h>
#include <nimbro_op_interface/AttEstMagCalib.h>
#include <actionlib/client/simple_action_client.h>
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
	virtual boost::shared_ptr<robotcontrol::Joint> createJoint(const std::string& name);
	virtual bool sendJointTargets();
	virtual bool readJointStates();
	virtual bool setStiffness(float torque);
	virtual void getDiagnostics(robotcontrol::DiagnosticsPtr ptr);

protected:
	// Virtual functions for hardware abstraction
	virtual bool initCM730();
	virtual int  readFeedbackData(bool onlyTryCM730);
	virtual bool syncWriteJointTargets(size_t numDevices, const uint8_t* data);
	virtual bool syncWriteTorqueEnable(size_t numDevices, const uint8_t* data);
	virtual bool syncWriteTorqueLimit(size_t numDevices, const uint8_t* data);
	virtual bool useModel() const { return m_useModel(); }
	
	// Scaling constants
	static const double TICKS_PER_RAD = 2048.0 / M_PI; // From the fact that a complete revolution is 4096 ticks
	static const double FULL_TORQUE = 1023.0; // The torque limit value that corresponds to full torque
	static const double FULL_EFFORT = 32.0; // The P gain value that an effort of 1.0 corresponds to
	static const double INT_TO_VOLTS = 0.1; // Multiply a voltage value read from the CM730 by this to convert it into volts (see 'cm730/firmware/CM730_HW/src/adc.c')
	static const double GYRO_SCALE = (M_PI / 180) * (2 * 250) / 65536; // From control register CTRL_REG4 in "cm730/firmware/CM730_HW/src/gyro_acc.c": +-250dps is 65536 LSb
	static const double ACC_SCALE = (2 * (4 * 9.81)) / 65536; // From control register CTRL_REG4 in "cm730/firmware/CM730_HW/src/gyro_acc.c": +-4g is 65536 LSb
	static const double MAG_SCALE = 1 / 820.0; // From Config Register B in "cm730/firmware/CM730_APP/src/compass.c": 820 LSb/gauss

	//! The robot model
	robotcontrol::RobotModel* m_model;

	//! Bulk read buffer for servo feedback
	std::vector<BRData> m_servoData;

	//! Bulk read buffer for CM730 feedback (e.g. IMU)
	BRBoard m_boardData;

	/**
	* @internal Helper struct that reflects the layout of an individual LED command write.
	* This struct is for use with the sendCM730LedData() function.
	**/
	struct CM730LedWriteData
	{
		uint8_t id;        //!< CM730 ID
		uint8_t led_panel; //!< P_LED_PANEL
		uint16_t rgbled5;  //!< P_RGBLED5_L / P_RGBLED5_H
		uint16_t rgbled6;  //!< P_RGBLED6_L / P_RGBLED6_H
	} __attribute__((packed));

	/**
	* @internal This is a helper struct for the SyncWrite command which transfers all
	* command values to the servo actuators. The memory layout reflects the actual layout
	* in the message and in the Dynamixel hardware, hence the `__attribute__((packed))`.
	**/
	struct JointCmdSyncWriteData
	{
		uint8_t id;             //!< Servo ID
		uint8_t p_gain;         //!< P_P_GAIN
		uint8_t nothing;        //!< Unused register
		uint16_t goal_position; //!< P_GOAL_POSITION_L / P_GOAL_POSITION_H
	} __attribute__((packed));

	/**
	* @internal Helper struct that reflects the layout of an individual torque enable sync write command.
	* This struct is for use with the setStiffness() function, and allows for the enabling and disabling
	* of individual servo torques.
	**/
	struct TorqueEnableSyncWriteData
	{
		uint8_t id;            //!< Servo ID
		uint8_t torque_enable; //!< P_TORQUE_ENABLE_L / P_TORQUE_ENABLE_H
	} __attribute__((packed));

	/**
	* @internal Helper struct that reflects the layout of an individual torque limit sync write command.
	* This struct is for use with the setStiffness() function, and allows for torque fading.
	**/
	struct TorqueLimitSyncWriteData
	{
		uint8_t id;            //!< Servo ID
		uint16_t torque_limit; //!< P_TORQUE_LIMIT_L / P_TORQUE_LIMIT_H
	} __attribute__((packed));

	/**
	* @internal Layout of the statistics data requested from the servo.
	**/
	struct StatisticsReadData
	{
		uint8_t voltage;     //!< P_PRESENT_VOLTAGE
		uint8_t temperature; //!< P_PRESENT_TEMPERATURE
	} __attribute__((packed));

private:
	// Constants
	const std::string CONFIG_PARAM_PATH;

	// Flag whether the real hardware is present (assumes no unless otherwise told)
	bool m_haveHardware;
	
	/**
	 * @brief Joint class with hardware information
	 **/
	struct DXLJoint : public robotcontrol::Joint
	{
		DXLJoint(const std::string& name);

		//! Servo command generator appropiate for the servo type
		robotcontrol::CommandGeneratorPtr commandGenerator;

		// Constants
		const std::string CONFIG_PARAM_PATH;

		//! @name config_server joint parameters
		//@{
		config_server::Parameter<std::string> type;  //!< Actuator type
		config_server::Parameter<int> id;            //!< Actuator address
		config_server::Parameter<int> tickOffset;    //!< Joint offset in ticks
		config_server::Parameter<bool> invert;       //!< Invert the joint direction
		config_server::Parameter<bool> readFeedback; //!< Read enabled?
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

	//! Process changed joint settings
	void updateJointSettings(DXLJoint* joint);
	void setJointFeedbackTime(const ros::Time stamp);

	//! Get an appropiate command generator for a servo type
	robotcontrol::CommandGeneratorPtr commandGenerator(const std::string& type);

	// Topic handlers
	void handleStatistics(const ros::TimerEvent&);
	void handleLEDCommand(const LEDCommand& cmd);

	//! Handle button press
	void handleButton(int number);
	config_server::Parameter<bool> m_buttonPress0;
	config_server::Parameter<bool> m_buttonPress1;
	config_server::Parameter<bool> m_buttonPress2;

	/**
	 * @brief Get the DXLJoint for an index
	 *
	 * @param idx Index in the RobotModel joint struct array (not servo ID)
	 */
	inline DXLJoint* dxlJoint(size_t idx) { return (DXLJoint*) m_model->joint(idx).get(); }

	void sendCM730LedCommand();

	DXLJoint* dxlJointForID(int id);

	//! Our hardware driver
	boost::shared_ptr<CM730> m_board;

	//! All servo types and generators we know about
	std::map<std::string, robotcontrol::CommandGeneratorPtr> m_generators;

	//! All joints relaxed?
	bool m_relaxed;

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

	//! Attitude estimator
	stateestimation::AttitudeEstimator m_attitudeEstimator; // Attitude estimator instance
	stateestimation::AttitudeEstimator m_attEstNoMag;       // Attitude estimator instance (no magnetometer)
	config_server::Parameter<bool> m_attEstUseFusedMethod;  // Boolean flag specifying which acc-only resolution method to use
	void updateAttEstMethod();                         // Transcribes the value from the config server parameters to the internals of the attitude estimator
	config_server::Parameter<float> m_attEstKp;        // Kp parameter (P gain) to use for the attitude estimator
	config_server::Parameter<float> m_attEstTi;        // Ti parameter (integral time constant) to use for the attitude estimator
	config_server::Parameter<float> m_attEstKpQuick;   // Maximum Kp parameter (P gain) to use for the attitude estimator during quick learning
	config_server::Parameter<float> m_attEstTiQuick;   // Maximum Ti parameter (integral time constant) to use for the attitude estimator during quick learning
	void updateAttEstPIGains();                        // Transcribes the values from the config server parameters to the internals of the attitude estimator
	config_server::Parameter<float> m_attEstMagCalibX; // m_attEstMagCalib_xyz should be equal to the vector value of m_magFilter.value() [i.e. RobotModel::magneticFieldVector()]
	config_server::Parameter<float> m_attEstMagCalibY; // when the robot is completely upright and with its coordinate axes perfectly aligned with the global coordinate frame.
	config_server::Parameter<float> m_attEstMagCalibZ; // Nominally this is when the robot has zero pitch/roll and is facing in the direction of the positive goal.
	void updateAttEstMagCalib();                       // Transcribes the values from the config server parameters to the internals of the attitude estimator

	//! Service to set the attitude estimation magnetometer calibration value
	ros::ServiceServer m_srv_attEstCalibrate;
	bool handleAttEstCalibrate(nimbro_op_interface::AttEstMagCalibRequest& req, nimbro_op_interface::AttEstMagCalibResponse& resp);

	//! Fused angle estimator
	stateestimation::AngleEstimator m_angleEstimator;

	//! Magnetometer data filter
	config_server::Parameter<bool> m_useMagnetometer;
	config_server::Parameter<float> m_magSpikeMaxDelta;
	nimbro_utils::SpikeFilter m_magSpikeFilterX;
	nimbro_utils::SpikeFilter m_magSpikeFilterY;
	nimbro_utils::SpikeFilter m_magSpikeFilterZ;
	robotcontrol::FIRFilter<9> m_magFirFilterX;
	robotcontrol::FIRFilter<9> m_magFirFilterY;
	robotcontrol::FIRFilter<9> m_magFirFilterZ;
	stateestimation::MagFilter m_magHardIronFilter;
	void updateMagSpikeMaxDelta();

	//! Use servo model for command generation or just send position commands?
	config_server::Parameter<bool> m_useModel;

	//! Maximum delta in one period for effort setting
	config_server::Parameter<float> m_effortSlopeLimit;

	//! Maximum delta in one period for raw setting
	config_server::Parameter<float> m_rawStateSlopeLimit;

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
		PM_GYRO_X,
		PM_GYRO_Y,
		PM_GYRO_Z,
		PM_GYRO_N,
		PM_ACC_XRAW,
		PM_ACC_YRAW,
		PM_ACC_ZRAW,
		PM_ACC_NRAW,
		PM_ACC_X,
		PM_ACC_Y,
		PM_ACC_Z,
		PM_ACC_N,
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
		PM_SENSOR_DT,
		PM_COUNT
	};
	plot_msgs::PlotManagerFS m_PM;
	config_server::Parameter<bool> m_plotRobotInterfaceData;

	//! Publish button events
	ros::Publisher m_pub_buttons;
	
	//! Publish LED state
	ros::Publisher m_pub_led_state;

	//! Last button mask
	uint8_t m_lastButtons;

	//! Action client for the torque fading
	actionlib::SimpleActionClient<robotcontrol::FadeTorqueAction> m_act_fadeTorque;

	std::vector<int> m_cm730_queryset;

	// Servo failure management
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

	bool m_skipStep;
	bool m_cm730Suspend;
	ros::Time m_lastSensorTime;
	ros::Time m_lastCM730Time;

	LEDCommand m_ledCommand;
	ros::Subscriber m_sub_led;
	config_server::Parameter<bool> m_robPause;

	// FIR filters for accelerometer data smoothing
	robotcontrol::FIRFilter<9> m_fir_accX;
	robotcontrol::FIRFilter<9> m_fir_accY;
	robotcontrol::FIRFilter<11> m_fir_accZ;

	//! Used for fadein/out on button press
	robotcontrol::RobotModel::State m_state_relaxed;
	actionlib::SimpleActionClient<robotcontrol::FadeTorqueAction> m_fadeTorqueClient;
	bool m_fadingIsTriggered;
	void resetFadingTriggered();
};

}

#endif
// EOF