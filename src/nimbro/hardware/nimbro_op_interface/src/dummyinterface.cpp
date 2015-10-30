// NimbRo-OP robot hardware interface (dummy)
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <nimbro_op_interface/dummyinterface.h>
#include <pluginlib/class_list_macros.h>
#include <rot_conv/rot_conv.h>
#include <cm730/dynamixel.h>

// Namespaces
using namespace nimbro_op_interface;

//
// DummyInterface class
//

// Default constructor
DummyInterface::DummyInterface()
 : CONFIG_PARAM_PATH("nopInterfaceDummy/")
 , m_useModel(CONFIG_PARAM_PATH + "useModel", false)
 , m_addDelay(CONFIG_PARAM_PATH + "addDelay", true)
 , m_noiseEnable(CONFIG_PARAM_PATH + "noise/enabled", true)
 , m_noiseMagnitude(CONFIG_PARAM_PATH + "noise/magnitude", 0.0, 0.0002, 0.04, 0.002)
 , m_buttonPress0(CONFIG_PARAM_PATH + "button/pressButton0", false)
 , m_buttonPress1(CONFIG_PARAM_PATH + "button/pressButton1", false)
 , m_buttonPress2(CONFIG_PARAM_PATH + "button/pressButton2", false)
 , m_fakeIMUGyroX(CONFIG_PARAM_PATH + "fakeIMU/gyroX", -2.0, 0.05, 2.0, 0.0)
 , m_fakeIMUGyroY(CONFIG_PARAM_PATH + "fakeIMU/gyroY", -2.0, 0.05, 2.0, 0.0)
 , m_fakeIMUGyroZ(CONFIG_PARAM_PATH + "fakeIMU/gyroZ", -2.0, 0.05, 2.0, 0.0)
 , m_fakeIMUAccX(CONFIG_PARAM_PATH + "fakeIMU/accX", -20.0, 0.2, 20.0, 0.0)
 , m_fakeIMUAccY(CONFIG_PARAM_PATH + "fakeIMU/accY", -20.0, 0.2, 20.0, 0.0)
 , m_fakeIMUAccZ(CONFIG_PARAM_PATH + "fakeIMU/accZ", -20.0, 0.2, 20.0, 9.81)
 , m_fakeIMUMagX(CONFIG_PARAM_PATH + "fakeIMU/magX", -2.0, 0.05, 20.0, 0.5)
 , m_fakeIMUMagY(CONFIG_PARAM_PATH + "fakeIMU/magY", -2.0, 0.05, 20.0, 0.0)
 , m_fakeIMUMagZ(CONFIG_PARAM_PATH + "fakeIMU/magZ", -2.0, 0.05, 20.0, 0.0)
 , m_fakeAttEnable(CONFIG_PARAM_PATH + "fakeAttitude/enabled", true)
 , m_fakeAttFusedX(CONFIG_PARAM_PATH + "fakeAttitude/fusedX", -M_PI_2, 0.01, M_PI_2, 0.0)
 , m_fakeAttFusedY(CONFIG_PARAM_PATH + "fakeAttitude/fusedY", -M_PI_2, 0.01, M_PI_2, 0.0)
 , m_fakeAttFusedZ(CONFIG_PARAM_PATH + "fakeAttitude/fusedZ", -M_PI, 0.02, M_PI, 0.0)
 , m_fakeAttFusedHemi(CONFIG_PARAM_PATH + "fakeAttitude/fusedHemi", true)
 , m_calledReadFeedback(false)
 , m_jointCmdBuf(5) // Note: This is the size of the joint position circular buffer, which determines the delay (number of cycles) introduced by the dummy
{
	// Reset the button press config parameters just in case
	m_buttonPress0.set(false);
	m_buttonPress1.set(false);
	m_buttonPress2.set(false);
}

// Destructor
DummyInterface::~DummyInterface()
{
}

// Send joint targets function
bool DummyInterface::sendJointTargets()
{
	// Pass on the work to the base implementation
	return RobotInterface::sendJointTargets();
}

// Read joint states function
bool DummyInterface::readJointStates()
{
	// Reset the flag whether readFeedbackData() was called
	m_calledReadFeedback = false;
	
	// Pass on the work to the base implementation
	bool ret = RobotInterface::readJointStates();
	
	// See whether the fake attitude should be written to the robot model
	if(m_calledReadFeedback && m_fakeAttEnable())
	{
		Eigen::Quaterniond fakeAtt = rot_conv::QuatFromFused(m_fakeAttFusedZ(), m_fakeAttFusedY(), m_fakeAttFusedX(), m_fakeAttFusedHemi());
		m_model->setRobotOrientation(fakeAtt);
	}
	
	// Return whether reading the joint states was successful
	return ret;
}

// Read feedback data
int DummyInterface::readFeedbackData(bool onlyTryCM730)
{
	// Indicate that this function was called
	m_calledReadFeedback = true;
	
	//
	// Update feedback from the CM730: m_boardData
	//
	
	// Populate the header data
	m_boardData.id = CM730::ID_CM730;
	m_boardData.length = CM730::READ_CM730_LENGTH;
	m_boardData.startAddress = CM730::READ_SERVO_ADDRESS;
	
	// Miscellaneous
	m_boardData.power = 1; // The servos always have power
	m_boardData.ledPanel = 0; // No LEDs are on
	m_boardData.rgbled5 = 0; // RGBLED5 is off
	m_boardData.rgbled6 = 0; // RGBLED6 is off
	m_boardData.voltage = (unsigned char) ((15.0 / INT_TO_VOLTS) + 0.5); // The board voltage is always 15.0V
	
	// Button presses
	unsigned char button = 0x00;
	if(m_buttonPress0()) { button |= 0x01; m_buttonPress0.set(false); }
	if(m_buttonPress1()) { button |= 0x02; m_buttonPress1.set(false); }
	if(m_buttonPress2()) { button |= 0x04; m_buttonPress2.set(false); }
	m_boardData.button = button;
	
	// IMU data
	m_boardData.gyroX = m_fakeIMUGyroX() / GYRO_SCALE;
	m_boardData.gyroY = m_fakeIMUGyroY() / GYRO_SCALE;
	m_boardData.gyroZ = m_fakeIMUGyroZ() / GYRO_SCALE;
	m_boardData.accX = m_fakeIMUAccX() / ACC_SCALE;
	m_boardData.accY = m_fakeIMUAccY() / ACC_SCALE;
	m_boardData.accZ = m_fakeIMUAccZ() / ACC_SCALE;
	m_boardData.magX = m_fakeIMUMagX() / MAG_SCALE;
	m_boardData.magY = m_fakeIMUMagY() / MAG_SCALE;
	m_boardData.magZ = m_fakeIMUMagZ() / MAG_SCALE;

	//
	// Update feedback from the servos: m_servoData
	//
	
	// Only do servo feedback if we're supposed to contact the servos
	if(!onlyTryCM730)
	{
		// Write the required joint feedback into m_servoData
		const JointCmd& jointCmd = (m_addDelay() ? m_jointCmdBuf.front() : m_jointCmdBuf.back());
		for(JointCmd::const_iterator it = jointCmd.begin(); it != jointCmd.end(); ++it)
		{
			// Retrieve a reference to the required read data struct
			uint8_t id = it->first;
			if(id > m_servoData.size())
				m_servoData.resize(id);
			BRData& servoData = m_servoData.at(id - 1); // Minus 1 as id's are stored zero-based, which 1 being the first ID
			
			// Populate the header data
			servoData.id = id;
			servoData.length = CM730::READ_SERVO_LENGTH;
			servoData.startAddress = CM730::READ_SERVO_ADDRESS;
			
			// Set the servo feedback position
			int servoPos = it->second.goal_position;
			if(m_noiseEnable())
				servoPos += (int)(TICKS_PER_RAD * m_noiseMagnitude() * (drand48() - 0.5) + 0.5);
			servoData.position = servoPos;
		}
	}

	// Return success
	return CM730::RET_SUCCESS;
}

// Write the joint targets to the dummy robot
bool DummyInterface::syncWriteJointTargets(size_t numDevices, const uint8_t* data)
{
	// Re-cast the data pointer to an array of numDevices JointCmdSyncWriteData structs
	const JointCmdSyncWriteData* jointData = (const JointCmdSyncWriteData*) data;
	
	// Insert the commanded targets into the joint command circular buffer
	JointCmd jointCmd;
	for(size_t i = 0; i < numDevices; i++)
		jointCmd[jointData[i].id] = jointData[i];
	m_jointCmdBuf.push_back(jointCmd);

	// Return that we have successfully written the joint targets
	return true;
}

PLUGINLIB_EXPORT_CLASS(nimbro_op_interface::DummyInterface, robotcontrol::HardwareInterface);
// EOF