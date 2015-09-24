// Dummy hardware interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
//         Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <robotcontrol/hw/dummyinterface.h>
#include <robotcontrol/model/robotmodel.h>
#include <pluginlib/class_list_macros.h>
#include <boost/make_shared.hpp>
#include <rot_conv/rot_conv.h>
#include <angles/angles.h>
#include <ros/console.h>
#include <cmath>

namespace robotcontrol
{

DummyInterface::DummyJoint::DummyJoint(const std::string& name)
 : velocityMode("joints/" + name + "/velocityMode", false)
{
}

DummyInterface::DummyInterface()
 : CONFIG_PARAM_PATH("dummyInterface/")
 , m_model(NULL)
 , m_addDelay(CONFIG_PARAM_PATH + "addDelay", true)
 , m_noiseEnable(CONFIG_PARAM_PATH + "noise/enabled", true)
 , m_noiseMagnitude(CONFIG_PARAM_PATH + "noise/magnitude", 0.0, 0.0002, 0.04, 0.002)
 , m_fakeIMUGyroX(CONFIG_PARAM_PATH + "fakeIMU/gyroX", -2.0, 0.05, 2.0, 0.0)
 , m_fakeIMUGyroY(CONFIG_PARAM_PATH + "fakeIMU/gyroY", -2.0, 0.05, 2.0, 0.0)
 , m_fakeIMUGyroZ(CONFIG_PARAM_PATH + "fakeIMU/gyroZ", -2.0, 0.05, 2.0, 0.0)
 , m_fakeIMUAccX(CONFIG_PARAM_PATH + "fakeIMU/accX", -20.0, 0.2, 20.0, 0.0)
 , m_fakeIMUAccY(CONFIG_PARAM_PATH + "fakeIMU/accY", -20.0, 0.2, 20.0, 0.0)
 , m_fakeIMUAccZ(CONFIG_PARAM_PATH + "fakeIMU/accZ", -20.0, 0.2, 20.0, 9.81)
 , m_fakeIMUMagX(CONFIG_PARAM_PATH + "fakeIMU/magX", -2.0, 0.05, 20.0, 0.5)
 , m_fakeIMUMagY(CONFIG_PARAM_PATH + "fakeIMU/magY", -2.0, 0.05, 20.0, 0.0)
 , m_fakeIMUMagZ(CONFIG_PARAM_PATH + "fakeIMU/magZ", -2.0, 0.05, 20.0, 0.0)
 , m_fakeAttFusedX(CONFIG_PARAM_PATH + "fakeAttitude/fusedX", -M_PI_2, 0.01, M_PI_2, 0.0)
 , m_fakeAttFusedY(CONFIG_PARAM_PATH + "fakeAttitude/fusedY", -M_PI_2, 0.01, M_PI_2, 0.0)
 , m_dataBuf(5) // Note: This is the size of the circular data buffer, which determines the delay (number of cycles) introduced by the dummy
{
}

DummyInterface::~DummyInterface()
{
}

bool DummyInterface::init(RobotModel* model)
{
	m_model = model;
	return true;
}

void DummyInterface::getDiagnostics(robotcontrol::DiagnosticsPtr ptr)
{
	ptr->header.stamp = ros::Time::now();
	ptr->batteryVoltage = 14.0;
	ptr->servoTemperature = 30.0;
}

boost::shared_ptr< Joint > DummyInterface::createJoint(const std::string& name)
{
	Joint::Ptr joint(new DummyJoint(name));
	joint->name = name;
	joint->feedback.pos = 0;

	return joint;
}

bool DummyInterface::readJointStates()
{
	// Save the current ROS time
	ros::Time now = ros::Time::now();

	// Set the feedback positions with some delay and some noise
	std::vector<double> cmds = (m_addDelay() ? m_dataBuf.front() : m_dataBuf.back());
	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		const boost::shared_ptr<DummyJoint>& joint = boost::reinterpret_pointer_cast<DummyJoint>(m_model->joint(i));

		double noisyCmd = cmds[i];
		if(m_noiseEnable())
			noisyCmd += (drand48() - 0.5) * m_noiseMagnitude();

		if(joint->velocityMode())
		{
			joint->feedback.pos += noisyCmd * m_model->timerDuration();
			joint->feedback.pos = angles::normalize_angle(joint->feedback.pos);
		}
		else
			joint->feedback.pos = noisyCmd;

		joint->feedback.stamp = now;
	}

	// Calculate the fake orientation of the robot (nominal values taken from RobotModel constructor)
	Eigen::Quaterniond fakeQuat = rot_conv::QuatFromFused(m_fakeAttFusedY(), m_fakeAttFusedX());
	Eigen::Vector3d gyro(m_fakeIMUGyroX(), m_fakeIMUGyroY(), m_fakeIMUGyroZ());
	Eigen::Vector3d acc(m_fakeIMUAccX(), m_fakeIMUAccY(), m_fakeIMUAccZ());
	Eigen::Vector3d mag(m_fakeIMUMagX(), m_fakeIMUMagY(), m_fakeIMUMagZ());

	// Write dummy values into the RobotModel
	m_model->setRobotOrientation(fakeQuat);
	m_model->setAccelerationVector(acc);
	m_model->setMagneticFieldVector(mag);
	m_model->setRobotAngularVelocity(gyro);

	// Return success
	return true;
}

bool DummyInterface::sendJointTargets()
{
	std::vector<double> cmds(m_model->numJoints());
	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		const boost::shared_ptr<DummyJoint>& joint = boost::reinterpret_pointer_cast<DummyJoint>(m_model->joint(i));

		if(joint->velocityMode())
			cmds[i] = joint->cmd.vel;
		else
		{
			double cmd = joint->cmd.pos;

			// Enforce urdf position limits
			if(joint->modelJoint->type == urdf::Joint::REVOLUTE)
			{
				const boost::shared_ptr<urdf::JointLimits>& limits = joint->modelJoint->limits;
				if(limits)
				{
					if(cmd > limits->upper)
					{
						ROS_ERROR("Position controlled joint '%s' out of bounds! Constraining position %f to upper limit %f!",
							joint->name.c_str(),
							cmd, limits->upper
						);
						cmd = limits->upper;
					}
					else if(cmd < limits->lower)
					{
						ROS_ERROR("Position controlled joint '%s' out of bounds! Constraining position %f to lower limit %f!",
							joint->name.c_str(),
							cmd, limits->lower
						);
						cmd = limits->lower;
					}
				}
			}
			cmds[i] = cmd;
		}

		joint->cmd.rawPos = joint->cmd.pos;
	}

	m_dataBuf.push_back(cmds);

	return true;
}

bool DummyInterface::setStiffness(float torque)
{
	// Display a throttled info message that fading is active
	ROS_INFO_THROTTLE(0.4, "Fading is active (%.3f)", torque);

	// Return that all is ok and we've handled it
	return true;
}

}

PLUGINLIB_EXPORT_CLASS(robotcontrol::DummyInterface, robotcontrol::HardwareInterface)
// EOF
