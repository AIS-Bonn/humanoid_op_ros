// Dummy hardware interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
//         Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

#ifndef DUMMYINTERFACE_H
#define DUMMYINTERFACE_H

#include <robotcontrol/hw/hardwareinterface.h>
#include <robotcontrol/model/joint.h>
#include <config_server/parameter.h>
#include <boost/circular_buffer.hpp>

namespace robotcontrol
{

/**
 * @brief Dummy hardware interface
 *
 * This provides a simple loopback hardware interface. The joint commands
 * are simply stored and returned as position feedback.
 **/
class DummyInterface : public HardwareInterface
{
public:
	DummyInterface();
	virtual ~DummyInterface();

	virtual bool init(RobotModel* model);
	virtual boost::shared_ptr<Joint> createJoint(const std::string& name);
	virtual void getDiagnostics(robotcontrol::DiagnosticsPtr ptr);
	virtual bool readJointStates();
	virtual bool sendJointTargets();
	virtual bool setStiffness(float torque);

private:
	// Constants
	const std::string CONFIG_PARAM_PATH;

	RobotModel* m_model;
	
	struct DummyJoint : public Joint
	{
		explicit DummyJoint(const std::string& name);
		config_server::Parameter<bool> velocityMode;
	};

	// Config server parameters
	config_server::Parameter<bool>  m_addDelay;
	config_server::Parameter<bool>  m_noiseEnable;
	config_server::Parameter<float> m_noiseMagnitude;
	config_server::Parameter<float> m_fakeIMUGyroX;
	config_server::Parameter<float> m_fakeIMUGyroY;
	config_server::Parameter<float> m_fakeIMUGyroZ;
	config_server::Parameter<float> m_fakeIMUAccX;
	config_server::Parameter<float> m_fakeIMUAccY;
	config_server::Parameter<float> m_fakeIMUAccZ;
	config_server::Parameter<float> m_fakeIMUMagX;
	config_server::Parameter<float> m_fakeIMUMagY;
	config_server::Parameter<float> m_fakeIMUMagZ;
	config_server::Parameter<float> m_fakeAttFusedX;
	config_server::Parameter<float> m_fakeAttFusedY;

	typedef boost::circular_buffer<std::vector<double> > DataBuf;
	DataBuf m_dataBuf;
};

}

#endif
