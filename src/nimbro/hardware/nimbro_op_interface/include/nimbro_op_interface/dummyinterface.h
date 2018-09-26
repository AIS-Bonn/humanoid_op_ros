// NimbRo-OP robot hardware interface (dummy)
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef NOP_DUMMY_INTERFACE_H
#define NOP_DUMMY_INTERFACE_H

// Includes
#include <nimbro_op_interface/robotinterface.h>
#include <boost/circular_buffer.hpp>
#include <config_server/parameter.h>

// NimbRo-OP interface namespace
namespace nimbro_op_interface
{
	// DummyInterface class
	class DummyInterface : public virtual RobotInterface
	{
	public:
		// Constructor/destructor
		DummyInterface();
		virtual ~DummyInterface();

		// Virtual function overrides
		virtual bool sendJointTargets() override;
		virtual bool readJointStates() override;

		// Constants
		static const std::string CONFIG_PARAM_PATH;

	protected:
		// Virtual functions to override attempts to connect to the CM730
		virtual bool initCM730() override { return true; }
		virtual int  readFeedbackData(bool onlyTryCM730) override;
		virtual bool syncWriteJointTargets(const std::vector<JointCmdData>& jointCmdData) override;
		virtual bool syncWriteReturnLevel(size_t numDevices, const uint8_t* data) override { return true; }
		virtual bool syncWriteTorqueEnable(size_t numDevices, const uint8_t* data) override { return true; }
		virtual bool syncWriteTorqueLimit(size_t numDevices, const uint8_t* data) override { return true; }
		virtual bool useModel() const override { return m_useModel(); }

	private:
		// Config server parameters
		config_server::Parameter<bool>  m_useModel;
		config_server::Parameter<bool>  m_addDelay;
		config_server::Parameter<bool>  m_noiseEnable;
		config_server::Parameter<float> m_noiseMagnitude;
		config_server::Parameter<bool>  m_buttonPress0;
		config_server::Parameter<bool>  m_buttonPress1;
		config_server::Parameter<bool>  m_buttonPress2;
		config_server::Parameter<int>   m_fakeTemperature;
		config_server::Parameter<float> m_fakeIMUGyroX;
		config_server::Parameter<float> m_fakeIMUGyroY;
		config_server::Parameter<float> m_fakeIMUGyroZ;
		config_server::Parameter<float> m_fakeIMUAccX;
		config_server::Parameter<float> m_fakeIMUAccY;
		config_server::Parameter<float> m_fakeIMUAccZ;
		config_server::Parameter<float> m_fakeIMUMagX;
		config_server::Parameter<float> m_fakeIMUMagY;
		config_server::Parameter<float> m_fakeIMUMagZ;
		config_server::Parameter<bool>  m_fakeAttEnable;
		config_server::Parameter<float> m_fakeAttFusedX;
		config_server::Parameter<float> m_fakeAttFusedY;
		config_server::Parameter<float> m_fakeAttFusedZ;
		config_server::Parameter<bool>  m_fakeAttFusedHemi;

		// Config server callbacks
		void updateUseModel();

		// Misc flags
		bool m_calledReadFeedback;

		// Joint command data buffer
		typedef std::map<int, JointCmdData> JointCmd;
		boost::circular_buffer<JointCmd> m_jointCmdBuf;
	};
}

#endif
// EOF
