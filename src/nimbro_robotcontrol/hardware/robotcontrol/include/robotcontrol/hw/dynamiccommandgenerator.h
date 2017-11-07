// Configurable command generator
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DYNAMICCOMMANDGENERATOR_H
#define DYNAMICCOMMANDGENERATOR_H

#include <servomodel/servocommandgenerator.h>
#include <config_server/parameter.h>

namespace robotcontrol
{

/**
 * @brief Configurable servo command generator
 *
 * This class stores the command generator parameters on the config_server.
 **/
class DynamicCommandGenerator : public ServoCommandGenerator
{
public:
	DynamicCommandGenerator(const std::string& conf_prefix);
	virtual ~DynamicCommandGenerator();

private:
	config_server::Parameter<float> m_param_km;
	config_server::Parameter<float> m_param_viscuous;
	config_server::Parameter<float> m_param_stribeckOne;
	config_server::Parameter<float> m_param_stribeckTwo;
	config_server::Parameter<float> m_param_ticksPerRev;
	config_server::Parameter<int> m_param_minTickValue;
	config_server::Parameter<int> m_param_maxTickValue;

	void updateParameters();
};

}

#endif
