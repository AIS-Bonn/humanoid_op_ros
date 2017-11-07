// Configurable command generator
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <robotcontrol/hw/dynamiccommandgenerator.h>
#include <boost/bind.hpp>

namespace robotcontrol
{

DynamicCommandGenerator::DynamicCommandGenerator(const std::string& conf_prefix)
 : ServoCommandGenerator()
 , m_param_km(conf_prefix + "km", 0.0, 0.01, 1.0, 0.2)
 , m_param_viscuous(conf_prefix + "viscuous", 0.0, 0.01, 1.0, 0.5)
 , m_param_stribeckOne(conf_prefix + "stribeckOne", -0.005, 0.001, 1.0, 0.0)
 , m_param_stribeckTwo(conf_prefix + "stribeckTwo", 0.0, 0.001, 1.0, 0.0)
 , m_param_ticksPerRev(conf_prefix + "ticksPerRev", 64.0, 64.0, 16384.0, DefaultTicksPerRev)
 , m_param_minTickValue(conf_prefix + "minTickValue", 0, 64, 16384, 0)
 , m_param_maxTickValue(conf_prefix + "maxTickValue", 0, 64, 16384, DefaultTicksPerRev - 1)
{
	m_param_km.setCallback(boost::bind(&DynamicCommandGenerator::updateParameters, this));
	m_param_viscuous.setCallback(boost::bind(&DynamicCommandGenerator::updateParameters, this));
	m_param_stribeckOne.setCallback(boost::bind(&DynamicCommandGenerator::updateParameters, this));
	m_param_stribeckTwo.setCallback(boost::bind(&DynamicCommandGenerator::updateParameters, this));
	m_param_ticksPerRev.setCallback(boost::bind(&DynamicCommandGenerator::updateParameters, this));
	m_param_minTickValue.setCallback(boost::bind(&DynamicCommandGenerator::updateParameters, this));
	m_param_maxTickValue.setCallback(boost::bind(&DynamicCommandGenerator::updateParameters, this));
	updateParameters();
}

DynamicCommandGenerator::~DynamicCommandGenerator()
{
}

void DynamicCommandGenerator::updateParameters()
{
	setKM(m_param_km());
	setViscousFriction(m_param_viscuous());
	setStribeckOne(m_param_stribeckOne());
	setStribeckTwo(m_param_stribeckTwo());
	setTicksPerRev(m_param_ticksPerRev());
	setMinTickValue(m_param_minTickValue());
	setMaxTickValue(m_param_maxTickValue());
}

}
