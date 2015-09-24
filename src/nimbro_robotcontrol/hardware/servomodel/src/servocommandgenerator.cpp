// Servo command generator
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/node_handle.h>

#include <servomodel/servocommandgenerator.h>

#include <math.h>
#include <Eigen/Core>

using namespace Eigen;

ServoCommandGenerator::ServoCommandGenerator()
 : m_coeff(4)
 , m_pValue(2)
 , m_latency(0)
 , m_voltage(15.0)
{
	m_coeff << 0.00180, 0.02960, 0.27590, 0.27090;
}

inline double sgn(double x)
{
	if(x > 0.0) return 1.0;
	else if(x < 0.0) return -1.0;
	else return 0.0;
}

VectorXd ServoCommandGenerator::commandPartsFor(double pos, double vel, double acc, double outsideTorque) const
{
	VectorXd ret(4);
	// Motor model
	double stribeckFactor = exp(-fabs(vel / 0.1));
	ret <<
		outsideTorque,
		vel,                           // viscous friction / back-EMF
		sgn(vel) * (1-stribeckFactor), // stribeck I
		sgn(vel) * stribeckFactor      // stribeck II
	;

	return ret;
}

double ServoCommandGenerator::currentFactor() const
{
	return (15.0 / m_voltage) / m_pValue * 2.0;
}

double ServoCommandGenerator::servoCommandFor(double pos, double vel, double acc, double outside) const
{
	return pos + currentFactor() * (
		m_coeff.dot(commandPartsFor(pos, vel, acc, outside))
	);
}

double ServoCommandGenerator::servoTorqueFromCommand(double pos_cmd, double pos_cur, double vel) const
{
	Eigen::VectorXd frictionParts = commandPartsFor(0.0, vel, 0.0, 0.0);

	double d = (pos_cmd - pos_cur) / currentFactor();

	double torquePart = d - m_coeff.dot(frictionParts);

	return torquePart / m_coeff(0);
}

void ServoCommandGenerator::setCoefficients(const VectorXd& coeff)
{
	m_coeff = coeff;
	update();
}

void ServoCommandGenerator::setStribeckOne(double val)
{
	m_coeff(2) = val;
	update();
}

void ServoCommandGenerator::setStribeckTwo(double val)
{
	m_coeff(3) = val;
	update();
}

void ServoCommandGenerator::setViscousFriction(double val)
{
	m_coeff(1) = val;
	update();
}

void ServoCommandGenerator::setKM(double val)
{
	m_coeff(0) = val;
	update();
}

void ServoCommandGenerator::setPValue(int p)
{
	m_pValue = p;
	update();
}

void ServoCommandGenerator::setLatency(double val)
{
	m_latency = val;
	update();
}

void ServoCommandGenerator::setVoltage(double volt)
{
	m_voltage = volt;
	update();
}

void ServoCommandGenerator::update()
{
}

std::string ServoCommandGenerator::serializeCoefficients()
{
	std::stringstream ss;

	for(int i = 0; i < m_coeff.rows(); ++i)
		ss << m_coeff(i) << " ";

	return ss.str();
}

bool ServoCommandGenerator::deserializeCoefficients(const std::string& coeff)
{
	std::stringstream ss;
	ss.str(coeff);

	Eigen::VectorXd newCoeff(m_coeff.rows());

	for(int i = 0; i < m_coeff.rows(); ++i)
	{
		ss >> newCoeff(i);

		if(ss.fail())
			return false;
	}

	setCoefficients(newCoeff);
	return true;
}

