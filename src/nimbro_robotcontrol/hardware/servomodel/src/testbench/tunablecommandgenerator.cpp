// Servo command generator with Qt property interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "tunablecommandgenerator.h"

TunableCommandGenerator::TunableCommandGenerator(QObject* parent)
 : ParametrizedObject(parent)
{
	addParameter("stribeckOne", 0, 0.01, 1.0, "Stribeck I");
	addParameter("stribeckTwo", 0, 0.01, 1.0, "Stribeck II");
	addParameter("viscousFriction", -2, 0.01, 2, "Viscous Friction");
	addParameter("km", 0, 0.000001, 1.0, "Motor KM");
	addParameter("latency", 0, 0.001, 0.1, "Latency");
	addParameter("voltage", 10.0, 0.1, 17.0, "Voltage");
}

TunableCommandGenerator::~TunableCommandGenerator()
{
}

void TunableCommandGenerator::update()
{
	ServoCommandGenerator::update();
	changed();
}

void TunableCommandGenerator::setPValue(int p)
{
	ServoCommandGenerator::setPValue(p);
}
