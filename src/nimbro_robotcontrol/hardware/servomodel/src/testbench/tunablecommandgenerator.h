// Servo command generator with Qt property interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TUNABLECOMMANDGENERATOR_H
#define TUNABLECOMMANDGENERATOR_H

#include <servomodel/servocommandgenerator.h>
#include "dynamicparameter.h"

class TunableCommandGenerator : public ParametrizedObject, public ServoCommandGenerator
{
Q_OBJECT
Q_PROPERTY(double stribeckOne WRITE setStribeckOne READ stribeckOne)
Q_PROPERTY(double stribeckTwo WRITE setStribeckTwo READ stribeckTwo)
Q_PROPERTY(double viscousFriction WRITE setViscousFriction READ viscousFriction)
Q_PROPERTY(double km WRITE setKM READ km)
Q_PROPERTY(double latency WRITE setLatency READ latency)
Q_PROPERTY(double voltage WRITE setVoltage READ voltage)
public:
	explicit TunableCommandGenerator(QObject* parent);
	virtual ~TunableCommandGenerator();

protected:
	virtual void update();
public Q_SLOTS:
	void setPValue(int p);
};

#endif
