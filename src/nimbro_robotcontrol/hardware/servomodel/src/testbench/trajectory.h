// Trajectory
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <QtCore/QObject>
#include "dynamicparameter.h"

class Trajectory : public ParametrizedObject
{
Q_OBJECT
public:
	virtual double position(double time) const = 0;
	virtual double velocity(double time) const = 0;
	virtual double acceleration(double time) const = 0;

	virtual double endTime() const = 0;
Q_SIGNALS:
	void changed();
private:
	QList<DynamicParameter*> m_parameters;
};

#endif
