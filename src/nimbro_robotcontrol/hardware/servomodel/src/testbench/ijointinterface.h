// Joint interface base class
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef IJOINTINTERFACE_H
#define IJOINTINTERFACE_H

#include <QtCore/QObject>

class IJointInterface : public QObject
{
Q_OBJECT
public:
	enum ControlType
	{
		CT_POSITION,
		CT_VELOCITY,
		CT_TORQUE,

		CT_NUM_TYPES
	};
	static const char* CONTROL_TYPE_NAMES[];

	IJointInterface(int id);
	virtual ~IJointInterface();

	virtual bool init() = 0;

	virtual void setGoalPosition(double goalPosition) = 0;
	virtual void setGoalSpeed(double speed) = 0;
	virtual void setGoalTorque(double torque) = 0;
	virtual void setTorqueLimit(double torqueLimit) = 0;

	virtual double currentPosition() const = 0;
	virtual double currentVelocity() const = 0;
	virtual double currentAcceleration() const = 0;
	virtual double currentLoad() const = 0;
	virtual double currentCurrent() const = 0;
	virtual double currentTemperature();

	virtual double maximumAngle() const = 0;
	virtual double minimumAngle() const = 0;

	virtual void relax() = 0;

	virtual void process() = 0;

	static IJointInterface* create();

	virtual void setControlType(ControlType ct)
	{ m_controlType = ct; }
	inline ControlType controlType() const
	{ return m_controlType; }

	inline int id() const
	{ return m_id; }
public Q_SLOTS:
	virtual void setPValue(int value);
private:
	ControlType m_controlType;
	int m_id;
};

#endif
