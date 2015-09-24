// Dynamixel joint interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DXLJOINTINTERFACE_H
#define DXLJOINTINTERFACE_H

#include "ijointinterface.h"

class DXLJointInterface : public IJointInterface
{
public:
	DXLJointInterface(int servoID = 1);
	virtual ~DXLJointInterface();

	virtual bool init();

	virtual void setGoalPosition(double goalPosition);
	virtual void setGoalSpeed(double speed);
	virtual void setGoalTorque(double torque);
	virtual void setTorqueLimit(double torqueLimit);
	virtual double currentPosition() const;
	virtual double currentVelocity() const;
	virtual double currentCurrent() const;
	virtual double currentLoad() const;
	virtual double currentAcceleration() const;
	virtual double currentTemperature();
	virtual void process();

	virtual double maximumAngle() const;
	virtual double minimumAngle() const;

	virtual void relax();

	virtual void setControlType(ControlType ct);

	virtual void setPValue(int value);

	void setOffsetTicks(int ticks);
private:
	int m_servoID;
	int m_tickOffset;
	double m_goalPosition;
	double m_goalSpeed;
	double m_goalTorque;
	double m_currentPosition;
	double m_currentVelocity;
	double m_currentAcc;
	double m_currentLoad;
	double m_currentCurrent;
	double m_torqueLimit;
};

#endif
