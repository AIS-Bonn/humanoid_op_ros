#ifndef BASICSMALLVIEW_H
#define BASICSMALLVIEW_H

// Base class for PosVelEffView, RateAngleView, PositionView
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <QWidget>
#include <vector>

class BasicSmallView : public QWidget
{
	Q_OBJECT
	
public:
	
	enum Type // Defines intervals of possible values for spins/sliders
	{
		REGULAR,   // [-PI, +PI]
		EXTENSION, // [0, 1]
		LEG,       // [-PI/2, +PI/2]
		MINUS_ONE_PLUS_ONE
	};

	enum Alignment // Order of widgets
	{
		NO_PAIR,
		LEFT,
		RIGHT
	};
	
	BasicSmallView(Alignment alignment, Type type, std::string jointName, bool shiftMirrored, QWidget *parent = 0);
	~BasicSmallView();
	
	std::string getJointName();
	void setEnabled(bool enabled);
	bool withinRange(double value);
	
	virtual bool eventFilter(QObject *object, QEvent *event);
	
protected:
	bool isShiftPressed();
	void setUpLayout(std::vector<QWidget*> widgets, Alignment alignment);
	std::string determineInverseJoint(std::string joint);
	
protected:
	std::string jointName;
	std::string m_inverse_joint_name; // If jointName is "left_***", inverse name is "right_***" and wisa wersa
	bool onShiftMirrored; // If true, mirrors the value for pair widget when 'shift' is pressed
	
	double min;
	double max;
	
private:
	std::vector<QWidget*> widgets;
};

#endif