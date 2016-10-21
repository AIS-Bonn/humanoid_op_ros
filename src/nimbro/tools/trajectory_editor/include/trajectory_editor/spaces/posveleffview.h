#ifndef POSVELEFFVIEW_H
#define POSVELEFFVIEW_H

// Widget to edit position, velocity and effort of certain joint
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <QWidget>
#include <QSpinBox>
#include <QSlider>

#include <trajectory_editor/historykeeper.h>
#include <trajectory_editor/spaces/basicsmallview.h>

class PosVelEffView : public BasicSmallView
{
    Q_OBJECT
public:
	
	enum Field
	{
		POSITION,
		EFFORT,
		VELOCITY
	};
	
	PosVelEffView(BasicSmallView::Alignment alignment, BasicSmallView::Type type, std::string jointName, bool shiftMirrored, QWidget *parent = 0);
	~PosVelEffView();
	
	void setField(PosVelEffView::Field field, double value);
	void setEffort(double effort);
	void setVelocity(double velocity);
	void setPosition(double rate);
	
	double getPosition();
	double getEffort();
	double getVelocity();
	
	void clearHistoryOfChanges();
	
Q_SIGNALS:
	void fieldChanged(PosVelEffView::Field, std::string jointName);
	void changeForInverse(std::string inverse_joint_name, PosVelEffView::Field field, double value);
	
private Q_SLOTS:
	void positionSliderChanged();
	void positionSpinChanged();
	
	void handleEffortChanged();
	void handleVelocityChanged();
	
private:
	QSlider        *positionSlider;
	QDoubleSpinBox *positionSpin;
	QDoubleSpinBox *effortSpin;
	QDoubleSpinBox *velocitySpin;
	
	HistoryKeeper *effortHistory;
	HistoryKeeper *positionHistory;
	HistoryKeeper *velocityHistory;
};

#endif // POSVELEFFVIEW_H
