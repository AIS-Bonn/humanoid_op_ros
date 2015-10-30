#ifndef POSVELEFFVIEW_H
#define POSVELEFFVIEW_H

// Widget to edit position, velocity and effort of certain joint
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <QObject>
#include <QWidget>
#include <QSpinBox>
#include <QSlider>
#include <QStack>
#include <QElapsedTimer>
#include <QTimer>
#include <boost/concept_check.hpp>

#include <trajectory_editor_2/historykeeper.h>

// TODO Make base class for all three views
class PosVelEffView : public QWidget
{
    Q_OBJECT
public:
	
	enum Field
	{
		POSITION,
		EFFORT,
		VELOCITY
	};
	
	enum Alignment
	{
		NO_PAIR,
		LEFT,
		RIGHT
	};
	
	PosVelEffView(PosVelEffView::Alignment alignment, std::string jointName, int id, QWidget *parent = 0);
	~PosVelEffView();
	
	virtual bool eventFilter(QObject *object, QEvent *event);
	
	void setEffort(double effort);
	void setVelocity(double velocity);
	
	void setPosition(double rate);
	double getPosition();
	
	int getID();
	
	void setOnShiftMirrored(bool mirrored); // If true, mirrors the value for pair widget when 'shift' is pressed
	void setField(PosVelEffView::Field field, float value);
	
	void clearHistoryOfChanges();
	
	
	// TODO put these to private
	std::string jointName;
	
	QSlider        *positionSlider;
	QDoubleSpinBox *positionSpin;
	QDoubleSpinBox *effortSpin;
	QDoubleSpinBox *velocitySpin;
	
Q_SIGNALS:
	void positionChanged();
	void velocityChanged();
	void effortChanged();
	
	void changeForID(int id, PosVelEffView::Field field, float value); // Set field in connected widget to be value
	
private Q_SLOTS:
	void positionSliderChanged();
	void positionSpinChanged();
	
	void handleEffortChanged();
	void handleVelocityChanged();
	
private:
	bool isShiftPressed();
	
private:
	double min;
	double max;
	
	int id;
	int connectedID; // ID of connected widget. By holding SHIFT both widgets will receive the same value
	
	bool onShiftMirrored;
	
	HistoryKeeper *effortHistory;
	HistoryKeeper *positionHistory;
	HistoryKeeper *velocityHistory;
};

#endif // POSVELEFFVIEW_H
