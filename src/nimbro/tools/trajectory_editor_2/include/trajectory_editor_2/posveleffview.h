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

class PosVelEffView : public QWidget
{
    Q_OBJECT
public:
	
	enum Alignment
	{
		LEFT,
		RIGHT
	};
	
	PosVelEffView(PosVelEffView::Alignment alignment, std::string jointName, QWidget *parent = 0);
	~PosVelEffView();
	
	virtual bool eventFilter(QObject *object, QEvent *event);
	
	void setEffort(double effort);
	void setVelocity(double velocity);
	
	void setPosition(double rate);
	double getPosition();
	
	void clearHistoryOfChanges();
	
	QSlider        *positionSlider;
	QDoubleSpinBox *positionSpin;
	QDoubleSpinBox *effortSpin;
	QDoubleSpinBox *velocitySpin;
	
	std::string jointName;
	
Q_SIGNALS:
	void positionChanged();
	void velocityChanged();
	void effortChanged();
	
private Q_SLOTS:
	void positionSliderChanged();
	void positionSpinChanged();
	
	void handleEffortChanged();
	void handleVelocityChanged();
	
private:
	double min;
	double max;
	
	HistoryKeeper *effortHistory;
	HistoryKeeper *positionHistory;
	HistoryKeeper *velocityHistory;
};

#endif // POSVELEFFVIEW_H
