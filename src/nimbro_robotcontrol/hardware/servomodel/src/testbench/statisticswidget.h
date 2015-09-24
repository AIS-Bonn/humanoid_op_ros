// Statistics display
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef STATISTICWIDGET_H
#define STATISTICWIDGET_H

#include <QtGui/QWidget>

class Trajectory;
class ValueLogger;
class Ui_StatisticsWidget;

class StatisticsWidget : public QWidget
{
Q_OBJECT
public:
	explicit StatisticsWidget(QWidget* parent = 0);
	virtual ~StatisticsWidget();

	void setTrajectory(Trajectory* trajectory);
	void setPosLogger(ValueLogger* logger);
	void setCurrentLogger(ValueLogger* logger);
	void setTemperature(double temp);
public Q_SLOTS:
	void update();
private:
	Ui_StatisticsWidget* m_ui;
	Trajectory* m_traj;
	ValueLogger* m_logger;
	ValueLogger* m_currentLogger;
	double m_temp;
};

#endif
