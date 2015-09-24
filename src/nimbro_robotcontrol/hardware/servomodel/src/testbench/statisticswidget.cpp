// Statistics display
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "statisticswidget.h"

#include "ui_statisticswidget.h"
#include "valuelogger.h"
#include "trajectory.h"

#include <math.h>

StatisticsWidget::StatisticsWidget(QWidget* parent)
 : QWidget(parent)
{
	m_ui = new Ui_StatisticsWidget();
	m_ui->setupUi(this);
}

StatisticsWidget::~StatisticsWidget()
{
	delete m_ui;
}

void StatisticsWidget::setPosLogger(ValueLogger* logger)
{
	m_logger = logger;
}

void StatisticsWidget::setTrajectory(Trajectory* trajectory)
{
	m_traj = trajectory;
}

void StatisticsWidget::setCurrentLogger(ValueLogger* logger)
{
	m_currentLogger = logger;
}

void StatisticsWidget::setTemperature(double temp)
{
	m_temp = temp;
}

void StatisticsWidget::update()
{
	double maxdev = 0;
	const double DEV_WINDOW = 0.05;

	for(ValueLogger::const_iterator it = m_logger->begin(); it.isValid(); ++it)
	{
		if(it.time() < 1.0)
			continue;

		double pos = it.value();
		double dev = -1;

		for(int i = 0; i < 100; ++i)
		{
			double t = it.time() - DEV_WINDOW + DEV_WINDOW*i/50.0;
			double d = fabs(m_traj->position(t) - pos);
			if(dev < 0 || d < dev)
				dev = d;
		}

		maxdev = qMax(maxdev, dev);
	}

	m_ui->deviationLabel->setText(QString("%1 rad  /  %2 deg").arg(maxdev, 0, 'f', 4).arg(maxdev * 180.0/M_PI, 0, 'f', 2));

	double energy = 0;
	double lastTime = 0;
	for(ValueLogger::const_iterator it = m_currentLogger->begin(); it.isValid(); ++it)
	{
		if(it.time() < 0.5)
			continue;

		double current = it.value();
		double deltaT = it.time() - lastTime;
		energy += fabs(current) * 12.0 * deltaT;

		lastTime = it.time();
	}

	m_ui->energyLabel->setText(QString("%1 Ws").arg(energy, 0, 'f', 2));

	m_ui->temperatureLabel->setText(QString::fromUtf8("%1 °C").arg(m_temp, 0, 'f', 1));
}
