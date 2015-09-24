// Testbench main window
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TESTBENCH_H
#define TESTBENCH_H

#include <QtGui/QWidget>
#include <boost/shared_ptr.hpp>

#include <servomodel/torqueestimator.h>
#include "StopWatch.h"

class CompositeTrajectory;
class ServoCommandGenerator;
class SingleServoTestbench;

namespace Ui { class Testbench; }

class Testbench : public QWidget
{
Q_OBJECT
public:
	Testbench();
	virtual ~Testbench();

	bool init();

private Q_SLOTS:
	void generateCommands();
	void start();
	void stop();
	void doStep();

protected:
	virtual void closeEvent(QCloseEvent* );

private:
	QList<SingleServoTestbench*> m_benches;
	QStringList m_jointNames;
	boost::shared_ptr<TorqueEstimator> m_estimator;
	Ui::Testbench* m_ui;
	StopWatch m_stopWatch;
	QTimer* m_timer;
	QTimer* m_repTimer;
	int m_timeStamp;
	CompositeTrajectory* m_traj;
	KDL::Tree m_tree;


	int trajectoryTicks();
};

#endif
