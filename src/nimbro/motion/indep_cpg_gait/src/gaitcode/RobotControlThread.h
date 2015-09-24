#ifndef ROBOTCONTROLTHREAD_H
#define ROBOTCONTROLTHREAD_H

#include "Interface/SimulationInterface.h"
#include "Interface/LoopbackInterface.h"
#include "RobotControl/RobotControl.h"

#include "util/WindowsMMTimer.h"
#include "util/StopWatch.h"

namespace indep_cpg_gait
{
class RobotControlThread : public QObject
{
	Q_OBJECT

	StopWatch stopWatch; // for precise performance measuring
		WindowsMMTimer timer; // drives the rc thread
	double lastUpdateTimestamp;
	double lastStartTimestamp;

		RobotControl robotControl;

		SimulationInterface simulationInterface;
		LoopbackInterface loopbackInterface;
		Interface* iface;

	public:
	RobotControlThread(QObject *parent = 0);
	~RobotControlThread(){};

	void init();
	bool isRunning();

	public slots:
		void start();
		void stop();
		void step();
		void reset();
		void draw();
		void configChangedIn();
		void toggleInterface();

	signals:
		void messageOut(QString);
		void configChangedOut();
	};
}
#endif
