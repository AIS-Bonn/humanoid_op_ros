//Small rqt plugin to control walking
//Author: Sebastian Schüller

#ifndef WALKCONTROL_H
#define WALKCONTROL_H

#include <rqt_gui_cpp/plugin.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <ui_walkcontrol.h>

#include <gait_msgs/GaitCommand.h>
#include <gait_msgs/GaitPerformance.h>

namespace walkcontrol
{

class WalkControl : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	WalkControl();
	virtual ~WalkControl();

	virtual void initPlugin(qt_gui_cpp::PluginContext& context);
	virtual void shutdownPlugin();

Q_SIGNALS:
	void perf_changed(const gait_msgs::GaitPerformanceConstPtr& data);

private Q_SLOTS:
	void walk();
	void reset();
	void handleWalk();
	void handleHalt();
	void setXTarget();
	void setYTarget();
	void setOmegaTarget();
	void handlePerf(const gait_msgs::GaitPerformanceConstPtr& data);

private:
	bool m_walking;

	ros::Publisher m_pub_cmd;
	gait_msgs::GaitCommand m_cmd;

	ros::Subscriber m_sub_perf;
	gait_msgs::GaitPerformance m_perf;

	Ui::walkcontrol m_ui;

};

}
#endif