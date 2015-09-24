//Small rqt plugin to control walking
//Author: Sebastian Schüller

#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

#include "walkcontrol.h"
#include <qshortcut.h>

namespace walkcontrol
{

WalkControl::WalkControl()
 : m_walking(false)
{}

WalkControl::~WalkControl()
{}

void WalkControl::initPlugin(qt_gui_cpp::PluginContext& context)
{
	QWidget* w = new QWidget();
	m_ui.setupUi(w);

	connect(m_ui.walk_button, SIGNAL(clicked(bool)), this, SLOT(walk()));
	connect(m_ui.reset_button, SIGNAL(clicked(bool)), this, SLOT(reset()));

	connect(m_ui.x_spinbox, SIGNAL(editingFinished()), this, SLOT(setXTarget()));
	connect(m_ui.y_spinbox, SIGNAL(editingFinished()), this, SLOT(setYTarget()));
	connect(m_ui.o_spinbox, SIGNAL(editingFinished()), this, SLOT(setOmegaTarget()));

	m_pub_cmd = getNodeHandle().advertise<gait_msgs::GaitCommand>("/gaitCommand", 1, true);

	m_sub_perf = getNodeHandle().subscribe("/gait/performance/", 10, &WalkControl::perf_changed, this);
	qRegisterMetaType<gait_msgs::GaitPerformanceConstPtr>("gait_msgs::GaitPerformanceConstPtr");

	connect(this, SIGNAL(perf_changed(gait_msgs::GaitPerformanceConstPtr)), this, SLOT(handlePerf(gait_msgs::GaitPerformanceConstPtr)), Qt::QueuedConnection);

	m_ui.x_spinbox->setSingleStep(0.1);
	m_ui.y_spinbox->setSingleStep(0.1);
	m_ui.o_spinbox->setSingleStep(0.1);

	context.addWidget(w);

	QWidget* mainWindow = qobject_cast< QWidget* >(w->parent());
	if (mainWindow)
	{
		QShortcut* walkSc = new QShortcut(QKeySequence("Ctrl+w"), mainWindow);
		QShortcut* haltSc = new QShortcut(QKeySequence("Ctrl+h"), mainWindow);
		QShortcut* resetSc = new QShortcut(QKeySequence("Ctrl+r"), mainWindow);
		connect(walkSc, SIGNAL(activated()), this, SLOT(handleWalk()));
		connect(haltSc, SIGNAL(activated()), this, SLOT(handleHalt()));
		connect(resetSc, SIGNAL(activated()), this, SLOT(reset()));
	}

}

void WalkControl::shutdownPlugin()
{
	rqt_gui_cpp::Plugin::shutdownPlugin();

	m_pub_cmd.shutdown();
	m_sub_perf.shutdown();
}


void WalkControl::handleWalk()
{
	m_cmd.walk = true;
	m_pub_cmd.publish(m_cmd);

	m_ui.walk_button->setText("Halt");

	m_walking = true;
}

void WalkControl::handleHalt()
{
	m_cmd.walk = false;
	m_pub_cmd.publish(m_cmd);

	m_ui.walk_button->setText("Walk");

	m_walking = false;
}


void WalkControl::walk()
{
	if (m_walking)
		handleHalt();
	else
		handleWalk();
}

void WalkControl::reset()
{
	m_cmd.gcvX = 0;
	m_cmd.gcvY = 0;
	m_cmd.gcvZ = 0;
	m_pub_cmd.publish(m_cmd);

	m_ui.x_spinbox->setValue(0);
	m_ui.y_spinbox->setValue(0);
	m_ui.o_spinbox->setValue(0);
}

void WalkControl::setXTarget()
{
	m_cmd.gcvX = m_ui.x_spinbox->value();
	m_pub_cmd.publish(m_cmd);
}

void WalkControl::setYTarget()
{
	m_cmd.gcvY = m_ui.y_spinbox->value();
	m_pub_cmd.publish(m_cmd);
}

void WalkControl::setOmegaTarget()
{
	m_cmd.gcvZ = m_ui.o_spinbox->value();
	m_pub_cmd.publish(m_cmd);
}

void WalkControl::handlePerf(const gait_msgs::GaitPerformanceConstPtr& data)
{
	m_ui.torque_label->setText(QString::number(data->torque));
	m_ui.angle_label->setText(QString::number(data->maxAngle));
}


}

PLUGINLIB_EXPORT_CLASS(walkcontrol::WalkControl, rqt_gui_cpp::Plugin)
