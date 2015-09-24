// Diagnostics GUI
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "diagnostics/diagnostics.h"
#include <robotcontrol/FadeTorqueAction.h>

#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

#include <QLabel>
#include <QDockWidget>

namespace diagnostics
{

Diagnostics::Diagnostics()
 : m_fadeStatus(false)
 , m_fadeDone(true)
{
}

Diagnostics::~Diagnostics()
{
}

void Diagnostics::initPlugin(qt_gui_cpp::PluginContext& context)
{
	QWidget* w = new QWidget();
	m_ui.setupUi(w);

	m_torqueClient.reset(
		new actionlib::SimpleActionClient<robotcontrol::FadeTorqueAction>(getNodeHandle(), "/robotcontrol/fade_torque")
	);

	updateFadeButton();
	connect(m_ui.fadeButton, SIGNAL(clicked(bool)), SLOT(handleFade()));

	m_sub_diag = getNodeHandle().subscribe("/robotcontrol/diagnostics", 1, &Diagnostics::handleDiagnostics, this);

	connect(this, SIGNAL(updateRequested()), this, SLOT(update()), Qt::QueuedConnection);

	context.addWidget(w);

	// Hack our title bar widget to make our widget slimmer
	QDockWidget* dock = qobject_cast<QDockWidget*>(w->parent());
	if(dock)
	{
		QWidget* titleBarWidget = dock->titleBarWidget();
		foreach(QObject* child, titleBarWidget->children())
		{
			QPushButton* btn = qobject_cast<QPushButton*>(child);

			// Nobody needs those buttons anyway.
			if(btn)
				btn->hide();
		}
	}
}

void Diagnostics::handleDiagnostics(const robotcontrol::DiagnosticsPtr& diag)
{
	QMutexLocker locker(&m_mutex);
	m_diag = diag;
	updateRequested();
}

void Diagnostics::update()
{
	QMutexLocker locker(&m_mutex);
	m_ui.battBar->setValue(m_diag->batteryVoltage * 10);
	m_ui.battLabel->setText(QString("%1V").arg(m_diag->batteryVoltage, 0, 'f', 1));

	m_ui.tempBar->setValue(m_diag->servoTemperature * 10);
	m_ui.tempLabel->setText(QString("%1°C").arg(m_diag->servoTemperature, 0, 'f', 1));

	m_ui.stateLabel->setText(QString::fromStdString(m_diag->state));

	updateFadeButton();
}

void Diagnostics::handleFade()
{
	robotcontrol::FadeTorqueGoal goal;
	if(m_fadeStatus)
		goal.torque = 0.0;
	else
		goal.torque = 1.0;

	m_fadeDone = false;
	m_ui.fadeButton->setEnabled(false);
	m_torqueClient->sendGoal(goal,
		boost::bind(&Diagnostics::handleFadeDone, this, _1, _2),
		actionlib::SimpleActionClient<robotcontrol::FadeTorqueAction>::SimpleActiveCallback(),
		boost::bind(&Diagnostics::handleFadeFeedback, this, _1)
	);
}

void Diagnostics::updateFadeButton()
{
	if(!m_fadeDone && m_fadeFB)
	{
		m_ui.fadeButton->setText(QString("%1%").arg(
			(int)(m_fadeFB->current_torque * 100)
		));
	}
	else
	{
		m_ui.fadeButton->setEnabled(true);
		if(m_fadeStatus)
			m_ui.fadeButton->setText("Fade out");
		else
			m_ui.fadeButton->setText("Fade in");
	}
}

void Diagnostics::handleFadeDone(const actionlib::SimpleClientGoalState&, const robotcontrol::FadeTorqueResultConstPtr& ptr)
{
	m_fadeStatus = !m_fadeStatus;
	m_fadeFB.reset();
	m_fadeDone = true;
	qDebug("Fade done!\n");

	updateRequested();
}

void Diagnostics::handleFadeFeedback(const robotcontrol::FadeTorqueFeedbackConstPtr& fb)
{
	m_fadeFB = fb;
	updateRequested();
}

}

PLUGINLIB_EXPORT_CLASS(diagnostics::Diagnostics, rqt_gui_cpp::Plugin)
