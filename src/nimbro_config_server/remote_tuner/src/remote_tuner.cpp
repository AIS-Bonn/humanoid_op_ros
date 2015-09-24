// Parameter tuner tweaked for remote connections
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "remote_tuner.h"

#include "ui_remote_tuner.h"

#include <pluginlib/class_list_macros.h>

#include <ros/node_handle.h>
#include <ros/service.h>

#include <QHeaderView>
#include <QMessageBox>

#include <config_server/SetParameter.h>

Q_DECLARE_METATYPE(config_server::ParameterValueList);

namespace remote_tuner
{

RemoteTuner::RemoteTuner()
{
	m_ui = new Ui_RemoteTuner;
}

RemoteTuner::~RemoteTuner()
{
	delete m_ui;
}

void RemoteTuner::initPlugin(qt_gui_cpp::PluginContext& ctx)
{
	rqt_gui_cpp::Plugin::initPlugin(ctx);

	QWidget* w = new QWidget;
	m_ui->setupUi(w);

	m_ui->view->setModel(&m_model);
	m_model.initializeView(m_ui->view);

	qRegisterMetaType<config_server::ParameterValueList>();
	connect(
		this, SIGNAL(parameterValuesReceived(config_server::ParameterValueList)),
		&m_model, SLOT(update(config_server::ParameterValueList)),
		Qt::QueuedConnection
	);

	connect(
		&m_model, SIGNAL(setRequested(QString,QString)),
		SLOT(setParameter(QString,QString))
	);

	connect(
		m_ui->prefixEdit, SIGNAL(textChanged(QString)),
		SLOT(setPrefix(QString))
	);

	ctx.addWidget(w);
}

void RemoteTuner::shutdownPlugin()
{
	m_sub_parameterValues.shutdown();
}

void RemoteTuner::saveSettings(qt_gui_cpp::Settings& pluginSettings, qt_gui_cpp::Settings& instanceSettings) const
{
	instanceSettings.setValue("view_state", m_ui->view->header()->saveState());
	instanceSettings.setValue("prefix", m_prefix);
}

void RemoteTuner::restoreSettings(const qt_gui_cpp::Settings& pluginSettings, const qt_gui_cpp::Settings& instanceSettings)
{
	if(instanceSettings.contains("view_state"))
	{
		m_ui->view->header()->restoreState(instanceSettings.value("view_state").toByteArray());
	}

	QString prefix;
	if(instanceSettings.contains("prefix"))
		prefix = instanceSettings.value("prefix").toString();
	else
		prefix = "/remote/config_server";

	setPrefix(prefix);
	m_ui->prefixEdit->setText(m_prefix);
}

void RemoteTuner::setPrefix(const QString& prefix)
{
	m_prefix = prefix;

	ros::NodeHandle nh = getPrivateNodeHandle();
	m_sub_parameterValues = nh.subscribe(
		(m_prefix + "/parameter_values").toStdString(), 1,
		&RemoteTuner::parameterValuesReceived, this
	);
}

void RemoteTuner::setParameter(const QString& name, const QString& value)
{
	config_server::SetParameter srv;
	srv.request.name = name.toStdString();
	srv.request.value = value.toStdString();

	if(!ros::service::call((m_prefix + "/set_parameter").toStdString(), srv))
	{
		QMessageBox::critical(m_ui->prefixEdit, "Error", "Could not call set_parameter");
	}
}

}

PLUGINLIB_EXPORT_CLASS(remote_tuner::RemoteTuner, rqt_gui_cpp::Plugin)
