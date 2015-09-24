// Parameter tuner tweaked for remote connections
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef REMOTE_TUNER_H
#define REMOTE_TUNER_H

#include <rqt_gui_cpp/plugin.h>

#include <ros/subscriber.h>

#include <QTreeView>

#include "parameter_model.h"
#include "parameter_delegate.h"

class Ui_RemoteTuner;

namespace remote_tuner
{

class RemoteTuner : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	RemoteTuner();
	virtual ~RemoteTuner();

	virtual void initPlugin(qt_gui_cpp::PluginContext& ctx) override;
	virtual void shutdownPlugin() override;

	virtual void saveSettings(qt_gui_cpp::Settings& pluginSettings, qt_gui_cpp::Settings& instanceSettings) const override;
	virtual void restoreSettings(const qt_gui_cpp::Settings& pluginSettings, const qt_gui_cpp::Settings& instanceSettings) override;
public Q_SLOTS:
	void setPrefix(const QString& prefix);
	void setParameter(const QString& name, const QString& value);
Q_SIGNALS:
	void parameterValuesReceived(const config_server::ParameterValueList& list);
private:
	ros::Subscriber m_sub_parameterValues;

	ParameterModel m_model;
	Ui_RemoteTuner* m_ui;

	QString m_prefix;
};

}

#endif
