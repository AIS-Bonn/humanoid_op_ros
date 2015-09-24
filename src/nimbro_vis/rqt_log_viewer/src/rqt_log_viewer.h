// RQT plugin
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef RQT_LOG_VIEWER_H
#define RQT_LOG_VIEWER_H

#include <rqt_gui_cpp/plugin.h>

#include <rosgraph_msgs/Log.h>

#include <ros/subscriber.h>

#include "log_model.h"
#include "node_model.h"
#include "log_filter.h"

#include "editors/editor.h"

class Ui_RQTLogViewer;

namespace rqt_log_viewer
{

class RQTLogViewer : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	RQTLogViewer();
	virtual ~RQTLogViewer();

	virtual void initPlugin(qt_gui_cpp::PluginContext&);
	virtual void shutdownPlugin();

	virtual void saveSettings(
		qt_gui_cpp::Settings& pluginSettings,
		qt_gui_cpp::Settings& instanceSettings
	) const;

	virtual void restoreSettings(
		const qt_gui_cpp::Settings& pluginSettings,
		const qt_gui_cpp::Settings& instanceSettings
	);
Q_SIGNALS:
	void messageReceived(const rosgraph_msgs::Log& msg);
private Q_SLOTS:
	void toggleConfig();
	void handleMessageSelection(const QModelIndex& index);
	void handleMessageClosing();
	void displayContextMenu(const QPoint& point);
private:
	ros::Subscriber m_sub_log;
	Ui_RQTLogViewer* m_ui;

	NodeModel* m_nodeModel;
	LogModel* m_model;
	LogFilter* m_filter;

	QList<editors::Editor*> m_editors;
};

}

#endif
