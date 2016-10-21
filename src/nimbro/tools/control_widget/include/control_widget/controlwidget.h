// Widget to execute service calls
// Author: Dmytro Pavlichenko dm.mark999@gmail.com

#ifndef CONTROLWIDGET_H
#define CONTROLWIDGET_H

#include <control_widget/controlbutton.h>

#include <rqt_gui_cpp/plugin.h>
#include <QMainWindow>
#include <QMenu>

#include <ros/node_handle.h>

#include <vector>

namespace Ui { class ControlWidget; }

namespace control_widget
{

class ControlWidget : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	ControlWidget();
	virtual ~ControlWidget();

	virtual void initPlugin(qt_gui_cpp::PluginContext& context);
	virtual void shutdownPlugin();
	
private Q_SLOTS:
	void receiveFeedback(QStringList& output);
	void showConsoleMenu(const QPoint&);
	void clearConsole();
	
	void retrievePressed();
	
private:
	void initButtons();

private:
	QMainWindow *m_view;
	Ui::ControlWidget *m_ui;
	QMenu m_console_menu;
	
	std::vector<ControlButton*>m_buttons;
	
	ros::NodeHandle m_nh;
};

}

#endif