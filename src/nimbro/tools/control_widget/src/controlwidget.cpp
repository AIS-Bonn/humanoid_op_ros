// Widget to execute service calls
// Author: Dmytro Pavlichenko dm.mark999@gmail.com

#include "ui_controlwidget.h"
#include <control_widget/controlwidget.h>

#include <QSplitter>
#include <QScrollBar>
#include <QAction>

#include <pluginlib/class_list_macros.h>

#include <ros/package.h>
#include <ros/console.h>

#include <stdlib.h>
#include <boost/thread.hpp>

namespace control_widget
{

ControlWidget::ControlWidget():
m_nh("~")
{
	QWidget *widget = new QWidget();
	
	m_ui = new Ui::ControlWidget;
	m_ui->setupUi(widget);
	
	initButtons();
	
	// Set splitter for console and buttons areas
	QSplitter *splitter = new QSplitter(widget);
	splitter->setOrientation(Qt::Vertical);
	
	splitter->addWidget(m_ui->buttonsScrollArea);
	splitter->addWidget(m_ui->consoleScrollArea);
	
	m_view = new QMainWindow();
	m_view->setWindowTitle(QString("Control Widget"));
	m_view->setCentralWidget(splitter);
	
	// Set initial sizes of buttons area and console
	QList<int> sizes;
	int consoleHeight = 115;
	int totalHeight = m_view->height();
	
	sizes.push_back(totalHeight - consoleHeight);
	sizes.push_back(consoleHeight);
	
	splitter->setSizes(sizes);
	
	// Set up context menu for console
	m_console_menu.addAction("Clear");
	
	QPixmap pixmap(QString::fromStdString(ros::package::getPath("control_widget") + "/icons/clear.png"));
	QIcon icon(pixmap);
	m_console_menu.actions().at(0)->setIcon(icon);
	
	connect(m_console_menu.actions().at(0), SIGNAL(triggered(bool)), this, SLOT(clearConsole()));
	connect(m_ui->console, SIGNAL(customContextMenuRequested(const QPoint&)), this
											, SLOT(showConsoleMenu(const QPoint&)));
}

void ControlWidget::showConsoleMenu(const QPoint& positon)
{
	QPoint globalPos = m_ui->console->mapToGlobal(positon);
    m_console_menu.exec(globalPos);
}

void ControlWidget::clearConsole()
{
	m_ui->console->clear();
	m_ui->console->setText("<b style=\"font-family:'Ubuntu'; font-size:10pt;\">Control widget log:</b>");
}

int execute_command(const std::string command)
{
	return system(command.c_str());
}

void ControlWidget::retrievePressed()
{
	// Try to get 'robot_name' parameter
	std::string robot_name;
	
	if(!m_nh.getParam("/robot_name", robot_name))
	{
		ROS_ERROR("Failed to get robot name. Cant retrieve");
		return;
	}
	
	robot_name.append(std::string(".local"));
	
	// Construct command and execute it in a thread
	std::string retrieve_command = std::string("xterm -e bash -i -c 'nimbro config retrieve " 
													+ robot_name + "; bash'");
	ROS_INFO("Will execute: %s", retrieve_command.c_str());
	
	boost::thread(execute_command, retrieve_command);
	
	//system("xterm -e bash -i -c 'nimbro gui; bash'");
	//system("gnome-terminal -x bash -i -c 'nimbro gui; bash'");
	//system("bash -i -c 'nimbro gui'");
}

void ControlWidget::initPlugin(qt_gui_cpp::PluginContext& context)
{
	context.addWidget(m_view);
	connect(m_ui->test_button, SIGNAL(clicked(bool)), this, SLOT(retrievePressed()));
}

void ControlWidget::shutdownPlugin()
{
}

void ControlWidget::receiveFeedback(QStringList& output)
{
	for(int i = 0; i < output.size(); i++)
	{
		if(output.at(i).isEmpty() == false)
		{	
			// Insert new line to the end
			m_ui->console->moveCursor(QTextCursor::End);
			
			m_ui->console->append(QString(""));
			m_ui->console->insertHtml(output.at(i));
			
			// Scroll to the bottom
			QScrollBar *scrollBar = m_ui->console->verticalScrollBar();
			scrollBar->setValue(scrollBar->maximum());
		}
	}
}

void ControlWidget::initButtons()
{
	// Magnetometer 
	m_buttons.push_back(new ControlButton(m_ui->mag_startCalib, std::string("play.png"),
				new service::Empty(std::string("/nimbro_op_interface/magFilter/startCalibration"))));
	
	m_buttons.push_back(new ControlButton(m_ui->mag_stopCalib2D, std::string("stop.png"),
				new service::MagCalib2D(std::string("/nimbro_op_interface/magFilter/stopCalibration2D"))));
	
	m_buttons.push_back(new ControlButton(m_ui->mag_stopCalib3d, std::string("stop.png"),
				new service::MagCalib3D(std::string("/nimbro_op_interface/magFilter/stopCalibration3D"))));
	
	m_buttons.push_back(new ControlButton(m_ui->mag_abortCalib, std::string("abort.png"),
				new service::Empty(std::string("/nimbro_op_interface/magFilter/abortCalibration"))));
	
	m_buttons.push_back(new ControlButton(m_ui->mag_showCalib, std::string("show.png"),
				new service::MagCalibShow(std::string("/nimbro_op_interface/magFilter/showCalibration"))));
	
	// Warp Points
	m_buttons.push_back(new ControlButton(m_ui->mag_Point0, std::string("arrowUp.png"),
				new service::WarpAddPoint(std::string("/nimbro_op_interface/magFilter/warpAddPoint"), 0)));
	
	m_buttons.push_back(new ControlButton(m_ui->mag_point45, std::string("arrowUpLeft.png"),
				new service::WarpAddPoint(std::string("/nimbro_op_interface/magFilter/warpAddPoint"), 45)));
	
	m_buttons.push_back(new ControlButton(m_ui->mag_point90, std::string("arrowLeft.png"),
				new service::WarpAddPoint(std::string("/nimbro_op_interface/magFilter/warpAddPoint"), 90)));
	
	m_buttons.push_back(new ControlButton(m_ui->mag_point135, std::string("arrowDownLeft.png"),
				new service::WarpAddPoint(std::string("/nimbro_op_interface/magFilter/warpAddPoint"), 135)));
	
	m_buttons.push_back(new ControlButton(m_ui->mag_point180, std::string("arrowDown.png"),
				new service::WarpAddPoint(std::string("/nimbro_op_interface/magFilter/warpAddPoint"), 180)));
	
	m_buttons.push_back(new ControlButton(m_ui->mag_point225, std::string("arrowDownRight.png"),
				new service::WarpAddPoint(std::string("/nimbro_op_interface/magFilter/warpAddPoint"), 225)));
	
	m_buttons.push_back(new ControlButton(m_ui->mag_point270, std::string("arrowRight.png"),
				new service::WarpAddPoint(std::string("/nimbro_op_interface/magFilter/warpAddPoint"), 270)));
	
	m_buttons.push_back(new ControlButton(m_ui->mag_point315, std::string("arrowUpRight.png"),
				new service::WarpAddPoint(std::string("/nimbro_op_interface/magFilter/warpAddPoint"), 315)));
	
	m_buttons.push_back(new ControlButton(m_ui->mag_clearPoints, std::string("delete.png"),
				new service::Empty(std::string("/nimbro_op_interface/magFilter/warpClearPoints"))));
	
	// Attitude estimator
	m_buttons.push_back(new ControlButton(m_ui->att_zeroHeading,
				new service::AttEstCalib(std::string("/nimbro_op_interface/attEstCalibrate"))));
	
	
	
	// Headcontrol
	m_buttons.push_back(new ControlButton(m_ui->head_startRecording, std::string("play.png"),
				new service::Empty(std::string("/robotcontrol/headcontrol/startRecording"))));
	
	m_buttons.push_back(new ControlButton(m_ui->head_stoprecording, std::string("stop.png"),
				new service::Empty(std::string("/robotcontrol/headcontrol/stopRecording"))));
	
	m_buttons.push_back(new ControlButton(m_ui->head_startCalib, std::string("play.png"),
				new service::Empty(std::string("/robotcontrol/headcontrol/startCalibration"))));
	
	m_buttons.push_back(new ControlButton(m_ui->head_stopCalib, std::string("stop.png"),
				new service::Empty(std::string("/robotcontrol/headcontrol/stopCalibration"))));
	
	m_buttons.push_back(new ControlButton(m_ui->head_showLimits, std::string("show.png"),
				new service::Empty(std::string("/robotcontrol/headcontrol/showHeadLimits"))));
	
	// Config server
	m_buttons.push_back(new ControlButton(m_ui->config_resetConfig, std::string("reset.png"),
				new service::ResetConfig(std::string("/config_server/load"))));
	
	m_buttons.push_back(new ControlButton(m_ui->config_saveConfig, std::string("save.png"),
				new service::SaveConfig(std::string("/config_server/save"))));
	
	m_buttons.push_back(new ControlButton(m_ui->config_showDead, std::string("list.png"),
				new service::ShowDeadVars(std::string("/config_server/show_dead_vars"), m_ui->config_showDead_Param)));
	
	// Gait
	m_buttons.push_back(new ControlButton(m_ui->gait_resetOdom, std::string("reset.png"),
				new service::Empty(std::string("/gait/resetOdom"))));
	
	m_buttons.push_back(new ControlButton(m_ui->gait_setOdom, std::string("set.png"),
				new service::SetOdom(std::string("/gait/setOdom"),
									 m_ui->gait_setOdomParamX, m_ui->gait_setOdomParamY, m_ui->gait_setOdomParamT)));
	
	// Robot interface
	m_buttons.push_back(new ControlButton(m_ui->robot_readOffset, std::string("read.png"),
				new service::ReadOffset(std::string("/nimbro_op_interface/readOffset"), m_ui->robot_readOffsetParam)));
	
	m_buttons.push_back(new ControlButton(m_ui->robot_readOffsets, std::string("readOffsets.png"),
				new service::Empty(std::string("/nimbro_op_interface/readOffsets"))));
	
	// Robotcontrol
	m_buttons.push_back(new ControlButton(m_ui->robCtrl_printJointCMDS, std::string("show.png"),
				new service::Empty(std::string("/robotcontrol/printJointCommands"))));
	
	// Set up connects
	for(unsigned i = 0; i < m_buttons.size(); i++)
		connect(m_buttons.at(i), SIGNAL(feedback(QStringList&)), this, SLOT(receiveFeedback(QStringList&)));
}


ControlWidget::~ControlWidget()
{
	for(unsigned i = 0; i < m_buttons.size(); i++)
		delete m_buttons.at(i);
}

}

PLUGINLIB_EXPORT_CLASS(control_widget::ControlWidget, rqt_gui_cpp::Plugin)
