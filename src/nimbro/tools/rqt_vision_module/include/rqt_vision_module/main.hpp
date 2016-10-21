//This package is based on rqt_image_view package. (for copyright notes, please read the ReadMe.txt)
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#pragma once

#include <ui_main.h>
#include <rqt_gui_cpp/plugin.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>

#include <ros/macros.h>
#include <ros/time.h>

#include <std_msgs/Empty.h>
#include <rqt_vision_module/GuiEvent.h>
#include <rqt_vision_module/ConsoleMsg.h>

#include <QList>
#include <QSize>
#include <QWidget>
#include <QString>

#include <vector>

namespace rqt_vision_module
{

class ImageView: public rqt_gui_cpp::Plugin
{

Q_OBJECT

public:

	QString styledString;

	ImageView();

	virtual void initPlugin(qt_gui_cpp::PluginContext& context);

	virtual void shutdownPlugin();

	virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
			qt_gui_cpp::Settings& instance_settings) const;

	virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
			const qt_gui_cpp::Settings& instance_settings);

protected slots:

	virtual void updateTopicList();
	
	virtual void onTopicChanged(int index);

	virtual void onZoom1(bool checked);

	virtual void onDynamicRange(bool checked);

	virtual void saveImage();
	
	void onMouseEvent(MouseEvent event, float x, float y);
	
	void clearConsole();

protected:

	// deprecated function for backward compatibility only, use getTopics() instead
	ROS_DEPRECATED
	virtual QList<QString> getTopicList(const QSet<QString>& message_types,
			const QList<QString>& transports);

	virtual QSet<QString> getTopics(const QSet<QString>& message_types,
			const QSet<QString>& message_sub_types,
			const QList<QString>& transports);

	virtual void selectTopic(const QString& topic);
	
	//bool eventFilter(QObject *obj, QEvent *event);

protected:

	virtual void callbackImage(const sensor_msgs::Image::ConstPtr& msg);

	Ui::ImageViewWidget ui_;

	QWidget* widget_;

	image_transport::Subscriber subscriber_;

	cv::Mat conversion_mat_;

private:
	void handleClearConsole(const std_msgs::EmptyConstPtr &msg);
	
	// Applies color and font size
	QString prepareForConsole(QString text, int r = 255, int g = 255, int b = 255, int fontSize = 15); 
	void appendToConNewLine(QString text, QString color="white", int fontSize=15);
	
	void consoleMsgReceived(const rqt_vision_module::ConsoleMsgConstPtr &console_msg);
	
	QString rgbToHex(int r, int g, int b);

	signals:

	void conSetHtml(QString);
	void conApp(QString);
	
private:
	QString arg_topic_name;
	
	ros::Subscriber console_subscriber;
	ros::Subscriber clear_console_subscriber;
	ros::Publisher  gui_event_publisher;
	
	uint64 time_started; // Time in nanoseconds, saved at start. Plays a role of a unique ID
};

}
