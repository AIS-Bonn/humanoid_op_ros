// Small rqt plugin to show the robot buttons and LEDs
// Author: Dmytro Pavlichenko

#ifndef LED_WIDGET_H
#define LED_WIDGET_H

#include <led_widget/led.h>
#include <rqt_gui_cpp/plugin.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <nimbro_op_interface/LEDCommand.h>
#include <config_server/parameter.h>
#include <QTimer>

namespace Ui { class LEDWidget; }

namespace led_widget
{

class LEDWidget : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	LEDWidget();
	virtual ~LEDWidget();

	virtual void initPlugin(qt_gui_cpp::PluginContext& context);
	virtual void shutdownPlugin();

private Q_SLOTS:
	void handleBlinkTimerTimeout();

	void handleButton_0();
	void handleButton_1();
	void handleButton_2();

private:
	void stateReceived(const nimbro_op_interface::LEDCommandConstPtr& state);
	void updateLED(int led, bool state);
	void updateBlinking();

private:
	QWidget *view;
	Ui::LEDWidget *ui;

	std::vector<LED *> leds;

	ros::NodeHandle n;
	ros::Subscriber subscriber;

	QTimer m_blinkTimer;
	bool m_blinkState;

	config_server::Parameter<bool> m_pressButton0;
	config_server::Parameter<bool> m_pressButton1;
	config_server::Parameter<bool> m_pressButton2;
};

}

#endif