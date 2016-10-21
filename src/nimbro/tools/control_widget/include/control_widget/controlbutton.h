// Wrapper for QPushButton
// Executes attached service call
// Button becomes red/green which shows if service call was successfull
// Author: Dmytro Pavlichenko dm.mark999@gmail.com

#ifndef CONTROLBUTTON_H
#define CONTROLBUTTON_H

#include <QObject>
#include <QPushButton>
#include <QTimer>

#include <ros/time.h>
#include <control_widget/service.h>

namespace control_widget
{

class ControlButton : public QObject
{
Q_OBJECT
public:
	ControlButton(QPushButton *button_, service::Service *service_);
	ControlButton(QPushButton *button_, std::string iconName, service::Service *service_);
	~ControlButton();
	
	void setStatus(bool success); // Makes button red/green for given highlightDuration
	
Q_SIGNALS:
	void feedback(QStringList&);
	
private Q_SLOTS:
	void handleClick();
	void loop(); // Main loop. Is called every 1 second. Checks if its time to turn off the highlight

private:
	void init(QPushButton *button_, service::Service *service_);
	
private:
	QPushButton *button;
	service::Service *service;
	
	QTimer *mainLoopTimer;
	ros::Time activatedTime; // Time when button was clicked
	bool higlightActive;
	
	static const int highlightDuration = 5; // How long the button is highlighted, seconds
};

}

#endif