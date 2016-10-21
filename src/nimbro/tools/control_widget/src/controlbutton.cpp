#include <control_widget/controlbutton.h>

#include <stdio.h>
#include <ros/package.h>

#include <QIcon>

namespace control_widget
{
	
ControlButton::ControlButton(QPushButton* button_, service::Service* service_) : QObject()
{
	init(button_, service_);
}

ControlButton::ControlButton(QPushButton* button_, std::string iconName, service::Service *service_) : QObject()
{
	init(button_, service_);
	
	// Set up icon
	QPixmap pixmap(QString::fromStdString(ros::package::getPath("control_widget") + "/icons/" + iconName));
	QIcon icon(pixmap);
	button->setIcon(icon);
}

void ControlButton::init(QPushButton* button_, service::Service* service_)
{
	button = button_;
	service = service_;
	
	higlightActive = false;
	
	mainLoopTimer = new QTimer();
	connect(mainLoopTimer, SIGNAL(timeout()), this, SLOT(loop()));
    mainLoopTimer->start(1000);
	
	connect(button, SIGNAL(clicked(bool)), this, SLOT(handleClick()));
}

void ControlButton::setStatus(bool success)
{
	higlightActive = true;
	
	QString successSheet("QPushButton { background: rgb(100, 230, 20); }");
	QString failureSheet("QPushButton { background: rgb(200, 60, 60); }");
	
	if(success)
		button->setStyleSheet(successSheet);
	else
		button->setStyleSheet(failureSheet);
	
	activatedTime = ros::Time::now();
}

void ControlButton::loop()
{
	if(higlightActive == false)
		return;
	
	ros::Duration elapsedTime = ros::Time::now() - activatedTime;
	
	if(elapsedTime.toSec() >= highlightDuration)
	{
		higlightActive = false;
		button->setStyleSheet("");
	}
}

void ControlButton::handleClick()
{
	QStringList output;
	setStatus(service->call(output));
	
	emit feedback(output);
}

ControlButton::~ControlButton()
{
	delete mainLoopTimer;
	delete service;
}

}
