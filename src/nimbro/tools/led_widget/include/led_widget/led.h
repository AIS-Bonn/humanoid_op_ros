// Widget which visualizes LED
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef LED_H
#define LED_H

#include <QWidget>
#include <QObject>

#include <QPaintEvent>

class LED : public QWidget
{
Q_OBJECT
public:
	LED(int width_, int height_, QWidget *parent = 0);
	~LED();
	
	void turn(bool on);
	void setColor(QColor color);
	
	void setOnColor(QColor color);
	
private:
	void paintEvent(QPaintEvent *event);
	
	int width;
	int height;
	
	QColor onColor;
	QColor offColor;
	QColor currentColor;
};

#endif