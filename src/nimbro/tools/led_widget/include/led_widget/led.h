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
	virtual ~LED();

	bool turn(bool on);
	bool turned() const { return state; }

	void setBlinking(bool blink) { this->blink = blink; }
	bool isBlinking() const { return blink; }

	bool setColor(QColor color);
	void setOnColor(QColor color);

private:
	void paintEvent(QPaintEvent *event);

	int width;
	int height;

	bool state;
	bool blink;

	QColor onColor;
	QColor offColor;
	QColor currentColor;
};

#endif