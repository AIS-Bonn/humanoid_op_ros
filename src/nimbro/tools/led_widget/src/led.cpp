#include <led_widget/led.h>

#include <QPainter>

LED::LED(int width_, int height_, QWidget *parent) : QWidget(parent)
{
	width  = width_;
	height = height_;
	
	this->setFixedSize(width, height);
	
	onColor = QColor(255, 0, 0);
	offColor = QColor(50, 50, 50);
	
	currentColor = offColor;
	state = false;
	blink = false;
}

LED::~LED()
{
}

bool LED::turn(bool on)
{
	QColor old = currentColor;
	state = on;
	if(on)
		currentColor = onColor;
	else
		currentColor = offColor;
	return (currentColor != old);
}

bool LED::setColor(QColor color)
{
	QColor old = currentColor;
	currentColor = color;
	return (currentColor != old);
}

void LED::setOnColor(QColor color)
{
	onColor = color;
}

void LED::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
    QRectF rectangle(0.0, 0.0, width, height);
	
	QBrush brush;
	brush.setColor(currentColor);
	brush.setStyle(Qt::SolidPattern);
	
	painter.setBrush(brush);
    painter.drawRect(rectangle);
}