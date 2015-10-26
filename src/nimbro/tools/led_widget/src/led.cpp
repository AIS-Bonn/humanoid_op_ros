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
}

LED::~LED()
{
	
}

void LED::turn(bool on)
{
	if(on)
		currentColor = onColor;
	else
		currentColor = offColor;
}

void LED::setColor(QColor color)
{
	currentColor = color;
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