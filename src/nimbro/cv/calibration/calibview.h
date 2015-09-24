// Customized QGraphicsView for YUV widget
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CALIBVIEW_H
#define CALIBVIEW_H

#include <QtGui/QGraphicsView>

class CalibView : public QGraphicsView
{
Q_OBJECT
public:
	CalibView(QWidget* parent = 0);

	virtual void wheelEvent(QWheelEvent* event);
signals:
	void zoomed();
};

#endif
