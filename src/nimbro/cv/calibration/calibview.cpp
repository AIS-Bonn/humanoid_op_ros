// Customized QGraphicsView for YUV widget
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "calibview.h"

#include <QtGui/QMouseEvent>

CalibView::CalibView(QWidget* parent)
 : QGraphicsView(parent)
{
	setTransformationAnchor(AnchorUnderMouse);
}

void CalibView::wheelEvent(QWheelEvent* event)
{
	float delta = -(1.0 / 140.0) * event->delta();
	if(delta < 0)
		delta = -1.0 / delta;

	scale(delta, delta);
	emit zoomed();
}
