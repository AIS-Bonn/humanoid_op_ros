// Calibration Image widget (handles rect selection)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "calibimagewidget.h"

#include <QtGui/QPainter>
#include <QtGui/QMouseEvent>

CalibImageWidget::CalibImageWidget(QWidget* parent)
 : ImageWidget(parent)
{
}

CalibImageWidget::~CalibImageWidget()
{
}

void CalibImageWidget::paintEvent(QPaintEvent* event)
{
	ImageWidget::paintEvent(event);

	if(m_selection.isNull())
		return;

	QPainter painter(this);
	painter.scale(zoom(), zoom());
	painter.drawRect(m_selection);
}

void CalibImageWidget::mousePressEvent(QMouseEvent* event)
{
	m_selection.setTopLeft((1.0 / zoom()) * event->pos());
	m_selection.setBottomRight((1.0 / zoom()) * event->pos());
}

void CalibImageWidget::mouseReleaseEvent(QMouseEvent* event)
{
	m_selection.setBottomRight((1.0 / zoom()) * event->pos());
	emit selectionChanged();
}

void CalibImageWidget::mouseMoveEvent(QMouseEvent* event)
{
	m_selection.setBottomRight((1.0 / zoom()) * event->pos());
	update();
}

