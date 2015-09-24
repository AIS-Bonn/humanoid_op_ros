// Y histogram display widget
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "histwidget.h"

#include <QtGui/QPainter>
#include <QtGui/QMouseEvent>

#include "calibellipse.h"
#include "calibscene.h"
#include "histogram.h"

HistWidget::HistWidget(CalibScene* scene, Histogram* hist, QWidget* parent)
 : QWidget(parent)
 , m_scene(scene)
 , m_histogram(hist)
 , m_grabState(GR_None)
 , m_colorID(-1)
{
	setMinimumHeight(80);
	setMouseTracking(true);
}

HistWidget::~HistWidget()
{
}

void HistWidget::paintEvent(QPaintEvent*)
{
	QPainter painter(this);

	if(!m_scene || !m_histogram)
		return;

	int w = width();
	int h = height();

	// Y ruler black to white
	if(m_y_ruler.width() != w)
	{
		QImage img(w, 10, QImage::Format_ARGB32);
		for(int x = 0; x < w; ++x)
		{
			int gray = x*256/width();
			QRgb color = qRgba(gray, gray, gray, 0xFF);
			for(int y = 0; y < 10; ++y)
				img.setPixel(x,y, color);
		}
		m_y_ruler = QPixmap::fromImage(img);
	}
	painter.drawPixmap(0, h-10, m_y_ruler);
	h -= 15;

	// histogram
	QList<QGraphicsItem*> selected = m_scene->selectedItems();
	CalibEllipse* e = 0;
	if(!selected.empty())
	{
		e = (CalibEllipse*)selected[0];
		m_colorID = e->id();
	}
	else
		m_colorID = -1;

	QPen inclPen;
	inclPen.setWidth(1);
	inclPen.setColor(Qt::green);

	QPen exclPen(inclPen);
	exclPen.setColor(Qt::red);

	// Neutral pen if no ellipse is selected
	painter.setPen(QPen(Qt::black, 1));

	for(int x = 0; x < w; ++x)
	{
		int Y = (x*256) / w;

		int bar_height = 0;
		if(m_histogram->maximum())
			bar_height = ((*m_histogram)[Y]*h) / m_histogram->maximum();

		if(e)
		{
			if(Y >= e->minY() && Y <= e->maxY())
				painter.setPen(inclPen);
			else
				painter.setPen(exclPen);
		}

		painter.drawLine(
			x, h,
			x, h - bar_height
		);
	}

	// Y minimum/maximum slider
	if(e)
	{
		painter.setPen(Qt::blue);

		m_x_minY = e->minY() * w / 256;
		painter.drawLine(m_x_minY, h, m_x_minY, 0);

		m_x_maxY = (e->maxY()+1) * w / 256;
		if(m_x_maxY >= width())
			m_x_maxY -= 1;
		painter.drawLine(m_x_maxY, h, m_x_maxY, 0);
	}
}

void HistWidget::mouseMoveEvent(QMouseEvent* event)
{
	switch(m_grabState)
	{
		case GR_None:
			if(abs(event->x() - m_x_minY) < 3 || abs(event->x() - m_x_maxY) < 3)
			{
				setCursor(Qt::SizeHorCursor);
			}
			else
				setCursor(Qt::ArrowCursor);
			break;
		case GR_Min:
		{
			int minY = event->x() * 256 / width();
			QList<CalibEllipse*> ellipses = m_scene->ellipsesWithID(m_colorID);
			foreach(CalibEllipse* e, ellipses)
				e->setMinY(minY);

			update();
		}
			break;
		case GR_Max:
		{
			int maxY = event->x() * 256 / width() - 1;
			QList<CalibEllipse*> ellipses = m_scene->ellipsesWithID(m_colorID);
			foreach(CalibEllipse* e, ellipses)
				e->setMaxY(maxY);

			update();
		}
			break;
	}
}

void HistWidget::mousePressEvent(QMouseEvent* event)
{
	int dist_min = abs(event->x() - m_x_minY);
	int dist_max = abs(event->x() - m_x_maxY);

	if(dist_min < 4 && dist_min < dist_max)
		m_grabState = GR_Min;
	else if(dist_max < 4)
		m_grabState = GR_Max;
}

void HistWidget::mouseReleaseEvent(QMouseEvent* event)
{
	m_grabState = GR_None;
	emit selectionChanged();
}



