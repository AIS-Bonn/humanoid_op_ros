// Plot widget
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "plotter/plotwidget.h"
#include "plotter/plot.h"

#include <QtGui/QPainter>
#include <QtGui/QMouseEvent>
#include <QtGui/QShortcut>

#include <math.h>
#include <boost/concept_check.hpp>

namespace plotter
{

PlotWidget::PlotWidget(QWidget* parent)
 : QWidget(parent)
 , m_cursorStep("/plotter/cursorStep", 0.001, 0.005, 0.5, 0.010)
 , m_wheelScaleFactor("/plotter/wheelScaleFactor", 1.0, 0.01, 2.0, 1.2)
 , m_vertScaleFactor("/plotter/vertScaleFactor", 0.5, 0.1, 10.0, 4.0)
 , m_playbackSpeedExp("/plotter/playbackSpeedExp", -5.0, 0.05, 2.0, 0.0)
{
	m_swipeFadeOutTimer.setInterval(20);
	connect(&m_swipeFadeOutTimer, SIGNAL(timeout()), this, SLOT(swipeFadeOut()));

	m_playingTimer.setInterval(20);
	connect(&m_playingTimer, SIGNAL(timeout()), this, SLOT(updatePlaying()));

	m_mouseClickElapsed.start();
	m_swipeFadeOutElapsed.start();

	setMouseTracking(true);
	m_screenScale_x = m_screenScale_y = m_screenScale_y_orig = 0.01; // 1 px is worth 0.01 value
	m_screenOffset = QPointF(2,1);
	m_mousePresent = false;

	setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

	m_font_default = QFont("Arial", 8);
	m_font_monospace = QFont("monospace", 8);

	m_pressHadShift = false;
	m_showDots = false;
	m_playing = false;

	m_selectionStart = m_selectionEnd = ros::Time(0);
	rangeChanged();

	new QShortcut(Qt::Key_D, this, SLOT(toggleDots()));
	new QShortcut(Qt::Key_S, this, SLOT(advanceFixedTime()));
	new QShortcut(Qt::Key_A, this, SLOT(retreatFixedTime()));
}

PlotWidget::~PlotWidget()
{
}

void PlotWidget::addPlot(Plot* plot)
{
	m_plots << plot;

	connect(plot, SIGNAL(changed(Plot*)), SLOT(update()));
}

void PlotWidget::refresh()
{
	// Re-register config variables (in case config server has been restarted
	m_cursorStep.reinit();
	m_wheelScaleFactor.reinit();
	m_vertScaleFactor.reinit();
	m_playbackSpeedExp.reinit();
	
	// Update all kinds of stuff
	updateScreenTransform();
	emit timeChanged();
	rangeChanged();
	unsetCursor();
	update();
}

void PlotWidget::setPlaying(bool play)
{
	if(play == m_playing) return;
	m_playing = play;
	if(m_playing)
	{
		m_playingTimer.start();
		m_playingTimerElapsed.start();
		if(!m_fixedTime.isZero())
		{
			if((!m_selectionStart.isZero() && m_fixedTime <= m_selectionStart) || (!m_selectionEnd.isZero() && m_fixedTime >= m_selectionEnd))
				m_fixedTime = m_selectionStart;
		}
		else if(!m_selectionStart.isZero())
			m_fixedTime = m_selectionStart;
		else
		{
			if(!m_currentTime.isZero())
				m_fixedTime = m_currentTime + ros::Duration(m_screenTransform.inverted().map(QPointF(0, 0)).x());
			else
				emit stopPlaying();
		}
		rangeChanged();
		emit timeChanged();
	}
	else
	{
		m_playingTimer.stop();
	}
}

void PlotWidget::updatePlaying()
{
	if(m_fixedTime.isZero())
	{
		rangeChanged();
		return;
	}
	m_fixedTime += ros::Duration(exp(m_playbackSpeedExp()) * 0.001 * m_playingTimerElapsed.elapsed());
	m_playingTimerElapsed.restart();
	if(m_fixedTime > m_currentTime || (!m_selectionEnd.isZero() && m_fixedTime > m_selectionEnd) || (!m_selectionStart.isZero() && m_fixedTime < m_selectionStart))
	{
		if(!m_selectionStart.isZero())
			m_fixedTime = m_selectionStart;
		else
			emit stopPlaying();
	}
	emit timeChanged();
	update();
}

void PlotWidget::mousePressEvent(QMouseEvent *event)
{
	m_mouseClick = event->posF();
	m_mappedMouseClick = m_screenTransform.inverted().map(m_mouseClick);
	m_mouseClickElapsed.restart();
	
	m_screenScale_y_orig = m_screenScale_y;

	m_swipeFadeOutTimer.stop();
	m_swipeFadeOutVelocity *= 0;
	
	m_pressHadShift = (event->modifiers() & Qt::ShiftModifier);

	if(event->modifiers() == Qt::NoModifier && event->button() == Qt::LeftButton && !m_playing)
		setCursor(Qt::ClosedHandCursor);
}

void PlotWidget::mouseMoveEvent(QMouseEvent *event)
{
	updateMouse(event->posF());

	setCursor(Qt::ArrowCursor);

	// Detect drag motion for panning
	if(event->buttons() & Qt::LeftButton && !m_pressHadShift)
	{
		m_screenOffset -= m_mappedMouseDiff;
		updateScreenTransform();
	}

	// Detect drag motion for vertical scaling
	if(event->buttons() & Qt::RightButton && !m_pressHadShift)
	{
		// We want to have the mappedMouseClick at the mouse point.
		m_screenScale_y = -(m_mappedMouseClick.y() - m_screenOffset.y()) / (m_mouse.y() - height()/2); // Old scale method: Scales in mapped coordinates, somewhat unstable
		m_screenScale_y = m_screenScale_y_orig * std::exp(m_vertScaleFactor() * (m_mouse.y() - m_mouseClick.y()) / height());
		updateScreenTransform();
	}

	update();
}

void PlotWidget::mouseReleaseEvent(QMouseEvent *event)
{
	if(m_pressHadShift)
	{
		ros::Time time = m_currentTime + ros::Duration(m_mappedMouse.x());
		Plot::LinkedBufferIterator it = m_plots[0]->findDataPointAt(time, m_mappedMouse.y(), Plot::FindRecursive | Plot::FindOnlyVisible);
		if(it.isValid())
			time = (*it).time;

		switch(event->button())
		{
			case Qt::LeftButton:
				m_selectionStart = time;

				if(time >= m_selectionEnd)
					m_selectionEnd = ros::Time(0);
				break;
			case Qt::RightButton:
				m_selectionEnd = time;

				if(time <= m_selectionStart)
					m_selectionStart = ros::Time(0);
				break;
			case Qt::MiddleButton:
				m_selectionStart = m_selectionEnd = ros::Time(0);
				break;
			default:
				break;
		}
		
		rangeChanged();
		
		m_pressHadShift = (event->modifiers() & Qt::ShiftModifier);
		update();
		return;
	}

	// Start swipe fade out if the screen was dragged
	if(event->button() == Qt::LeftButton)
	{
		if(m_mouse != m_mouseClick)
		{
			updateMouse(event->posF());
			m_swipeFadeOutVelocity = m_mappedMouseVelocity;
			m_swipeFadeOutElapsed.restart();
			m_swipeFadeOutTimer.start();
		}
		else
		{
			if(m_currentTime.isZero()) return;
			m_fixedTime = m_currentTime + ros::Duration(m_mappedMouse.x());
			emit timeChanged();
			rangeChanged();
		}
	}
	else if(event->button() == Qt::RightButton)
	{
		if(m_mouse == m_mouseClick)
		{
			m_fixedTime = ros::Time();
			emit timeChanged();
			rangeChanged();
			emit stopPlaying();
		}
	}
	else if(event->button() == Qt::MiddleButton)
	{
		m_screenScale_x = m_screenScale_y = 0.01;
		m_screenOffset = QPointF(0, 0);
		updateScreenTransform();
	}

	unsetCursor();
	update();
}

QPointF PlotWidget::scale(const QPointF& src)
{
	QPointF ret;
	ret.rx() = src.x() * m_screenScale_x;
	ret.ry() = src.y() * m_screenScale_y;

	return ret;
}

// Updates the mouse state (position and velocity).
void PlotWidget::updateMouse(QPointF mousePos)
{
	m_mouse = mousePos;
	m_mousePresent = true;
	m_mappedMouse = m_screenTransform.inverted().map(m_mouse);
	m_mouseDiff = (m_mouse - m_lastMouse);

	m_mappedMouseDiff = scale(m_mouseDiff);

	m_mappedMouseDiff.ry() = -m_mappedMouseDiff.y();

	double timeDiff = 0.001 * m_mouseClickElapsed.elapsed();
	if(timeDiff > 0.3)
	{
		m_mouseVelocity *= 0;
		m_mappedMouseVelocity = scale(m_mouseVelocity);
		m_mappedMouseVelocity.ry() = -m_mappedMouseVelocity.y();
		m_lastMouse = m_mouse;
		m_mouseClickElapsed.restart();
	}
	else if(timeDiff >= 0.003)
	{
		QPointF measuredMouseVelocity = (m_mouse - m_lastMouse)/timeDiff;
		m_mouseVelocity = 0.5*m_mouseVelocity + 0.5*measuredMouseVelocity;
		m_mappedMouseVelocity = scale(m_mouseVelocity);
		m_mappedMouseVelocity.ry() = -m_mappedMouseVelocity.y();
		m_lastMouse = m_mouse;
		m_mouseClickElapsed.restart();
	}

	emit timeChanged();
}

void PlotWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
	update();
}

void PlotWidget::wheelEvent(QWheelEvent *event)
{
	if(event->buttons() & Qt::LeftButton || event->buttons() & Qt::RightButton)
		return;
	
	if(event->delta() > 0)
	{
		m_screenScale_x /= m_wheelScaleFactor();
		m_screenScale_y /= m_wheelScaleFactor();
	}
	else
	{
		m_screenScale_x *= m_wheelScaleFactor();
		m_screenScale_y *= m_wheelScaleFactor();
	}

	updateScreenTransform();
	updateMouse(QPointF(event->pos()));
	update();
}

void PlotWidget::leaveEvent(QEvent* event)
{
	m_mousePresent = false;
	emit timeChanged();
}

void PlotWidget::resizeEvent(QResizeEvent* event)
{
	updateScreenTransform();
}

// Updates the transformation from screen to logical coordinates.
// This should be called each time when the scaling factor or the screen translation changes.
void PlotWidget::updateScreenTransform()
{
	// The screen is first scaled and then translated by the screen offset, i.e. the screen
	// offset is scaled too, so that zooming with the mouse wheel only has to adjust the
	// scaling factor and automatically does the right thing.

	m_screenTransform = QTransform();
	m_screenTransform.translate(width()/2, height()/2);
	m_screenTransform.scale(1.0/m_screenScale_x, -1.0/m_screenScale_y);
	m_screenTransform.translate(-m_screenOffset.x(), -m_screenOffset.y());
	m_mappedMouse = m_screenTransform.inverted().map(m_mouse);
}


// Handles the swipe fade out (inertial movement after the mouse button was released).
void PlotWidget::swipeFadeOut()
{
	if (m_swipeFadeOutVelocity.manhattanLength() < 0.8)
	{
		m_swipeFadeOutVelocity *= 0;
		m_swipeFadeOutTimer.stop();
	}
	else
	{
		double elapsedTime = 0.001 * m_swipeFadeOutElapsed.elapsed();
		m_screenOffset -= m_swipeFadeOutVelocity*elapsedTime;
		m_swipeFadeOutVelocity *= qMax(0.0, 1.0 - 4.0*elapsedTime);
		updateScreenTransform();
	}

	m_swipeFadeOutElapsed.restart();
	update();
}

void PlotWidget::drawLegend(QPainter* painter, const Plot* plot)
{
	const int fieldWidth = 9;

	int lineHeight = painter->fontMetrics().height();

	if(!plot->isEnabled())
		return;

	if(plot->hasData())
	{
		painter->setPen(QPen(plot->color(), 2.0));
		painter->drawLine(QPoint(0, 1), QPoint(10, 1));
		painter->setPen(Qt::black);

		double val;
		if(!m_fixedTime.isZero() || m_mousePresent)
			val = plot->value(currentTime());
		else
			val = plot->lastValue();

		painter->setFont(m_font_monospace);
		if(plot->isEventChannel())
			painter->drawText(QPoint(15, lineHeight/2), QString("%1").arg("event", fieldWidth));
		else
			painter->drawText(QPoint(15, lineHeight/2), QString("%1").arg(val, fieldWidth, 'f', fieldWidth - 5));
		int valueWidth = painter->fontMetrics().averageCharWidth() * fieldWidth;

		painter->setFont(m_font_default);
		painter->drawText(QPoint(15 + valueWidth + 5, lineHeight/2), plot->path());
		painter->translate(0, lineHeight);
	}

	for(int i = 0; i < plot->childCount(); ++i)
		drawLegend(painter, plot->child(i));
}

void PlotWidget::paintEvent(QPaintEvent* )
{
	QPainter painter(this);

	painter.fillRect(rect(), Qt::white);

	// Calculate the bounding box in the transformed coordinate system.
	QPointF topLeft = m_screenTransform.inverted().map(QPointF(0,0));
	QPointF bottomRight = m_screenTransform.inverted().map(QPointF(width(),height()));
	QRectF boundingBox = QRectF(topLeft, bottomRight);

	// Draw selection boxes
	QColor unselectedColor(0xE0, 0xE0, 0xE0);
	if(!m_selectionStart.isZero())
	{
		QRectF rect(boundingBox.topLeft(), QPointF((m_selectionStart - m_currentTime).toSec(), boundingBox.bottom()));
		painter.fillRect(m_screenTransform.mapRect(rect), unselectedColor);
	}
	if(!m_selectionEnd.isZero())
	{
		QRectF rect(QPointF((m_selectionEnd - m_currentTime).toSec(), boundingBox.top()), boundingBox.bottomRight());
		painter.fillRect(m_screenTransform.mapRect(rect), unselectedColor);
	}

	// Prepare the basic painter colors and pen.
	QPen pen = QPen(QColor(0, 0, 0));
	pen.setCosmetic(true);
	painter.setPen(pen);
	painter.setFont(m_font_default);
	QFontMetrics fm(painter.font());

	painter.save();
	painter.translate(10, 10);
	drawLegend(&painter, m_plots[0]);
	painter.restore();

	// Draw mouse location in lower right corner
	if(m_mousePresent)
	{
		painter.save();
		painter.setFont(m_font_monospace);

		int h = painter.fontMetrics().height();
		QRect footer(0, height() - h, width(), h);

		QString text = QString("%1s: %2")
				.arg(m_mappedMouse.x(), 10, 'f', 4)
				.arg(m_mappedMouse.y(), 10, 'f', 4)
		;

		if(!m_selectionStart.isZero() && !m_selectionEnd.isZero())
		{
			text.append(QString(" selected %1s to %2s (diff %3s)")
				.arg((m_selectionStart - m_currentTime).toSec(), 10, 'f', 4)
				.arg((m_selectionEnd - m_currentTime).toSec(), 10, 'f', 4)
				.arg((m_selectionEnd - m_selectionStart).toSec(), 10, 'f', 4)
			);
		}

		painter.drawText(footer, Qt::AlignRight, text);

		painter.restore();
	}

	// Apply the coordinate transformation from device coordinates ((0,0) is in the top left corner)
	// to logical coordinates, where the origin of the coordinate system is in the middle and y grows
	// upwards. A scaling factor converts from pixel values to logical units (e.g. 100 px = 1 second).
	painter.setTransform(m_screenTransform);

	// Draw the axes.
	painter.drawLine(QPointF(boundingBox.left(), 0), QPointF(boundingBox.right(), 0));
	painter.drawLine(QPointF(0, boundingBox.bottom()), QPointF(0, boundingBox.top()));

	for (double i = floor(boundingBox.left()); i < boundingBox.right(); i=i+1.0)
		painter.drawLine(QPointF(i, 0), QPointF(i, -4.0*m_screenScale_y));
	for (double i = floor(boundingBox.bottom()); i < boundingBox.top(); i=i+1.0)
		painter.drawLine(QPointF(0, i), QPointF(4.0*m_screenScale_x, i));


	painter.setRenderHint(QPainter::Antialiasing, true);

	ros::Time now = m_plots[0]->recursiveLastTime();
	for(int i = 0; i < m_plots.count(); ++i)
	{
		m_plots[i]->draw(&painter, now, boundingBox, m_showDots);
	}

	// Draw fixed time marker
	painter.setPen(Qt::black);
	if(!m_fixedTime.isZero())
	{
		double sec = (m_fixedTime - now).toSec();
		painter.drawLine(
			QPointF(sec, -1000),
			QPointF(sec, 1000)
		);
	}
	else if(m_mousePresent)
	{
		painter.drawLine(
			QPointF(m_mappedMouse.x(), -1000),
			QPointF(m_mappedMouse.x(), 1000)
		);
	}

	m_currentTime = now;
}

ros::Time PlotWidget::currentTime() const
{
	if(!m_fixedTime.isZero())
		return m_fixedTime;

	if(m_currentTime.isZero())
		return ros::Time();

	if(m_mousePresent)
		return m_currentTime + ros::Duration(m_mappedMouse.x());
	else
		return m_currentTime;
}

void PlotWidget::setShowDots(bool enabled)
{
	m_showDots = enabled;
	update();
}

void PlotWidget::toggleDots()
{
	m_showDots = !m_showDots;
	update();
}

void PlotWidget::advanceFixedTime()
{
	if(!m_fixedTime.isZero() && !m_playing)
	{
		m_fixedTime += ros::Duration(m_cursorStep());
		emit timeChanged();
		update();
	}
}

void PlotWidget::retreatFixedTime()
{
	if(!m_fixedTime.isZero() && !m_playing)
	{
		m_fixedTime -= ros::Duration(m_cursorStep());
		emit timeChanged();
		update();
	}
}

void PlotWidget::rangeChanged()
{
	emit selectionChanged(m_selectionStart.toSec(), m_selectionEnd.toSec(), !m_fixedTime.isZero());
}

}
