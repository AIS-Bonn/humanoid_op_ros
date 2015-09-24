// Customized Qt view for log entries
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "log_view.h"
#include "log_model.h"

#include <QtCore/QEvent>
#include <QtCore/QDebug>

#include <QtGui/QPainter>
#include <QtGui/QScrollBar>
#include <QtGui/QResizeEvent>
#include <QtGui/QSortFilterProxyModel>
#include <QtGui/QApplication>

#include <ros/assert.h>

static const int TIMESTAMP_WIDTH = 10;
static const int MARGIN = 2;
static const int SOURCE_WIDTH = 20;

static const int INFO_WIDTH = TIMESTAMP_WIDTH + MARGIN + SOURCE_WIDTH + MARGIN;

namespace rqt_log_viewer
{

LogView::LogView(QWidget* parent)
 : QAbstractItemView(parent)
 , m_sticky(true)
 , m_textWidth(100)
 , m_displayAbsoluteStamps(false)
{
	setFont(QFont("Monospace", 9));
	m_lineSpacing = fontMetrics().lineSpacing();
// 	setMouseTracking(true);
}

LogView::~LogView()
{
}

int LogView::horizontalOffset() const
{
	return 0;
}

int LogView::verticalOffset() const
{
	return 0;
}

QModelIndex LogView::indexAt(const QPoint& point) const
{
	int y = 0;

	for(int i = 0; i < m_shownMessages.count(); ++i)
	{
		int messageHeight = m_shownMessageHeights[i];
		if(point.y() < y + messageHeight)
		{
			int msg = m_shownMessages[i];
			if(point.x() < TIMESTAMP_WIDTH)
				return model()->index(msg, LogModel::COL_TIME);
			if(point.x() < TIMESTAMP_WIDTH + MARGIN + SOURCE_WIDTH)
				return model()->index(msg, LogModel::COL_SOURCE);

			return model()->index(msg, LogModel::COL_TEXT);
		}

		y += messageHeight;
	}

	return QModelIndex();
}

bool LogView::isIndexHidden(const QModelIndex& index) const
{
	// FIXME
	return false;
}

QModelIndex LogView::moveCursor(QAbstractItemView::CursorAction cursorAction, Qt::KeyboardModifiers modifiers)
{
	return QModelIndex();
}

void LogView::scrollTo(const QModelIndex& index, QAbstractItemView::ScrollHint hint)
{
	int height = m_messageLines[index.row()];
	int viewportLines = viewport()->height() / m_lineSpacing;

	// Determine current position of the item
	bool above;
	bool below;

	int lineno = 0;
	for(int i = 0; i < index.row(); ++i)
		lineno += m_messageLines[i];

	int topLine = verticalScrollBar()->value();
// 	int bottomLine = topLine + viewportLines; // TODO: This variable is unused

	above = lineno < topLine;
	below = lineno + height > topLine + viewportLines;

	int valueForBottom = lineno + height;
	int h = 0;
	for(int i = index.row(); i >= 0; --i)
	{
		int msgHeight = m_messageLines[i];
		if(h + msgHeight >= viewportLines)
		{
			valueForBottom -= std::min(viewportLines - h, msgHeight);
			break;
		}

		valueForBottom -= msgHeight;
		h += msgHeight;
	}

	switch(hint)
	{
		case QAbstractItemView::PositionAtTop:
			verticalScrollBar()->setValue(lineno);
			break;
		case QAbstractItemView::PositionAtBottom:
			verticalScrollBar()->setValue(valueForBottom);
			break;
		case QAbstractItemView::PositionAtCenter:
			verticalScrollBar()->setValue((lineno + valueForBottom)/2);
			break;
		case QAbstractItemView::EnsureVisible:
			if(above && below)
				verticalScrollBar()->setValue(lineno); // best guess at what the user wants
			else if(above)
				verticalScrollBar()->setValue(lineno);
			else if(below)
				verticalScrollBar()->setValue(valueForBottom);
			break;
	}
}

void LogView::setSelection(const QRect& rect, QItemSelectionModel::SelectionFlags command)
{
}

QRegion LogView::visualRegionForSelection(const QItemSelection& selection) const
{
	return QRegion();
}

QRect LogView::visualRect(const QModelIndex& index) const
{
	return QRect();
}

int LogView::findHeightIndex(int* ypos)
{
	// Find message corresponding to scroll position
	// depending on the scroll bar position it might be faster to count from
	// the end of m_messageHeights.

	int scrollPos = verticalScrollBar()->value();

	if(scrollPos >= verticalScrollBar()->maximum()/2)
	{
		int pos = m_lines;

		for(int i = m_messageLines.count()-1; i >= 0; --i)
		{
			pos -= m_messageLines[i];
			if(pos <= scrollPos)
			{
				*ypos = -(scrollPos - pos) * m_lineSpacing;
				return i;
			}
		}
	}
	else
	{
		int pos = 0;
		for(int i = 0; i < m_messageLines.count(); ++i)
		{
			int h = m_messageLines[i];
			pos += h;
			if(pos > scrollPos)
			{
				*ypos = -(scrollPos - (pos - h)) * m_lineSpacing;
				return i;
			}
		}
	}

	*ypos = 0;
	return 0;
}

void LogView::paintEvent(QPaintEvent* event)
{
	QPainter painter(viewport());

	painter.setFont(font());

	int em = painter.fontMetrics().averageCharWidth();

	painter.fillRect(rect(), Qt::black);

	if(!model() || m_messageLines.isEmpty())
		return;

	int y;
	int heightIdx = findHeightIndex(&y);

	m_shownMessageHeights.clear();
	m_shownMessages.clear();

	int i;
	for(i = heightIdx; i < model()->rowCount(); ++i)
	{
		int x = 0;
		int height = m_lineSpacing;

		QModelIndex idx = model()->index(i, 0);

		void *ptr = model()->data(idx, LogModel::ROLE_PTR).value<void*>();
		ROS_ASSERT(ptr);

		const rosgraph_msgs::Log& msg = *(reinterpret_cast<rosgraph_msgs::Log*>(ptr));

		int textWidth = viewport()->width() - (TIMESTAMP_WIDTH + MARGIN + SOURCE_WIDTH + MARGIN)*em;
		if(textWidth > 0)
			height = m_messageLines[heightIdx] * m_lineSpacing;

		QColor color;
		switch(msg.level)
		{
			case rosgraph_msgs::Log::FATAL:
			case rosgraph_msgs::Log::ERROR:
				color = Qt::red;
				break;
			case rosgraph_msgs::Log::WARN:
				color = QColor(178, 104, 24);
				break;
			case rosgraph_msgs::Log::INFO:
				color = Qt::white;
				break;
			case rosgraph_msgs::Log::DEBUG:
				color = Qt::green;
				break;
		}

		if(selectionModel()->isSelected(idx))
		{
			painter.fillRect(x, y, viewport()->width(), height, color);
			color = Qt::black;
		}

		if(!m_highlightedNode.empty() && msg.name != m_highlightedNode)
		{
			color = color.darker(150);
		}
		painter.setPen(color);

		if(!m_displayAbsoluteStamps)
		{
			int64_t stamp = model()->index(i, LogModel::COL_RELATIVE_TIME).data().toLongLong();
			stamp /= 1000 * 1000;

			int msecs = stamp % 1000;
			stamp /= 1000;

			int secs = stamp % 60;
			stamp /= 60;

			int mins = stamp;

			painter.drawText(0, y, TIMESTAMP_WIDTH*em, m_lineSpacing,
				Qt::AlignRight,
				QString("%1:%2.%3")
					.arg(mins, 0, 10, QLatin1Char('0'))
					.arg(secs, 2, 10, QLatin1Char('0'))
					.arg(msecs, 3, 10, QLatin1Char('0'))
			);
		}
		else
		{
			painter.drawText(0, y, TIMESTAMP_WIDTH*em, m_lineSpacing,
				Qt::AlignRight,
				QString::number(msg.header.stamp.toSec(), 'f', 3)
			);
		}
		x += TIMESTAMP_WIDTH*em + MARGIN*em;

		painter.drawText(x, y, SOURCE_WIDTH*em, m_lineSpacing,
			0,
			QString::fromStdString(msg.name)
		);
		x += SOURCE_WIDTH*em + MARGIN*em;


		if(textWidth > 0)
		{
			painter.drawText(
				x, y, textWidth, height,
				Qt::TextWordWrap,
				QString::fromStdString(msg.msg)
			);

			if(height > m_lineSpacing)
			{
				const int MARGIN = 2;
				painter.drawLine(
					x-2, y+MARGIN,
					x-5, y+MARGIN
				);
				painter.drawLine(
					x-5, y+MARGIN,
					x-5, y+height-MARGIN
				);
				painter.drawLine(
					x-5, y+height-MARGIN,
					x-2, y+height-MARGIN
				);
			}
		}

		m_shownMessageHeights << (height + std::min(0, y));

		y += height;

		m_shownMessages << i;

		if(y > viewport()->height())
			break;

		heightIdx++;
	}

	m_lastItemVisible = (i == model()->rowCount());

	if(!m_sticky && m_lastItemVisible)
		m_sticky = true;

	if(!m_lastItemVisible && verticalScrollBar()->value() != verticalScrollBar()->maximum())
		m_sticky = false;
}

int LogView::numLinesForText(const QString& text)
{
	QRect bbox = fontMetrics().boundingRect(
		0, 0, m_textWidth, INT_MAX,
		Qt::TextWordWrap,
		text
	);

	return bbox.height() / m_lineSpacing;
}

void LogView::calculateMessageHeights()
{
	m_messageLines.clear();
	m_lines = 0;

	if(!model())
		return;

	for(int i = 0; i < model()->rowCount(); ++i)
	{
		void *ptr = model()->data(model()->index(i, 0), LogModel::ROLE_PTR).value<void*>();
		const rosgraph_msgs::Log& msg = *reinterpret_cast<rosgraph_msgs::Log*>(ptr);

		int lines = numLinesForText(QString::fromStdString(msg.msg));

		m_messageLines << lines;
		m_lines += lines;
	}

	verticalScrollBar()->setMaximum(std::max(0, m_lines - viewport()->height()/m_lineSpacing));
}

void LogView::reset()
{
	QAbstractItemView::reset();

	calculateMessageHeights();
	verticalScrollBar()->setValue(0);

	viewport()->update();
}

void LogView::rowsInserted(const QModelIndex& parent, int start, int end)
{
	QAbstractItemView::rowsInserted(parent, start, end);

	if(!model())
		return;

	QList<int>::iterator ith = m_messageLines.begin() + start;

	for(int i = start; i <= end; ++i)
	{
		void *ptr = model()->data(model()->index(i, 0), LogModel::ROLE_PTR).value<void*>();
		const rosgraph_msgs::Log& msg = *reinterpret_cast<rosgraph_msgs::Log*>(ptr);

		int lines = numLinesForText(QString::fromStdString(msg.msg));

		ith = m_messageLines.insert(ith, lines);
		m_lines += lines;
	}

	verticalScrollBar()->setMaximum(std::max(0, m_lines - viewport()->height()/m_lineSpacing));

	if(m_sticky)
		verticalScrollBar()->setValue(verticalScrollBar()->maximum());

	viewport()->update();
}

void LogView::rowsAboutToBeRemoved(const QModelIndex& parent, int start, int end)
{
	QAbstractItemView::rowsAboutToBeRemoved(parent, start, end);

	int scrollPos = verticalScrollBar()->value();

	int pos = 0;
	for(int i = 0; i < start; ++i)
	{
		pos += m_messageLines[i];
	}

	QList<int>::iterator ith = m_messageLines.begin() + start;
	for(int i = start; i <= end; ++i)
	{
		int lines = *ith;

		m_lines -= lines;

		if(pos < scrollPos)
			scrollPos -= std::min(lines, scrollPos - pos);

		ith = m_messageLines.erase(ith);
	}

	verticalScrollBar()->setMaximum(std::max(0, m_lines - viewport()->height()/m_lineSpacing));
	verticalScrollBar()->setValue(scrollPos);

	viewport()->update();
}

bool LogView::viewportEvent(QEvent* event)
{
	if(event->type() == QEvent::Resize)
	{
		QResizeEvent* resize = (QResizeEvent*)event;

		m_textWidth = resize->size().width() - INFO_WIDTH * fontMetrics().averageCharWidth();

		calculateMessageHeights();

		if(m_sticky)
			verticalScrollBar()->setValue(verticalScrollBar()->maximum());
	}

	return QAbstractItemView::viewportEvent(event);
}

void LogView::enterEvent(QEvent* event)
{
	event->accept();
}

void LogView::mouseMoveEvent(QMouseEvent* event)
{
	QAbstractItemView::mouseMoveEvent(event);
}

void LogView::mousePressEvent(QMouseEvent* event)
{
}

void LogView::mouseDoubleClickEvent(QMouseEvent* event)
{

}

void LogView::selectMessage(int idx)
{
	QModelIndex index = model()->index(idx, 0);

	selectionModel()->select(index,
				 QItemSelectionModel::ClearAndSelect
				 | QItemSelectionModel::Rows
				 | QItemSelectionModel::Current
	);
	selectionModel()->setCurrentIndex(index, QItemSelectionModel::Rows | QItemSelectionModel::Current);

	std::string highlight = model()->index(idx, LogModel::COL_SOURCE).data().toString().toStdString();
	if(m_highlightedNode != highlight)
		m_highlightedNode = highlight;

	// Temporarily disable sticky mode, so that the log does not move
	// because the viewport is made smaller by the details window
	// If the last message is still visible, paintEvent() will enable it
	// again.
	m_sticky = false;

	scrollTo(index);
	viewport()->update();
}

void LogView::mouseReleaseEvent(QMouseEvent* event)
{
	if(event->button() != Qt::LeftButton)
		return;

	QModelIndex idx = indexAt(event->pos());

	if(!idx.isValid())
	{
		m_highlightedNode.clear();
		selectionModel()->clearSelection();
		viewport()->update();
		return;
	}

	selectMessage(idx.row());
}

void LogView::leaveEvent(QEvent* event)
{
// 	QWidget::leaveEvent(event);
}

void LogView::deselect()
{
	m_highlightedNode.clear();
	selectionModel()->clear();
	viewport()->update();
}

void LogView::keyPressEvent(QKeyEvent* event)
{
	switch(event->key())
	{
		case Qt::Key_Down:
		{
			QModelIndex index = selectionModel()->currentIndex();
			if(!index.isValid())
				return;

			if(index.row() >= model()->rowCount()-1)
				return;

			selectMessage(index.row() + 1);
			break;
		}
		case Qt::Key_Up:
		{
			QModelIndex index = selectionModel()->currentIndex();
			if(!index.isValid())
				return;

			if(index.row() == 0)
				return;

			selectMessage(index.row() - 1);
			break;
		}
		case Qt::Key_Escape:
			deselect();
			break;
	}
}

void LogView::setDisplayAbsoluteTimeStamps(bool on)
{
	m_displayAbsoluteStamps = on;
	viewport()->update();
}

void LogView::dataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight)
{
	QAbstractItemView::dataChanged(topLeft, bottomRight);
	viewport()->update();
}

}
