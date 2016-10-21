// A class that maintains an undo/redo history for a particular variable
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//         Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <parameter_tuner/historykeeper.h>

using namespace parametertuner;

HistoryKeeper::HistoryKeeper(int historySize, double waitTime, double emphasisTime)
 : m_treeItem(NULL)
{
	m_historySize = historySize;
	if(m_historySize < 1)
		m_historySize = 1;

	if(waitTime < 0.1)
		waitTime = 0.1;

	if(emphasisTime < 0.1)
		emphasisTime = 0.1;

	m_pendingTimer.setInterval(qRound(1000.0*waitTime));
	m_pendingTimer.setSingleShot(true);
	connect(&m_pendingTimer, SIGNAL(timeout()), this, SLOT(handlePendingTimerTimeout()));

	m_widgetTimer.setInterval(qRound(1000.0*emphasisTime));
	m_widgetTimer.setSingleShot(true);
	connect(&m_widgetTimer, SIGNAL(timeout()), this, SLOT(handleWidgetTimerTimeout()));

	reset();
}

HistoryKeeper::~HistoryKeeper()
{
	reset();
}

void HistoryKeeper::reset()
{
	m_pendingTimer.stop();
	m_widgetTimer.stop();
	clearFilterWidgets();
	clearTreeItem();
	clearHistory();
}

void HistoryKeeper::clearHistory()
{
	m_history.clear();
	m_history.reserve(m_historySize + 1);
	m_current = m_history.end();
}

void HistoryKeeper::addNewValue(QVariant value)
{
	const QVariant* latest = NULL;
	if(m_pendingTimer.isActive())
		latest = &m_pending;
	else if(m_current != m_history.end())
		latest = &(*m_current);

	if(!latest || (latest && value != *latest))
	{
		m_pending = value;
		m_pendingTimer.start();
	}
}

bool HistoryKeeper::undo()
{
	if(canUndo())
	{
		valueChanged(performUndo());
		return true;
	}
	return false;
}

bool HistoryKeeper::redo()
{
	if(canRedo())
	{
		valueChanged(performRedo());
		return true;
	}
	return false;
}

void HistoryKeeper::clearFilterWidgets()
{
	for(int i = 0; i < m_filterWidgets.size(); i++)
		m_filterWidgets[i]->removeEventFilter(this);
	m_filterWidgets.clear();
}

void HistoryKeeper::addFilterWidget(QWidget* widget)
{
	if(!widget) return;
	widget->installEventFilter(this);
	m_filterWidgets.push_back(widget);
}

bool HistoryKeeper::eventFilter(QObject *object, QEvent *event)
{
	if(event->type() != QEvent::KeyPress || m_filterWidgets.empty())
		return false;

	if(!m_treeItem->isSelected())
	{
		bool haveFocus = false;
		for(int i = 0; i < m_filterWidgets.size(); i++)
		{
			if(m_filterWidgets[i]->hasFocus())
			{
				haveFocus = true;
				break;
			}
		}
		if(!haveFocus)
			return false;
	}

	QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);

	if(keyEvent->key() == Qt::Key_Z && keyEvent->modifiers() == Qt::ControlModifier) 
	{
		if(m_treeItem)
		{
			setBackground(QColor("lightpink"));
			m_widgetTimer.start();
		}
		undo();
		return true;
	}

	if(keyEvent->key() == Qt::Key_Z && keyEvent->modifiers() == (Qt::ControlModifier | Qt::ShiftModifier))
	{
		if(m_treeItem)
		{
			setBackground(QColor("palegreen"));
			m_widgetTimer.start();
		}
		redo();
		return true;
	}

	return false;
}

void HistoryKeeper::handlePendingTimerTimeout()
{
	m_pendingTimer.stop(); // Just to be very sure...
	performAdd(m_pending);
}

void HistoryKeeper::handleWidgetTimerTimeout()
{
	m_widgetTimer.stop(); // Just to be very sure...
	clearBackground();
}

void HistoryKeeper::performAdd(QVariant value)
{
	if(!m_history.empty())
	{
		if(m_current == m_history.end())
			m_current--;
		if(value == *m_current)
			return;
		m_current++;
		if(m_current != m_history.end())
			m_history.erase(m_current, m_history.end());
	}
	m_history.push_back(value);
	m_current = m_history.end() - 1;

	int oversize = m_history.size() - m_historySize;
	if(oversize >= 1)
		m_history.erase(m_history.begin(), m_history.begin() + oversize);
}

bool HistoryKeeper::canUndo() const
{
	return (m_current != m_history.end() && m_current != m_history.begin());
}

bool HistoryKeeper::canRedo() const
{
	return (m_current != m_history.end() && m_current + 1 != m_history.end());
}

QVariant HistoryKeeper::performUndo()
{
	return *(--m_current);
}

QVariant HistoryKeeper::performRedo()
{
	return *(++m_current);
}

void HistoryKeeper::setBackground(const QColor& color)
{
	for(int i = 0; i < m_filterWidgets.size(); i++)
		m_filterWidgets[i]->setStyleSheet(QString("background-color:%1;").arg(color.name()));
	for(int i = 0; i < m_treeItem->columnCount(); i++)
		m_treeItem->setBackgroundColor(i, color);
}

void HistoryKeeper::clearBackground()
{
	QColor white("white");
	for(int i = 0; i < m_filterWidgets.size(); i++)
		m_filterWidgets[i]->setStyleSheet("");
	for(int i = 0; i < m_treeItem->columnCount(); i++)
		m_treeItem->setBackgroundColor(i, white);
}
// EOF