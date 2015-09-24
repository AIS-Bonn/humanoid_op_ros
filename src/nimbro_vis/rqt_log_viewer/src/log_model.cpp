// Qt Model for a log stream
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <QtCore/QTimer>

#include <QtGui/QColor>

#include "log_model.h"

namespace rqt_log_viewer
{

LogModel::LogModel(QObject* parent)
 : QAbstractTableModel(parent)
{
	m_pruneTimer = new QTimer(this);

	connect(m_pruneTimer, SIGNAL(timeout()), SLOT(prune()));
	m_pruneTimer->start(1000);

	m_startTime = ros::Time::now();
}

LogModel::~LogModel()
{
}

int LogModel::rowCount(const QModelIndex& parent) const
{
	if(parent.isValid())
		return 0;

	return m_messages.count();
}

int LogModel::columnCount(const QModelIndex& parent) const
{
	if(parent.isValid())
		return 0;

	return COL_COUNT;
}

QVariant LogModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if(orientation != Qt::Horizontal || role != Qt::DisplayRole)
		return QAbstractItemModel::headerData(section, orientation, role);

	switch(section)
	{
		case COL_TIME:
			return "Time";
		case COL_RELATIVE_TIME:
			return "Relative time";
		case COL_SOURCE:
			return "Node";
		case COL_TEXT:
			return "Text";
	}

	return QVariant();
}

void LogModel::insertMessage(int idx, const rosgraph_msgs::Log& msg)
{
	beginInsertRows(QModelIndex(), idx, idx);
	m_messages.insert(idx, msg);
	m_insertionPoint = idx+1;
	endInsertRows();

	if(msg.header.stamp < m_startTime)
	{
		m_startTime = msg.header.stamp;
		dataChanged(
			index(0, COL_RELATIVE_TIME),
			index(m_messages.count()-1, COL_RELATIVE_TIME)
		);
	}
}

class CompareTimeStamps
{
public:
	bool operator()(const rosgraph_msgs::Log& a, const rosgraph_msgs::Log& b)
	{
		return a.header.stamp <= b.header.stamp;
	}
};

void LogModel::addMessage(const rosgraph_msgs::Log& msg)
{
	// We need to find an appropriate insertion point in m_messages.
	// m_messages is sorted. However, a plain binary sort takes too long
	// for normal messages, because they usually arrive in order.

	// Trivial case
	if(m_messages.isEmpty())
	{
		insertMessage(0, msg);
		return;
	}

	// Try the last message
	if(msg.header.stamp >= m_messages.last().header.stamp)
	{
		insertMessage(m_messages.count(), msg);
		return;
	}

	// Try the last insertion point (good for messages that arrive in order)
	if(m_insertionPoint > 0 && m_insertionPoint < m_messages.count()
	   && m_messages[m_insertionPoint-1].header.stamp <= msg.header.stamp)
	{
		if(m_insertionPoint == m_messages.count()
		   || m_messages[m_insertionPoint].header.stamp >= msg.header.stamp)
		{
			insertMessage(m_insertionPoint, msg);
			return;
		}
	}

	// Fall back to binary search
	QList<rosgraph_msgs::Log>::iterator it = qLowerBound(
		m_messages.begin(), m_messages.end(), msg, CompareTimeStamps()
	);

	insertMessage(it - m_messages.begin(), msg);
}

void LogModel::prune()
{
	if(m_messages.count() <= 5000)
		return;

	int remove = m_messages.count() - 5000;
	beginRemoveRows(QModelIndex(), 0, remove-1);
	m_messages.erase(m_messages.begin(), m_messages.begin() + remove);
	endRemoveRows();
}

QVariant LogModel::data(const QModelIndex& index, int role) const
{
	if(index.column() >= COL_COUNT)
		return QVariant();

	if(index.row() >= m_messages.count())
		return QVariant();

	const rosgraph_msgs::Log& msg = m_messages[index.row()];

	if(role == ROLE_PTR)
		return QVariant::fromValue<void*>(const_cast<rosgraph_msgs::Log*>(&msg));

	if(role == Qt::TextColorRole)
	{
		switch(msg.level)
		{
			case rosgraph_msgs::Log::FATAL:
			case rosgraph_msgs::Log::ERROR:
				return Qt::red;
			case rosgraph_msgs::Log::WARN:
				return QVariant::fromValue(QColor(178, 104, 24));
			case rosgraph_msgs::Log::INFO:
				return Qt::black;
			case rosgraph_msgs::Log::DEBUG:
				return Qt::green;
		}

		return QVariant();
	}

	if(role != Qt::DisplayRole)
		return QVariant();

	switch(index.column())
	{
		case COL_TIME:
			return msg.header.stamp.toSec();
		case COL_RELATIVE_TIME:
			return (qlonglong)(msg.header.stamp - m_startTime).toNSec();
		case COL_SOURCE:
			return QString::fromStdString(msg.name);
		case COL_TEXT:
			return QString::fromStdString(msg.msg);
	}

	return QVariant();
}

void LogModel::clear()
{
	beginResetModel();
	m_messages.clear();
	endResetModel();
}

}
