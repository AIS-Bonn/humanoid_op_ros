// Qt model for all running nodes, exposing log level settings
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "node_model.h"

#include <ros/master.h>
#include <ros/service.h>

#include <roscpp/GetLoggers.h>
#include <roscpp/SetLoggerLevel.h>

#include <ros/console.h>

#include <boost/foreach.hpp>

namespace rqt_log_viewer
{

NodeModel::NodeModel(QObject* parent)
 : QAbstractItemModel(parent)
{
	m_backend = new NodeBackend;
	m_backend->moveToThread(&m_thread);

	connect(m_backend, SIGNAL(nodeListChanged(rqt_log_viewer::NodeList)),
		SLOT(processNodeList(rqt_log_viewer::NodeList))
	);
	connect(m_backend, SIGNAL(logLevelChanged(QString,QString,QString)),
		SLOT(processLogLevelChange(QString,QString,QString))
	);

	connect(this, SIGNAL(levelChangeRequested(QString,QString,QString)),
		m_backend, SLOT(changeLogLevel(QString,QString,QString))
	);

	m_thread.start();

	QMetaObject::invokeMethod(m_backend, "discoverNodes", Qt::QueuedConnection);

	for(int i = 0; i < ros::console::levels::Count; ++i)
		m_logLevelLabels << "";

	m_logLevelLabels[ros::console::levels::Debug] = "Debug";
	m_logLevelLabels[ros::console::levels::Info] = "Info";
	m_logLevelLabels[ros::console::levels::Warn] = "Warn";
	m_logLevelLabels[ros::console::levels::Error] = "Error";
	m_logLevelLabels[ros::console::levels::Fatal] = "Fatal";
}

NodeModel::~NodeModel()
{
	if(m_thread.isRunning())
		stopBackend();

	delete m_backend;
}

void NodeModel::stopBackend()
{
	m_thread.quit();
	m_thread.wait();
}

void NodeModel::updateNode(int idx, const Node& source)
{
	const QModelIndex nodeIndex = index(idx, 0);
	Node* dest = &m_nodes[idx];
	Q_ASSERT(dest->name == source.name);

	dest->debug = source.debug;

	int i = 0;
	int j = 0;

	while(i < dest->loggers.count() || j < source.loggers.count())
	{
		if(i < dest->loggers.count() && j < source.loggers.count()
		   && dest->loggers[i].name == source.loggers[j].name)
		{
			dest->loggers[i] = source.loggers[j];
			QModelIndex index = nodeIndex.child(i, COL_DEBUG);
			dataChanged(index, index);
			++i;
			++j;
			continue;
		}

		if(j < source.loggers.count()
		   && (i >= dest->loggers.count() || dest->loggers[i].name > source.loggers[j].name))
		{
			beginInsertRows(nodeIndex, i, i);
			dest->loggers.insert(i, source.loggers[j]);
			endInsertRows();
			++i;
			++j;
			continue;
		}

		if(i < dest->loggers.count()
		   && (j >= source.loggers.count() || dest->loggers[i] < source.loggers[i]))
		{
			beginRemoveRows(nodeIndex, i, i);
			dest->loggers.removeAt(i);
			endRemoveRows();
			continue;
		}
	}
}

void NodeModel::processNodeList(const NodeList& nodes)
{
	int i = 0;
	int j = 0;

	while(i < m_nodes.count() || j < nodes.count())
	{
		if(i < m_nodes.count() && j < nodes.count() && m_nodes[i].name == nodes[j].name)
		{
			updateNode(i, nodes[j]);
			++i;
			++j;
			continue;
		}

		if(j < nodes.count() && (i >= m_nodes.count() || m_nodes[i].name > nodes[j].name))
		{
			beginInsertRows(QModelIndex(), i, i);
			m_nodes.insert(i, nodes[j]);
			endInsertRows();

			++i;
			++j;
			continue;
		}

		if(i < m_nodes.count() && (j >= nodes.count() || m_nodes[i].name < nodes[j].name))
		{
			// Never forget nodes, that messes up the log level filtering.
// 			beginRemoveRows(QModelIndex(), i, i);
// 			m_nodes.removeAt(i);
// 			endRemoveRows();
			i++;
			continue;
		}
	}
}

QModelIndex NodeModel::index(int row, int column, const QModelIndex& parent) const
{
	if(!parent.isValid())
		return createIndex(row, column, -1);

	if(parent.internalId() != -1)
		return QModelIndex();

	return createIndex(row, column, parent.row());
}

QModelIndex NodeModel::parent(const QModelIndex& child) const
{
	if(!child.isValid() || child.internalId() < 0)
		return QModelIndex();

	return createIndex(child.internalId(), 0, -1);
}

int NodeModel::columnCount(const QModelIndex& parent) const
{
	if(parent.isValid() && parent.internalId() != -1)
		return 0;

	return COL_COUNT;
}

int NodeModel::rowCount(const QModelIndex& parent) const
{
	if(parent.isValid())
	{
		if(parent.internalId() != -1)
			return 0;
		else
			return m_nodes[parent.row()].loggers.count();
	}

	return m_nodes.count();
}

QVariant NodeModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if(orientation != Qt::Horizontal || role != Qt::DisplayRole)
		return QAbstractItemModel::headerData(section, orientation, role);

	switch(section)
	{
		case COL_SHOW:
			return "";
		case COL_NAME:
			return "Node";
		case COL_DEBUG:
			return "Dbg";
		case COL_LEVEL:
			return "Level";
	}

	return QVariant();
}

Qt::ItemFlags NodeModel::flags(const QModelIndex& index) const
{
	Qt::ItemFlags ret = QAbstractItemModel::flags(index);

	switch(index.column())
	{
		case COL_DEBUG:
			ret |= Qt::ItemIsEditable;
		case COL_SHOW:
		case COL_LEVEL:
			if(index.internalId() == -1)
				ret |= Qt::ItemIsEditable;
			break;
	}

	return ret;
}

QVariant NodeModel::data(const QModelIndex& index, int role) const
{
	if(index.column() >= COL_COUNT)
		return QVariant();

	if(index.internalId() == -1)
	{
		if(index.row() >= m_nodes.count())
			return QVariant();

		const Node& node = m_nodes[index.row()];

		if(role == Qt::UserRole && index.column() == COL_LEVEL)
			return m_logLevelLabels;

		if(role == Qt::EditRole && index.column() == COL_LEVEL)
			return node.logLevel;

		if(role != Qt::DisplayRole)
			return QVariant();

		switch(index.column())
		{
			case COL_SHOW:
				return node.shown;
			case COL_NAME:
				return node.name;
			case COL_DEBUG:
				return node.debug;
			case COL_LEVEL:
				return m_logLevelLabels[node.logLevel];
		}
	}
	else
	{
		const Node& node = m_nodes[index.internalId()];

		if(index.row() >= node.loggers.count())
			return QVariant();

		const Logger& logger = node.loggers[index.row()];

		if(role != Qt::DisplayRole)
			return QVariant();

		switch(index.column())
		{
			case COL_SHOW:
				return rqt_log_viewer::TRISTATE_INVALID;
			case COL_NAME:
				return logger.name;
			case COL_DEBUG:
				return logger.debug;
			case COL_LEVEL:
				return "";
		}
	}

	return QVariant();
}

bool NodeModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if(!index.isValid())
		return false;

	if(index.column() >= COL_COUNT)
		return false;

	if(role != Qt::EditRole)
		return false;

	if(index.internalId() == -1)
	{
		if(index.row() >= m_nodes.count())
			return false;

		Node* node = &m_nodes[index.row()];

		switch(index.column())
		{
			case COL_SHOW:
			{
				node->shown = value.toBool();

				dataChanged(index, index);
				filterDataChanged();
				return true;
			}
			case COL_DEBUG:
			{
				bool enable = value.toBool();

				Q_FOREACH(const Logger& logger, node->loggers)
					levelChangeRequested(node->name, logger.name, enable ? "DEBUG" : "INFO");

				return true;
			}
			case COL_LEVEL:
			{
				node->logLevel = value.toInt();

				dataChanged(index, index);
				filterDataChanged();
				return true;
			}
		}
	}
	else
	{
		Node* node = &m_nodes[index.internalId()];

		if(index.row() >= node->loggers.count())
			return false;

		Logger* logger = &node->loggers[index.row()];

		switch(index.column())
		{
			case COL_DEBUG:
				bool enable = value.toBool();

				levelChangeRequested(
					node->name,
					logger->name,
					enable ? "DEBUG" : "INFO"
				);

				return true;
		}
	}

	return false;
}

void NodeModel::processLogLevelChange(const QString& nodeName, const QString& logger, const QString& level)
{
	Node cmp;
	cmp.name = nodeName;

	QList<Node>::iterator it = qLowerBound(m_nodes.begin(), m_nodes.end(), cmp);
	if(it == m_nodes.end())
		return;

	QModelIndex nodeIndex = index(it - m_nodes.begin(), 0);
	Node* node = &(*it);

	Logger cmpl;
	cmpl.name = logger;
	QList<Logger>::iterator itl = qLowerBound(node->loggers.begin(), node->loggers.end(), cmpl);
	if(itl == node->loggers.end())
		return;

	itl->debug = (level.toUpper() == "DEBUG");
	QModelIndex idx = nodeIndex.child(itl - node->loggers.begin(), COL_DEBUG);
	dataChanged(idx, idx);

	node->updateFlags();
	idx = index(it - m_nodes.begin(), COL_DEBUG);
	dataChanged(idx, idx);

	idx = index(it - m_nodes.begin(), COL_SHOW);
	dataChanged(idx, idx);

	node->logLevel = ros::console::levels::Debug;
	idx = index(it - m_nodes.begin(), COL_LEVEL);
	dataChanged(idx, idx);
	filterDataChanged();
}

}
