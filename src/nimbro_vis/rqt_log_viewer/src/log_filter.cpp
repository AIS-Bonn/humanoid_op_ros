// Qt proxy model filtering the log according to the user choices
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "log_filter.h"

#include "log_model.h"

#include <stdio.h>

#include <ros/console.h>

namespace rqt_log_viewer
{

LogFilter::LogFilter(NodeModel* nodeModel, QObject* parent)
 : QSortFilterProxyModel(parent)
 , m_nodeModel(nodeModel)
{
	connect(m_nodeModel, SIGNAL(filterDataChanged()),
		SLOT(updateFilter())
	);

	setFilterCaseSensitivity(Qt::CaseInsensitive);
	setFilterKeyColumn(LogModel::COL_TEXT);
}

LogFilter::~LogFilter()
{
}

void LogFilter::updateFilter()
{
	m_nodes.clear();

	for(int i = 0; i < m_nodeModel->rowCount(); ++i)
	{
		QModelIndex idx = m_nodeModel->index(i, NodeModel::COL_SHOW);
		bool show = idx.data().toBool();

		idx = m_nodeModel->index(i, NodeModel::COL_NAME);
		std::string name = m_nodeModel->data(idx).toString().toStdString();

		NodeFilterInfo info;
		info.name = name;

		if(show)
		{
			int value = m_nodeModel->index(i, NodeModel::COL_LEVEL).data(Qt::EditRole).toInt();
			ros::console::Level logLevel = (ros::console::Level)value;

			switch(logLevel)
			{
				case ros::console::levels::Debug:
					info.logLevel = rosgraph_msgs::Log::DEBUG;
					break;
				case ros::console::levels::Info:
					info.logLevel = rosgraph_msgs::Log::INFO;
					break;
				case ros::console::levels::Warn:
					info.logLevel = rosgraph_msgs::Log::WARN;
					break;
				case ros::console::levels::Error:
					info.logLevel = rosgraph_msgs::Log::ERROR;
					break;
				case ros::console::levels::Fatal:
					info.logLevel = rosgraph_msgs::Log::FATAL;
					break;
				default:
					break;
			}
		}
		else
			info.logLevel = std::numeric_limits<int>::max();

		std::vector<NodeFilterInfo>::iterator it;
		it = std::lower_bound(m_nodes.begin(), m_nodes.end(), info);

		m_nodes.insert(it, info);
	}

	invalidateFilter();
}

bool LogFilter::filterAcceptsRow(int source_row, const QModelIndex& source_parent) const
{
	void *ptr = sourceModel()->data(sourceModel()->index(source_row, 0), LogModel::ROLE_PTR).value<void*>();
	assert(ptr);

	const rosgraph_msgs::Log& msg = *reinterpret_cast<rosgraph_msgs::Log*>(ptr);

	// Use binary search to check if msg.name is in m_nodes
	std::vector<NodeFilterInfo>::const_iterator it;
	it = std::lower_bound(m_nodes.begin(), m_nodes.end(), msg.name);

	// If we know about the node, perform level filtering
	if(it != m_nodes.end() && it->name == msg.name)
	{
		if(msg.level < it->logLevel)
			return false;
	}
	else
	{
		// perform default filtering
		if(msg.level < ros::console::levels::Info)
			return false;
	}

	return QSortFilterProxyModel::filterAcceptsRow(source_row, source_parent);
}

}
