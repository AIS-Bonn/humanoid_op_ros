// Qt proxy model filtering the log according to the user choices
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef LOG_FILTER_H
#define LOG_FILTER_H

#include <QtGui/QSortFilterProxyModel>

#include <vector>

#include "node_model.h"

namespace rqt_log_viewer
{

class LogFilter : public QSortFilterProxyModel
{
Q_OBJECT
public:
	LogFilter(NodeModel* nodeModel, QObject* parent = 0);
	virtual ~LogFilter();

	virtual bool filterAcceptsRow(int source_row, const QModelIndex& source_parent) const;
private Q_SLOTS:
	void updateFilter();
private:
	NodeModel* m_nodeModel;

	struct NodeFilterInfo
	{
		std::string name;
		int logLevel;

		bool operator<(const NodeFilterInfo& other) const
		{ return name < other.name; }

		bool operator<(const std::string& other) const
		{ return name < other; }
	};

	std::vector<NodeFilterInfo> m_nodes;
};

}

#endif
