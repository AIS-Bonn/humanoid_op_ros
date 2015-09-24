// Qt model for all running nodes, exposing log level settings
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef NODE_MODEL_H
#define NODE_MODEL_H

#include <QtCore/QAbstractTableModel>
#include <QtCore/QStringList>
#include <QtCore/QThread>

#include <qt_gui_cpp/settings.h>

#include "node_backend.h"

namespace rqt_log_viewer
{

class NodeModel : public QAbstractItemModel
{
Q_OBJECT
public:
	enum Column
	{
		COL_SHOW,
		COL_NAME,
		COL_DEBUG,
		COL_LEVEL,

		COL_COUNT
	};

	explicit NodeModel(QObject* parent = 0);
	virtual ~NodeModel();

	virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const;
	virtual QModelIndex parent(const QModelIndex& child) const;

	virtual int rowCount(const QModelIndex& parent = QModelIndex()) const;
	virtual int columnCount(const QModelIndex& parent = QModelIndex()) const;

	virtual QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;

	virtual QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;
	virtual bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole);
	virtual Qt::ItemFlags flags(const QModelIndex& index) const;

	void stopBackend();

	void saveConfiguration(qt_gui_cpp::Settings& settings);
	void restoreConfiguration(qt_gui_cpp::Settings& settings);
Q_SIGNALS:
	void filterDataChanged();
	void levelChangeRequested(const QString& node, const QString& logger, const QString& level);
private Q_SLOTS:
	void processNodeList(const rqt_log_viewer::NodeList& nodes);
	void processLogLevelChange(const QString& node, const QString& logger, const QString& level);
private:
	void updateNode(int idx, const Node& source);

	QList<Node> m_nodes;

	QThread m_thread;
	NodeBackend* m_backend;

	QStringList m_logLevelLabels;
};

};

#endif
