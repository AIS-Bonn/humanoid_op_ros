// QObject handling all node interaction
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef NODE_BACKEND_H
#define NODE_BACKEND_H

#include <QtCore/QObject>
#include <QtCore/QMetaClassInfo>

#include "tristate.h"

namespace rqt_log_viewer
{


struct Logger
{
	QString name;
	bool debug;

	inline bool operator<(const Logger& other) const
	{ return name < other.name; }
};

struct Node
{
	bool shown;
	QString name;
	Tristate debug;
	int logLevel;

	QList<Logger> loggers;

	void updateFlags();

	inline bool operator<(const Node& other) const
	{ return name < other.name; }
};

typedef QList<Node> NodeList;

class NodeBackend : public QObject
{
Q_OBJECT
public:
	explicit NodeBackend(QObject* parent = 0);
	virtual ~NodeBackend();

Q_SIGNALS:
	void nodeListChanged(const rqt_log_viewer::NodeList& nodeList);
	void logLevelChanged(const QString& nodeName, const QString& logger, const QString& level);
public Q_SLOTS:
	void discoverNodes();
	void changeLogLevel(const QString& nodeName, const QString& logger, const QString& level);
};

}

Q_DECLARE_METATYPE(rqt_log_viewer::NodeList)

#endif
