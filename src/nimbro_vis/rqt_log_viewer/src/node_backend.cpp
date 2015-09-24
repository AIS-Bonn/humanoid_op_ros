// QObject handling all node interaction
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "node_backend.h"

#include <QtCore/QTimer>

#include <ros/master.h>
#include <ros/service.h>
#include <ros/this_node.h>

#include <roscpp/GetLoggers.h>
#include <roscpp/SetLoggerLevel.h>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

namespace rqt_log_viewer
{

void Node::updateFlags()
{
	debug = TRISTATE_INVALID;
	shown = TRISTATE_INVALID;

	Q_FOREACH(const Logger& logger, loggers)
	{
		if(debug == TRISTATE_INVALID)
			debug = logger.debug ? TRISTATE_TRUE : TRISTATE_FALSE;
		else if(logger.debug && debug == TRISTATE_FALSE)
			debug = TRISTATE_TRISTATE;
		else if(!logger.debug && debug == TRISTATE_TRUE)
			debug = TRISTATE_TRISTATE;
	}
}


NodeBackend::NodeBackend(QObject* parent)
 : QObject(parent)
{
	qRegisterMetaType<rqt_log_viewer::NodeList>();

	// Discover nodes periodically
	QTimer* timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), SLOT(discoverNodes()));
	timer->start(2000);
}

NodeBackend::~NodeBackend()
{
}

void NodeBackend::discoverNodes()
{
	ros::V_string nodes;
	ros::master::getNodes(nodes);

	QList<Node> results;

	BOOST_FOREACH(const std::string& str, nodes)
	{
		// HACK: Ignore some known uninteresting nodes
		if(str == "/rosout" || str.substr(0, 5) == "/rqt_" || str == ros::this_node::getName())
			continue;

		Node node;
		node.name = QString::fromStdString(str);
		node.shown = true;
		node.debug = TRISTATE_INVALID;
		node.logLevel = ros::console::levels::Info;

		roscpp::GetLoggersRequest req;
		roscpp::GetLoggersResponse resp;
		if(!ros::service::call(str + "/get_loggers", req, resp))
		{
			// silently ignore errors, they are mostly caused by startup delays.
			continue;
		}

		BOOST_FOREACH(const roscpp::Logger& msg_logger, resp.loggers)
		{
			// HACK: Ignore some common system loggers
			if(msg_logger.name.substr(0, 5) == "rospy"
				|| msg_logger.name.substr(0, 10) == "ros.roscpp"
				|| msg_logger.name.substr(0, 8) == "rosgraph"
				|| msg_logger.name == "xmlrpc"
				|| msg_logger.name == "ros")
			{
				continue;
			}

			Logger logger;
			logger.name = QString::fromStdString(msg_logger.name);
			logger.debug = (boost::to_upper_copy(msg_logger.level) == "DEBUG");

			QList<Logger>::iterator it;
			it = qLowerBound(node.loggers.begin(), node.loggers.end(), logger);
			node.loggers.insert(it, logger);
		}

		node.updateFlags();

		QList<Node>::iterator it = qLowerBound(results.begin(), results.end(), node);
		results.insert(it, node);
	}

	nodeListChanged(results);
}

void NodeBackend::changeLogLevel(const QString& nodeName, const QString& logger, const QString& level)
{
	roscpp::SetLoggerLevelRequest req;
	roscpp::SetLoggerLevelResponse resp;

	req.logger = logger.toStdString();
	req.level = level.toStdString();

	if(!ros::service::call(nodeName.toStdString() + "/set_logger_level", req, resp))
	{
		ROS_WARN("Could not set logger '%s' level of node '%s' to '%s'",
			req.logger.c_str(), qPrintable(logger), req.level.c_str()
		);
		return;
	}

	logLevelChanged(nodeName, logger, level);
}

}
