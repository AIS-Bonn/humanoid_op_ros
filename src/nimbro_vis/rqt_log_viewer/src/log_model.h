// Qt Model for a log stream
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef LOG_MODEL_H
#define LOG_MODEL_H

#include <QtCore/QAbstractTableModel>

#include <rosgraph_msgs/Log.h>

#include <QList>

class QTimer;

namespace rqt_log_viewer
{

class LogModel : public QAbstractTableModel
{
Q_OBJECT
public:
	enum Column
	{
		COL_TIME,
		COL_RELATIVE_TIME,
		COL_SOURCE,
		COL_TEXT,

		COL_COUNT
	};

	enum Role
	{
		ROLE_PTR = Qt::UserRole + 256, //!< Get the raw rosgraph_msgs::Log*
	};

	explicit LogModel(QObject* parent = 0);
	virtual ~LogModel();

	virtual QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;
	virtual QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;

	virtual int rowCount(const QModelIndex& parent = QModelIndex()) const;
	virtual int columnCount(const QModelIndex& parent = QModelIndex()) const;

	inline QList<rosgraph_msgs::Log>::const_iterator begin() const
	{ return m_messages.begin(); }

	inline QList<rosgraph_msgs::Log>::const_iterator end() const
	{ return m_messages.end(); }

	inline ros::Time startTime() const
	{ return m_startTime; }
public Q_SLOTS:
	void addMessage(const rosgraph_msgs::Log& msg);
	void prune();
	void clear();
private:
	void insertMessage(int idx, const rosgraph_msgs::Log& msg);

	QList<rosgraph_msgs::Log> m_messages;
	int m_insertionPoint;

	QTimer* m_pruneTimer;

	ros::Time m_startTime;
};

}

#endif
