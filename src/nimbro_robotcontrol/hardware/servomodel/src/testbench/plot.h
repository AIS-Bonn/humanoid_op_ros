// Plot manager
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PLOT_H
#define PLOT_H

#include <QObject>

class ValueLogger;
class QPainter;
class QSettings;

class Plot : public QObject
{
Q_OBJECT
public:
	explicit Plot(QObject* parent = 0);
	virtual ~Plot();

	ValueLogger* addLogger(const QString& name);

	const QList<ValueLogger*>& loggers() const
	{ return m_loggers; }

	void serialize(QSettings* settings) const;
	void deserialize(QSettings* settings);
public Q_SLOTS:
	void reset();
	void paint(QPainter* painter);
Q_SIGNALS:
	void changed();
private:
	QList<ValueLogger*> m_loggers;
};

#endif
