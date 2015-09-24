// Value logger class
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef VALUELOGGER_H
#define VALUELOGGER_H

#include <QtCore/QObject>
#include <QtCore/QVector>
#include <QtGui/QPen>

class QPainter;

class ValueLogger : public QObject
{
Q_OBJECT
public:
	class const_iterator
	{
	friend class ValueLogger;
	public:
		double time() const;
		double value() const;
		bool isValid() const;
		int operator++();
	private:
		const_iterator(const ValueLogger* logger);

		const ValueLogger* m_logger;
		int m_idx;
	};

	explicit ValueLogger(QObject* parent = 0);
	virtual ~ValueLogger();

	void log(double time, double value);
	void paint(QPainter* painter);
	void reset();

	void setPen(const QPen& pen);
	inline const QPen& pen() const
	{ return m_pen; }

	void setColor(const QColor& color);

	void setName(const QString& name);
	inline QString name() const
	{ return m_name; }

	inline bool visible() const
	{ return m_visible; }

	double valueAtTime(double time) const;

	inline int count() const
	{ return m_entries.count(); }

	const_iterator begin() const;
public Q_SLOTS:
	void setVisible(bool visible);
Q_SIGNALS:
	void changed();
private:
	struct LogEntry
	{
		LogEntry()
		{}

		LogEntry(double _time, double _value)
		 : time(_time), value(_value)
		{}

		bool operator<(const LogEntry& other) const;

		double time;
		double value;
	};

	QVector<LogEntry> m_entries;
	QPen m_pen;
	QString m_name;
	double m_lastEntryTime;
	bool m_visible;
};

#endif
