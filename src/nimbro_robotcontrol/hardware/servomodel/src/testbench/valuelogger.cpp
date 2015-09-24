// Value logger class
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "valuelogger.h"

#include <QtCore/QDebug>
#include <QtGui/QPainter>

bool ValueLogger::LogEntry::operator<(const ValueLogger::LogEntry& other) const
{
	return time < other.time;
}

ValueLogger::const_iterator::const_iterator(const ValueLogger* logger)
 : m_logger(logger)
 , m_idx(0)
{
}

bool ValueLogger::const_iterator::isValid() const
{
	return m_idx < m_logger->m_entries.count();
}

int ValueLogger::const_iterator::operator++()
{
	m_idx++;
}

double ValueLogger::const_iterator::time() const
{
	return m_logger->m_entries[m_idx].time;
}

double ValueLogger::const_iterator::value() const
{
	return m_logger->m_entries[m_idx].value;
}

ValueLogger::ValueLogger(QObject* parent)
 : QObject(parent)
 , m_lastEntryTime(-1)
 , m_pen(Qt::red)
 , m_visible(true)
{
}

ValueLogger::~ValueLogger()
{
}

void ValueLogger::log(double time, double value)
{
	if(time < m_lastEntryTime)
	{
		qFatal("ValueLogger '%s': Non-monotonic entry time: %lf < %lf", m_name.toAscii().constData(), time, m_lastEntryTime);
		return;
	}

	m_entries.append(LogEntry(time, value));

	m_lastEntryTime = time;
}

void ValueLogger::paint(QPainter* painter)
{
	if(!m_visible)
		return;

	painter->setPen(m_pen);
	for(int i = 1; i < m_entries.size(); ++i)
	{
		const LogEntry& a = m_entries[i-1];
		const LogEntry& b = m_entries[i];
		painter->drawLine(QPointF(a.time, a.value), QPointF(b.time, b.value));
	}
}

void ValueLogger::reset()
{
	m_entries.clear();
	m_lastEntryTime = -1;
}

void ValueLogger::setPen(const QPen& pen)
{
	m_pen = pen;
	changed();
}

void ValueLogger::setColor(const QColor& color)
{
	m_pen.setColor(color);
	changed();
}

void ValueLogger::setVisible(bool visible)
{
	m_visible = visible;
	changed();
}

void ValueLogger::setName(const QString& name)
{
	m_name = name;
}

double ValueLogger::valueAtTime(double time) const
{
	LogEntry cmp(time, 0);
	QVector<LogEntry>::const_iterator it = qLowerBound(m_entries.begin(), m_entries.end(), cmp);

	if(it == m_entries.end())
	{
		if(m_entries.empty())
			return 0;
		else
			return m_entries[m_entries.count()-1].value;
	}

	return it->value;
}

ValueLogger::const_iterator ValueLogger::begin() const
{
	return const_iterator(this);
}

