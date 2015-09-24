// Single value plot
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "plotter/plot.h"
#include <plot_msgs/Plot.h>

#include <QtCore/QVariant>
#include <QtCore/QSettings>
#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <QtCore/QVector>
#include <QtCore/QDebug>
#include <QtGui/QPainter>
#include <boost/iterator/iterator_concepts.hpp>
#include <boost/foreach.hpp>
#include <ros/time.h>
#include <ros/console.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <cmath>

namespace plotter
{

Plot::Plot(const QString& name, Plot* parent)
 : QObject(parent)
 , m_buf(16384)
 , m_name(name)
 , m_settings(0)
 , m_enabled(false)
 , m_hasData(false)
 , m_color(Qt::red)
 , m_paused(false)
 , m_isEventChannel(false)
{
	if(parent)
		setParent(parent);
}

Plot::~Plot()
{
	if(parent())
	{
		Plot* p = ((Plot*)parent());
		p->m_children.removeOne(this);
		p->hierarchyChanged();
	}

	QList<Plot*> children = m_children;
	qDeleteAll(children);
}

class NameLessThan
{
public:
	bool operator()(Plot* a, Plot* b)
	{
		return a->name() < b->name();
	}
};

void Plot::setParent(Plot* parent)
{
	parent->addChild(this);
	setUsedSettings(parent->usedSettings());

	QObject::setParent(parent);
}

void Plot::addChild(Plot* plot)
{
	QList<Plot*>::iterator it = qLowerBound(m_children.begin(), m_children.end(), plot, NameLessThan());

	beginAddChild(this, it - m_children.begin());

	m_children.insert(it, plot);

	connect(plot, SIGNAL(changed(Plot*)), this, SIGNAL(changed(Plot*)));
	connect(plot, SIGNAL(beginAddChild(Plot*,int)), this, SIGNAL(beginAddChild(Plot*,int)));
	connect(plot, SIGNAL(childAdded(Plot*)), this, SIGNAL(childAdded(Plot*)));
	connect(plot, SIGNAL(hierarchyChanged()), this, SIGNAL(hierarchyChanged()));

	childAdded(this);
}

QVariant Plot::displayData(int role) const
{
	switch(role)
	{
		case Qt::DisplayRole:
			return m_name;
	}

	return QVariant();
}

void Plot::draw(QPainter* painter, const ros::Time& base, const QRectF& rect, bool dots) const
{
	painter->setPen(m_color);
	painter->setBrush(m_color);

	QPen pen(m_color);
	double scale_x = 1.0 / fabs(painter->transform().m11());
	double scale_y = 1.0 / fabs(painter->transform().m22());

	// Don't plot anything if this plot is not enabled
	if(!m_enabled)
		return;

	// Only need to draw something if this Plot element contains data
	if(hasData())
	{
		// Use a reverse iterator to draw backwards in time
		DataBuffer::const_reverse_iterator it  = m_buf.rbegin();
		DataBuffer::const_reverse_iterator end = m_buf.rend();
		
		// Ensure we have at least something to plot
		if(it != end)
		{
			// Handle the draw differently if this is a normal plot or an event
			if(!isEventChannel()) // Normal plot channel
			{
				// Retrieve the latest valid data point as the initialisation of lastPoint
				QPointF lastPoint((it->time - base).toSec(), it->value);
				while(!std::isfinite(lastPoint.x()) || !std::isfinite(lastPoint.y()))
				{
					it++;
					if(it == end) break;
					lastPoint.setX((it->time - base).toSec());
					lastPoint.setY(it->value);
				}

				// Reverse iterate through data points
				for(; it != end; ++it)
				{
					// Retrieve the current data point
					const DataPoint& data = *it;
					QPointF point((data.time - base).toSec(), data.value);
					
					// If the data point is not finite then ignore it
					if(!std::isfinite(point.x()) || !std::isfinite(point.y()))
						continue;

					// If the data point is off the right of the plotter window then don't bother drawing it
					if(point.x() > rect.right())
						continue;

					// Draw a line to the previous data point
					painter->setPen(pen);
					painter->drawLine(lastPoint, point);

					// Show plot point markers if required
					if(dots)
					{
						painter->setPen(QPen(Qt::transparent));
						painter->drawEllipse(point, 3.0 * scale_x, 3.0 * scale_y);
					}

					// Update the last point variable
					lastPoint = point;

					// If the data point was off the left of the plotter window then this was the last visible data point, so break
					if(point.x() < rect.left())
						break;
				}
			}
			else // Event channel
			{
				// Obtain the painter transform and font height
				QTransform trans = painter->transform();
				double fs = painter->fontMetrics().height();

				// Reverse iterate through occurrences of the event
				for(; it != end; ++it)
				{
					// Retrieve the time of the event occurrence
					const DataPoint& data = *it;
					double time = (data.time - base).toSec();

					// Ignore occurrence if it is off the right of the plot window
					if(time > rect.right())
						continue;

					// Calculate the bottom point of the event occurrence line
					QPointF bottomPoint(time, rect.bottom() + 3.0 * scale_y * fs);

					// Draw the event occurrence line
					painter->setPen(pen);
					painter->drawLine(
						QPointF(time, rect.top()),
						bottomPoint
					);

					// Draw the event occurrence label
					QPointF devCoord = painter->transform().map(bottomPoint);
					double w = painter->fontMetrics().width(name());
					QRectF textRect(
						devCoord.x() - 0.5*w,
						devCoord.y() + data.labelY * fs,
						w, fs);
					painter->setTransform(QTransform());
					painter->drawText(textRect, Qt::AlignHCenter | Qt::AlignVCenter, name());
					painter->setTransform(trans);

					// If the event occurrence was off the left of the plotter window then this was the last visible occurrence, so break
					if(time < rect.left())
						break;
				}
			}
		}
	}

	// Draw all child plots
	for(int i = 0; i < m_children.size(); ++i)
		m_children[i]->draw(painter, base, rect, dots);
}

void Plot::setEnabled(bool enabled)
{
	m_enabled = enabled;
	changed(this);
}

void Plot::setName(const QString& name)
{
	m_name = name;
	changed(this);
}

void Plot::setColor(const QColor& color)
{
	m_color = color;
	changed(this);
}

void Plot::setPaused(bool paused)
{
	m_paused = paused;
	for(int i = 0; i < m_children.size(); ++i)
		m_children[i]->setPaused(paused);
}

void Plot::setIsEventChannel(bool eventChannel)
{
	m_isEventChannel = eventChannel;
}

void Plot::put(const ros::Time& time, double value, unsigned int labelY, bool notify)
{
	if(m_paused)
		return;

	m_hasData = true;
	m_buf.push_back(DataPoint(time, value, labelY));

	if(notify)
		changed(this);
}

Plot* Plot::findPlotByPath(const QString& path) const
{
	QString local = path.section('/', 0, 0, QString::SectionSkipEmpty);
	QString subPath = path.section('/', 1, -1, QString::SectionSkipEmpty);

	for(int i = 0; i < m_children.size(); ++i)
	{
		if(m_children[i]->name() == local)
		{
			if(subPath.isEmpty())
				return m_children[i];
			else
				return m_children[i]->findPlotByPath(subPath);
		}
	}

	return 0;
}

Plot* Plot::findOrCreatePlotByPath(const QString& path)
{
	QString local = path.section('/', 0, 0, QString::SectionSkipEmpty);
	QString subPath = path.section('/', 1, -1, QString::SectionSkipEmpty);

	for(int i = 0; i < m_children.size(); ++i)
	{
		if(m_children[i]->name() == local)
		{
			if(subPath.isEmpty())
				return m_children[i];
			else
				return m_children[i]->findOrCreatePlotByPath(subPath);
		}
	}

	Plot* plot = new Plot(local, this);
	plot->setPaused(m_paused);

	if(subPath.isEmpty())
		return plot;
	else
		return plot->findOrCreatePlotByPath(subPath);
}


QString Plot::path() const
{
	QString p = m_name;

	for(Plot* plot = (Plot*)parent(); plot->parent(); plot = (Plot*)plot->parent())
		p.prepend(plot->name() + '/');

	return p;
}

ros::Time Plot::lastTime() const
{
	if(m_buf.size() == 0)
		return ros::Time();

	return (*m_buf.rbegin()).time;
}

ros::Time Plot::recursiveLastTime() const
{
	ros::Time time = lastTime();

	for(int i = 0; i < m_children.size(); ++i)
	{
		ros::Time cTime = m_children[i]->recursiveLastTime();

		if(cTime > time)
			time = cTime;
	}

	return time;
}

void Plot::setUsedSettings(QSettings* settings)
{
	if(!settings)
		return;

	if(parent())
	{
		settings->beginGroup(path());
		m_enabled = settings->value("enabled", m_enabled).toBool();
		m_color.setNamedColor(settings->value("color", m_color.name()).toString());
		settings->endGroup();
	}

	m_settings = settings;

	for(int i = 0; i < m_children.size(); ++i)
		m_children[i]->setUsedSettings(settings);
}

void Plot::serialize()
{
	if(!m_settings)
		return;

	if(parent())
	{
		m_settings->beginGroup(path());
		m_settings->setValue("enabled", m_enabled);
		m_settings->setValue("color", m_color.name());
		m_settings->endGroup();
	}

	for(int i = 0; i < m_children.size(); ++i)
		m_children[i]->serialize();
}

double Plot::value(const ros::Time& time) const
{
	DataPoint cmp;
	cmp.time = time;

	DataBuffer::const_iterator it = std::lower_bound(m_buf.begin(), m_buf.end(), cmp);

	if(it == m_buf.end())
		return NAN;

	return (*it).value;
}

QLinkedList< Plot::LinkedBufferIterator > Plot::iterators(FindFlags flags) const
{
	QLinkedList<Plot::LinkedBufferIterator> ret;

	if((flags & FindOnlyVisible) && !m_enabled)
		return ret;

	if(m_hasData)
		ret << LinkedBufferIterator(m_buf.begin(), this);

	if(flags & FindRecursive)
	{
		Q_FOREACH(Plot* child, m_children)
			ret += child->iterators(flags);
	}

	return ret;
}

Plot::LinkedBufferIterator Plot::findDataPointAt(const ros::Time& time, double value, FindFlags flags)
{
	DataPoint p(time, value);

	LinkedBufferIterator ret;

	if(flags & FindOnlyVisible && !isEnabled())
		return ret;

	// binary search for time
	// The point behind the time
	DataBuffer::const_iterator it = std::lower_bound(m_buf.begin(), m_buf.end(), p);
	DataBuffer::const_iterator it_before;

	if(it != m_buf.end())
	{
		ret = LinkedBufferIterator(it, this);

		if(it != m_buf.begin())
		{
			it_before = it - 1;
			ros::Duration dist1 = (*it).time - time;
			ros::Duration dist2 = time - (*it_before).time;

			if(dist2 < dist1)
				ret = LinkedBufferIterator(it_before, this);
		}
	}

	if(flags & FindRecursive)
	{
		Q_FOREACH(Plot* child, m_children)
		{
			LinkedBufferIterator it = child->findDataPointAt(time, value, flags);
			if(!it.isValid())
				continue;

			if(!ret.isValid()
				|| fabs(((*it).time - time).toSec()) < fabs(((*ret).time - time).toSec()))
			{
				ret = it;
			}
		}
	}

	return ret;
}


}
