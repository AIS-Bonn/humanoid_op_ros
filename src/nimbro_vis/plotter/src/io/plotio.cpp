// Provides file serialization for plots
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <plotter/io/plotio.h>
#include <plotter/io/fileformat.h>
#include <plotter/io/bagformat.h>
#include <plotter/io/csvformat.h>
#include <plotter/plot.h>
#include <plotter/plotter.h>
#include <plot_msgs/Plot.h>

#include <QtCore/QHash>
#include <QtCore/QVector>

#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace plotter
{

PlotIO::PlotIO(Plot* rootPlot, QObject* parent)
 : QObject(parent)
 , m_rootPlot(rootPlot)
{
}

PlotIO::~PlotIO()
{
}

bool PlotIO::read(const QString& path)
{
	rosbag::Bag bag;

	try
	{
		bag.open(path.toStdString(), rosbag::bagmode::Read);
	}
	catch(rosbag::BagException& e)
	{
		fprintf(stderr, "Could not open bag file: %s\n", e.what());
		return false;
	}

	std::vector<std::string> topics;
	topics.push_back("/plot");
	topics.push_back("plot"); // For legacy bag support

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	ros::Time begin = view.getBeginTime();
	ros::Time end = view.getEndTime();
	double duration = (end - begin).toSec();

	ros::Time start(0, 0);
	ros::Time stop(0, 0);

	QList<Plot*> childrenList = m_rootPlot->childPlots();
	qDeleteAll(childrenList);

	QHash<QString, Plot*> plotMap;

	int progressCounter = 0;
	unsigned int labelY = 0;
	for(rosbag::View::iterator it = view.begin(); it != view.end(); ++it)
	{
		plot_msgs::Plot::ConstPtr msg = it->instantiate<plot_msgs::Plot>();
		if(!msg)
			continue;
		
		ros::Time msgTime = it->getTime();

		for(size_t i = 0; i < msg->points.size(); ++i)
		{
			QString path = QString::fromStdString(msg->points[i].name);
			QHash<QString, Plot*>::iterator it = plotMap.find(path);

			Plot* plot;
			if(it != plotMap.end())
				plot = it.value();
			else
			{
				plot = m_rootPlot->findOrCreatePlotByPath(QString::fromStdString(msg->points[i].name));
				plot->setPaused(false);
				plotMap.insert(path, plot);
			}

			plot->put(msg->header.stamp, msg->points[i].value, 0, false);
		}
		
		for(size_t i = 0; i < msg->events.size(); ++i)
		{
			QString name = QString::fromStdString(msg->events[i]);
			if(name.isEmpty()) continue;
			Plot* plot = m_rootPlot->findPlotByPath(name);
			if(plot && !plot->isEventChannel()) continue;
			if(!plot) plot = m_rootPlot->findOrCreatePlotByPath(name);
			plot->setIsEventChannel(true);
			plot->put(msg->header.stamp, NAN, labelY);
			labelY = (labelY + 1) % 2;
		}

		if(++progressCounter == 20)
		{
			progress((msgTime - begin).toSec() / duration);
			progressCounter = 0;
		}

		if(start.isZero() || msgTime < start)
			start = msgTime;
		if(stop.isZero() || msgTime > stop)
			stop = msgTime;
	}

	m_rootPlot->setPaused(true);
	
	ROS_INFO("Loaded plotter data of time [%.3f,%.3f] from '%s'", start.toSec(), stop.toSec(), path.toLocal8Bit().data());

	Plotter::loadTimeWarp(path.toStdString());

	return true;
}

bool PlotIO::write(const QString& path, const ros::Time& start, const ros::Time& end)
{
	FileFormat* io = 0;
	
	bool isBag = path.endsWith(".bag");
	bool isCsv = path.endsWith(".csv");

	if(isBag)
		io = new BagFormat;
	else if(isCsv)
		io = new CSVFormat;

	if(!io)
	{
		fprintf(stderr, "Could not find output format\n");
		return false;
	}

	if(!io->init(path, start))
	{
		delete io;
		return false;
	}

	QLinkedList<Plot::LinkedBufferIterator> its = m_rootPlot->iterators(io->defaultFlags());

	io->writeHeader(its);

	QVector<Plot::LinkedBufferIterator*> to_write(its.size(), 0);

	while(1)
	{
		ros::Time stamp = ros::Time(0);
		qFill(to_write, (Plot::LinkedBufferIterator*)0);

		// Get a list of iterators with the next timestamp
		int i = 0;
		for(QLinkedList<Plot::LinkedBufferIterator>::iterator it = its.begin(); it != its.end(); ++it, ++i)
		{
			Plot::LinkedBufferIterator& buf_it = *it;
			if(!buf_it.isValid())
				continue;

			const Plot::DataPoint& point = *buf_it;

			if(point.time < stamp || stamp == ros::Time(0))
			{
				qFill(to_write, (Plot::LinkedBufferIterator*)0);
				to_write[i] = &buf_it;
				stamp = point.time;
			}
			else if(point.time == stamp)
				to_write[i] = &buf_it;
		}

		if(stamp == ros::Time(0))
			break;

		if(end != ros::Time(0) && stamp > end)
			break;

		if(stamp >= start)
			io->writeData(stamp, to_write);

		// Advance all used iterators
		Q_FOREACH(Plot::LinkedBufferIterator* it, to_write)
		{
			if(!it)
				continue;

			++(*it);
		}
	}

	delete io;

	ROS_INFO("Saved plotter data from time [%.3f,%.3f] to '%s'", start.toSec(), end.toSec(), path.toLocal8Bit().data());

	if(isBag)
		Plotter::saveTimeWarp(path.toStdString(), start, end);
	
	return true;
}

}