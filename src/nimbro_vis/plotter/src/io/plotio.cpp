// Provides file serialization for plots
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <plotter/io/plotio.h>
#include <plotter/io/fileformat.h>
#include <plotter/io/bagformat.h>
#include <plotter/io/csvformat.h>
#include <plotter/plot.h>
#include <plotter/plotter.h>
#include <plot_msgs/Plot.h>
#include <plot_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>

#include <QtCore/QHash>
#include <QtCore/QVector>

#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace plotter
{

PlotIO::PlotIO(ros::NodeHandle& nh, Plot* rootPlot, QObject* parent)
 : QObject(parent)
 , m_nh(nh)
 , m_rootPlot(rootPlot)
{
}

PlotIO::~PlotIO()
{
}

// Read plot data from a bag
bool PlotIO::read(const QString& path)
{
	// Attempt to open the bag
	rosbag::Bag bag;
	try { bag.open(path.toStdString(), rosbag::bagmode::Read); }
	catch(rosbag::BagException& e)
	{
		fprintf(stderr, "Could not open bag file: %s\n", e.what());
		return false;
	}

	// Get a view into the bag of the required topics
	rosbag::View view(bag, rosbag::TopicQuery("/plot"));

	// Ensure the root plot is paused (this is recursive and inherited by any new children, and is NOT optional)
	m_rootPlot->setPaused(true);

	// Clear out all the existing plots from the plotter
	QList<Plot*> childrenList = m_rootPlot->childPlots();
	qDeleteAll(childrenList);
	JointStatePlot* jointPlot = new JointStatePlot(m_nh, m_rootPlot); // The created JointStatePlot gets deleted when the root plot does, or all of its child plots

	// Clear all current timewarp data
	Plotter::clearTimewarp();

	// Initialise variables for the loop
	QHash<QString, Plot*> plotMap; // This establishes a map of the plot topics (of both points and events type) that have been added (more efficient than always checking inside the root plot)
	int progressCounter = 0;
	unsigned int labelY = 0;
	ros::Time start(0, 0);
	ros::Time stop(0, 0);
	Plot* plot = NULL;
	uint32_t viewSize, viewCount;

	// Loop through and process all relevant messages in the bag
	bool haveJointStateData = false;
	viewSize = view.size();
	viewCount = 0;
	for(rosbag::View::iterator it = view.begin(); it != view.end(); ++it, ++viewCount)
	{
		// Update the progress bar
		if(++progressCounter == 20)
		{
			progress(((double) viewCount) / viewSize);
			progressCounter = 0;
		}

		// Retrieve the message and its time stamp
		plot_msgs::PlotConstPtr msg = it->instantiate<plot_msgs::Plot>();
		if(!msg) continue;
		ros::Time msgTime = msg->header.stamp;

		// Process the plot points contained in the message
		for(std::size_t i = 0; i < msg->points.size(); ++i)
		{
			QString path = QString::fromStdString(msg->points[i].name);
			if(path.isEmpty()) continue;
			if(path[0] != '/')
				path.insert(0, '/'); // Standardised naming for the plot map...
			QHash<QString, Plot*>::iterator it = plotMap.find(path);
			if(it != plotMap.end())
				plot = it.value();
			else
			{
				plot = m_rootPlot->findOrCreatePlotByPath(path);
				plotMap.insert(path, plot);
			}
			if(plot->isEventChannel()) // Plot points have priority over plot events...
			{
				plot->clearData(false);
				plot->setIsEventChannel(false);
			}
			haveJointStateData |= path.startsWith(JS_PLOT_TOPIC_S);
			plot->put(msgTime, msg->points[i].value, NULL, false, true);
		}

		// Process the plot events contained in the message
		for(std::size_t i = 0; i < msg->events.size(); ++i)
		{
			QString path = QString::fromStdString(msg->events[i]);
			if(path.isEmpty()) continue;
			if(path[0] != '/')
				path.insert(0, '/'); // Standardised naming for the plot map...
			QHash<QString, Plot*>::iterator it = plotMap.find(path);
			if(it != plotMap.end())
				plot = it.value();
			else
			{
				plot = m_rootPlot->findOrCreatePlotByPath(path);
				if(!plot->hasData())
					plot->setIsEventChannel(true);
				plotMap.insert(path, plot);
			}
			if(plot->isEventChannel()) // Plot points have priority over plot events...
			{
				haveJointStateData |= path.startsWith(JS_PLOT_TOPIC_S);
				plot->put(msgTime, NAN, &labelY, false, true);
			}
		}

		// Update the start and stop time of the loaded data (for info purposes only)
		if(start.isZero() || msgTime < start)
			start = msgTime;
		if(stop.isZero() || msgTime > stop)
			stop = msgTime;
	}

	// If the plot messages did not include joint states and commands, then read them in from separate topics if possible
	if(!haveJointStateData)
	{
		// Retrieve any joint states contained in the bag
		rosbag::View* jointStateView = new rosbag::View(bag, rosbag::TopicQuery("/joint_states"));
		viewSize = jointStateView->size();
		viewCount = 0;
		if(viewSize == 0)
		{
			delete jointStateView;
			jointStateView = new rosbag::View(bag, rosbag::TopicQuery("/vis/joint_states"));
		}
		for(rosbag::View::iterator it = jointStateView->begin(); it != jointStateView->end(); ++it, ++viewCount)
		{
			// Update the progress bar
			if(++progressCounter == 20)
			{
				progress(((double) viewCount) / viewSize);
				progressCounter = 0;
			}

			// Retrieve the message and its time stamp
			sensor_msgs::JointStateConstPtr msg = it->instantiate<sensor_msgs::JointState>();
			if(!msg) continue;
			ros::Time msgTime = msg->header.stamp;

			// Add the message data to the plot
			jointPlot->handleData(msg, true);

			// Update the start and stop time of the loaded data (for info purposes only)
			if(start.isZero() || msgTime < start)
				start = msgTime;
			if(stop.isZero() || msgTime > stop)
				stop = msgTime;
		}
		delete jointStateView;

		// Retrieve any joint commands contained in the bag
		rosbag::View* jointCommandView = new rosbag::View(bag, rosbag::TopicQuery("/joint_commands"));
		viewSize = jointCommandView->size();
		viewCount = 0;
		if(viewSize == 0)
		{
			delete jointCommandView;
			jointCommandView = new rosbag::View(bag, rosbag::TopicQuery("/vis/joint_commands"));
		}
		for(rosbag::View::iterator it = jointCommandView->begin(); it != jointCommandView->end(); ++it, ++viewCount)
		{
			// Update the progress bar
			if(++progressCounter == 20)
			{
				progress(((double) viewCount) / viewSize);
				progressCounter = 0;
			}

			// Retrieve the message and its time stamp
			plot_msgs::JointCommandConstPtr msg = it->instantiate<plot_msgs::JointCommand>();
			if(!msg) continue;
			ros::Time msgTime = msg->header.stamp;

			// Add the message data to the plot
			jointPlot->handleCmdData(msg, true);

			// Update the start and stop time of the loaded data (for info purposes only)
			if(start.isZero() || msgTime < start)
				start = msgTime;
			if(stop.isZero() || msgTime > stop)
				stop = msgTime;
		}
		delete jointCommandView;
	}

	// Inform the user about the plot data that was loaded
	ROS_INFO("Loaded plotter data of time [%.3f,%.3f] and duration %.3fs from '%s'", start.toSec(), stop.toSec(), (stop - start).toSec(), path.toLocal8Bit().data());

	// Trigger the timewarp node to also load data from the bag
	Plotter::loadTimeWarp(path.toStdString());

	// Return success
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

	ros::Time earliest(0, 0);
	ros::Time latest(0, 0);

	while(1)
	{
		ros::Time stamp = ros::Time();
		qFill(to_write, (Plot::LinkedBufferIterator*)0);

		// Get a list of iterators with the next timestamp
		int i = 0;
		for(QLinkedList<Plot::LinkedBufferIterator>::iterator it = its.begin(); it != its.end(); ++it, ++i)
		{
			Plot::LinkedBufferIterator& buf_it = *it;
			if(!buf_it.isValid())
				continue;

			const Plot::DataPoint& point = *buf_it;

			if(stamp.isZero() || point.time < stamp)
			{
				qFill(to_write, (Plot::LinkedBufferIterator*)0);
				to_write[i] = &buf_it;
				stamp = point.time;
			}
			else if(point.time == stamp)
				to_write[i] = &buf_it;
		}

		if(stamp.isZero())
			break;

		if(!end.isZero() && stamp > end)
			break;

		if(stamp >= start)
		{
			io->writeData(stamp, to_write);
			if(earliest.isZero() || stamp < earliest)
				earliest = stamp;
			if(latest.isZero() || stamp > latest)
				latest = stamp;
		}

		// Advance all used iterators
		Q_FOREACH(Plot::LinkedBufferIterator* it, to_write)
		{
			if(!it)
				continue;

			++(*it);
		}
	}

	delete io;

	ROS_INFO("Saved plotter data of time [%.3f,%.3f] of duration %.3fs to '%s'", (earliest.isZero() ? -INFINITY : earliest.toSec()), (latest.isZero() ? INFINITY : latest.toSec()), (latest - earliest).toSec(), path.toLocal8Bit().data());

	if(isBag)
		Plotter::saveTimeWarp(path.toStdString(), start, end);
	
	return true;
}

}