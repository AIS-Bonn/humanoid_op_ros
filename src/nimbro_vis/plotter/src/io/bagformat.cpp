// Bag file output format
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <plotter/io/bagformat.h>

#include <plot_msgs/Plot.h>

namespace plotter
{

BagFormat::BagFormat()
{
}

BagFormat::~BagFormat()
{
}

Plot::FindFlags BagFormat::defaultFlags()
{
	return Plot::FindRecursive;
}

bool BagFormat::init(const QString& path, const ros::Time& startTime)
{
	try
	{
		m_bag.open(path.toStdString(), rosbag::bagmode::Write);
	}
	catch(rosbag::BagException& e)
	{
		fprintf(stderr, "Could not open output bag file: %s\n", e.what());
		return false;
	}

	return true;
}

void BagFormat::writeHeader(const QLinkedList< Plot::LinkedBufferIterator >& plots)
{
}

void BagFormat::writeData(const ros::Time& time, const QVector< Plot::LinkedBufferIterator* >& data)
{
	plot_msgs::Plot msg;
	msg.header.stamp = time;

	Q_FOREACH(Plot::LinkedBufferIterator* it, data)
	{
		if(!it)
			continue;

		const Plot::DataPoint& p = **it;
		const Plot* plot = it->plot();
		
		if(plot->isEventChannel())
		{
			msg.events.push_back(plot->path().toStdString());
		}
		else
		{
			plot_msgs::PlotPoint point;
			point.name = plot->path().toStdString();
			point.value = p.value;
			msg.points.push_back(point);
		}
	}

	try
	{
		m_bag.write("/plot", time, msg);
	}
	catch(rosbag::BagException& e)
	{
		fprintf(stderr, "Bag exception: %s\n", e.what());
		fprintf(stderr, "Time was %lf\n", time.toSec());
		throw;
	}
}







}
