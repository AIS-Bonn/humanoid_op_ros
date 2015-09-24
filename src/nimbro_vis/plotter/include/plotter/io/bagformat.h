// Bag file output format
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PLOTTER_IO_BAGFORMAT_H
#define PLOTTER_IO_BAGFORMAT_H

#include <plotter/io/fileformat.h>

#include <rosbag/bag.h>

namespace plotter
{

class BagFormat : public FileFormat
{
public:
	BagFormat();
	virtual ~BagFormat();

	virtual Plot::FindFlags defaultFlags();
	virtual bool init(const QString& path, const ros::Time& startTime);
	virtual void writeHeader(const QLinkedList< Plot::LinkedBufferIterator >& plots);
	virtual void writeData(const ros::Time& time, const QVector< Plot::LinkedBufferIterator* >& data);
private:
	rosbag::Bag m_bag;
};

}

#endif
