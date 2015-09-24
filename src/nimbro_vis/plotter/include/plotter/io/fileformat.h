// Output file format definition
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PLOTTER_FILEFORMAT_H
#define PLOTTER_FILEFORMAT_H

#include <plotter/plot.h>

#include <QtCore/QVector>

namespace plotter
{

class FileFormat
{
public:
	FileFormat() {}
	virtual ~FileFormat() {}

	virtual bool init(const QString& path, const ros::Time& startTime) = 0;
	virtual Plot::FindFlags defaultFlags() = 0;
	virtual void writeHeader(const QLinkedList<Plot::LinkedBufferIterator>& plots) = 0;
	virtual void writeData(const ros::Time& time, const QVector<Plot::LinkedBufferIterator*>& data) = 0;
};

}

#endif
