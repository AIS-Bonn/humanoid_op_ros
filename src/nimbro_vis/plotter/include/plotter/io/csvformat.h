// CSV file formatter
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PLOTTER_IO_CSVFORMAT_H
#define PLOTTER_IO_CSVFORMAT_H

#include <plotter/io/fileformat.h>

#include <QtCore/QFile>
#include <QtCore/QTextStream>

namespace plotter
{

class CSVFormat : public FileFormat
{
public:
	CSVFormat();
	virtual ~CSVFormat();

	virtual Plot::FindFlags defaultFlags();
	virtual bool init(const QString& path, const ros::Time& startTime);
	virtual void writeHeader(const QLinkedList< Plot::LinkedBufferIterator >& plots);
	virtual void writeData(const ros::Time& time, const QVector< Plot::LinkedBufferIterator* >& data);
private:
	QFile m_file;
	QTextStream m_out;
	ros::Time m_startTime;
};

}

#endif
