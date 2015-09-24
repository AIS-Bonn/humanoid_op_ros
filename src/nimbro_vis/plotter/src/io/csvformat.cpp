// CSV file formatter
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <plotter/io/csvformat.h>

#include <stdio.h>

const int FIELD_WIDTH = 40;

namespace plotter
{

CSVFormat::CSVFormat()
 : m_startTime(0)
{
}

CSVFormat::~CSVFormat()
{
}

bool CSVFormat::init(const QString& path, const ros::Time& startTime)
{
	m_file.setFileName(path);
	if(!m_file.open(QIODevice::WriteOnly))
	{
		fprintf(stderr, "Could not open output file: %s\n",
			m_file.errorString().toLatin1().constData()
		);
		return false;
	}

	m_out.setDevice(&m_file);

	return true;
}

Plot::FindFlags CSVFormat::defaultFlags()
{
	return Plot::FindRecursive | Plot::FindOnlyVisible;
}

void CSVFormat::writeHeader(const QLinkedList< Plot::LinkedBufferIterator >& plots)
{
	m_out << QString("%1").arg("Time", FIELD_WIDTH);
	Q_FOREACH(const Plot::LinkedBufferIterator& it, plots)
	{
		m_out << QString(", %1").arg("\"" + it.plot()->path() + "\"", FIELD_WIDTH);
	}
	m_out << '\n';
}

void CSVFormat::writeData(const ros::Time& time, const QVector< Plot::LinkedBufferIterator* >& data)
{
	if(m_startTime == ros::Time(0))
		m_startTime = time;

	m_out << QString("%1").arg((time - m_startTime).toSec(), FIELD_WIDTH, 'g', 7);

	Q_FOREACH(Plot::LinkedBufferIterator* it, data)
	{
		if(!it)
			m_out << QString(", %1").arg(QChar('x'), FIELD_WIDTH);
		else if(it->plot()->isEventChannel())
			m_out << QString(", %1").arg(1, FIELD_WIDTH);
		else
			m_out << QString(", %1").arg((**it).value, FIELD_WIDTH, 'g', 7);
	}
	m_out << '\n';
}

}
