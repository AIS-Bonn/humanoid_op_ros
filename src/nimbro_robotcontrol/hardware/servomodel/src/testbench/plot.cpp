// Plot manager
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "plot.h"
#include "valuelogger.h"

#include <QtCore/QSettings>

Plot::Plot(QObject* parent)
 : QObject(parent)
{
}

Plot::~Plot()
{
}

ValueLogger* Plot::addLogger(const QString& name)
{
	ValueLogger* logger = new ValueLogger(this);
	logger->setName(name);

	m_loggers << logger;

	changed();

	return logger;
}

void Plot::reset()
{
	Q_FOREACH(ValueLogger* logger, m_loggers)
		logger->reset();
}

void Plot::paint(QPainter* painter)
{
	Q_FOREACH(ValueLogger* logger, m_loggers)
		logger->paint(painter);
}

void Plot::deserialize(QSettings* settings)
{
	settings->beginGroup("Plot" + objectName());
	Q_FOREACH(ValueLogger* logger, m_loggers)
		logger->setVisible(settings->value(logger->name(), true).toBool());
	settings->endGroup();
}

void Plot::serialize(QSettings* settings) const
{
	settings->beginGroup("Plot" + objectName());
	Q_FOREACH(ValueLogger* logger, m_loggers)
		settings->setValue(logger->name(), logger->visible());
	settings->endGroup();
}


