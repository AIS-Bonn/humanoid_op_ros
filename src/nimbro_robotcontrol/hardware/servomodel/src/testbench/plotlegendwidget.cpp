// Legend widget for plot
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "plotlegendwidget.h"
#include "valuelogger.h"
#include "plot.h"

#include <QtGui/QGridLayout>
#include <QtGui/QCheckBox>
#include <QtCore/QVariant>

PlotLegendWidget::PlotLegendWidget(QWidget* parent)
 : QWidget(parent)
 , m_plot(0)
{
	m_layout = new QGridLayout(this);
	setLayout(m_layout);
}

PlotLegendWidget::~PlotLegendWidget()
{
}

void PlotLegendWidget::setPlot(Plot* plot)
{
	if(m_plot)
		disconnect(m_plot, SIGNAL(changed()), this, SLOT(plotChanged()));

	m_plot = plot;
	connect(m_plot, SIGNAL(changed()), SLOT(plotChanged()));

	plotChanged();
}

void PlotLegendWidget::plotChanged()
{
	// Delete all child widgets
	QList<QWidget*> ch = findChildren<QWidget*>();

	Q_FOREACH(QWidget* w, ch)
		delete w;

	disconnect(this, SLOT(loggerChanged()));

	// Rebuild UI
	const QList<ValueLogger*>& loggers = m_plot->loggers();
	for(int i = 0; i < loggers.size(); ++i)
	{
		ValueLogger* logger = loggers[i];

		QCheckBox* checkBox = new QCheckBox(logger->name(), this);

		QPalette p = checkBox->palette();
		p.setColor(QPalette::ButtonText, logger->pen().color());
		p.setColor(QPalette::Foreground, logger->pen().color());
		checkBox->setChecked(logger->visible());
		checkBox->setPalette(p);
		checkBox->setProperty("plot", QVariant::fromValue((QObject*)logger));

		m_layout->addWidget(checkBox, i, 0);

		connect(logger, SIGNAL(changed()), SLOT(loggerChanged()));

		connect(checkBox, SIGNAL(clicked(bool)), logger, SLOT(setVisible(bool)));
		connect(checkBox, SIGNAL(clicked(bool)), SIGNAL(plotSettingsChanged()));
	}
}

void PlotLegendWidget::loggerChanged()
{
	ValueLogger* logger = qobject_cast<ValueLogger*>(sender());
	Q_ASSERT(logger);

	QList<QCheckBox*> widgets = findChildren<QCheckBox*>();

	Q_FOREACH(QCheckBox* box, widgets)
	{
		if(box->property("plot").value<QObject*>() == logger)
		{
			QPalette p = box->palette();
			p.setColor(QPalette::ButtonText, logger->pen().color());
			p.setColor(QPalette::Foreground, logger->pen().color());
			box->setPalette(p);

			box->setChecked(logger->visible());
		}
	}
}
