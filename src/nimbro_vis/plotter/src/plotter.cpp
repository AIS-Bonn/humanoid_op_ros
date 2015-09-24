// Plotter plugin
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "plotter/plotter.h"
#include "plotter/plotwidget.h"
#include "plotter/plotmodel.h"
#include "plotter/utils/checkboxdelegate.h"
#include "plotter/utils/colordelegate.h"
#include <plotter/plotfiltermodel.h>
#include <plotter/io/plotio.h>

#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <ros/service.h>

#include <timewarp/TimeWarpControl.h>
#include <timewarp/TimeWarpLoad.h>
#include <timewarp/TimeWarpSave.h>

#include <QtCore/QDebug>

#include <QtGui/QPushButton>
#include <QtGui/QProgressDialog>
#include <QtGui/QFileDialog>
#include <QMessageBox>

#include "ui_plotter.h"

namespace plotter
{

Plotter::Plotter()
 : m_ui(0)
 , m_settings("de.uni-bonn", "plotter")
 , m_loadMenuAction(NULL)
 , m_saveMenuAction(NULL)
 , m_refreshMenuAction(NULL)
 , m_blocked(false)
 , m_labelY(0)
 , m_playing(false)
 , m_rangeStart(0.0)
 , m_rangeEnd(0.0)
 , m_haveFixed(false)
{
}

Plotter::~Plotter()
{
	if(m_ui)
		delete m_ui;
}

void Plotter::initPlugin(qt_gui_cpp::PluginContext& context)
{
	m_w = new QWidget;

	m_ui = new Ui::Plotter;
	m_ui->setupUi(m_w);

	context.addWidget(m_w);

	m_plotModel = new PlotModel(this);
	m_plotModel->rootPlot()->setUsedSettings(&m_settings);

	m_plotFilter = new PlotFilterModel(this);
	m_plotFilter->setSourceModel(m_plotModel);

	m_js_plot = new JointStatePlot(getNodeHandle(), m_plotModel->rootPlot());

	m_ui->plotView->setModel(m_plotFilter);
	m_ui->plotView->setDragEnabled(true);

	m_ui->plotView->setItemDelegateForColumn(PlotModel::COL_ENABLED, new CheckBoxDelegate);
	m_ui->plotView->setItemDelegateForColumn(PlotModel::COL_COLOR, new ColorDelegate);

	m_ui->plot->addPlot(m_plotModel->rootPlot());

	connect(m_ui->pauseButton, SIGNAL(clicked(bool)), SLOT(handlePaused(bool)));
	connect(m_ui->playButton, SIGNAL(clicked(bool)), SLOT(handlePlay(bool)));
	connect(m_plotModel, SIGNAL(modelReset()), SLOT(updateTreeGeometry()));

	connect(m_ui->searchEdit, SIGNAL(textChanged(QString)), m_plotFilter, SLOT(setFilterRegExp(QString)));
	connect(m_ui->hideCheckBox, SIGNAL(clicked(bool)), m_plotFilter, SLOT(setHideDisabledPlots(bool)));

	m_sub_plot = getNodeHandle().subscribe("/plot", 10, &Plotter::plotDataReceived, this);
	qRegisterMetaType<plot_msgs::PlotConstPtr>("plot_msgs::PlotConstPtr");
	connect(this, SIGNAL(plotDataReceived(plot_msgs::PlotConstPtr)),
		SLOT(handlePlotData(plot_msgs::PlotConstPtr)), Qt::QueuedConnection);

	m_ui->plotView->installEventFilter(this);

	handleSelectionChanged(0.0, 0.0, false);
	connect(m_ui->plot, SIGNAL(timeChanged()), SLOT(updateTimeWarp()));
	connect(m_ui->plot, SIGNAL(stopPlaying()), SLOT(handleStopPlaying()));
	connect(m_ui->plot, SIGNAL(selectionChanged(double,double,bool)), SLOT(handleSelectionChanged(double,double,bool)));

	QMenu* menu = new QMenu(m_w);
	m_loadMenuAction = menu->addAction(QIcon::fromTheme("document-open"), "Load data", this, SLOT(load()));
	m_saveMenuAction = menu->addAction(QIcon::fromTheme("document-save"), "Save data", this, SLOT(save()));
	m_refreshMenuAction = menu->addAction(QIcon::fromTheme("view-refresh"), "Refresh", this, SLOT(refresh()));
	m_ui->menuButton->setMenu(menu);

	connect(m_ui->pauseButton, SIGNAL(clicked(bool)), m_loadMenuAction, SLOT(setEnabled(bool)));
	connect(m_ui->pauseButton, SIGNAL(clicked(bool)), m_saveMenuAction, SLOT(setEnabled(bool)));
	m_loadMenuAction->setDisabled(true);
	m_saveMenuAction->setDisabled(true);

	m_io = new PlotIO(m_plotModel->rootPlot(), this);
	connect(m_io, SIGNAL(progress(double)), SLOT(ioProgress(double)));

	m_progressDialog = new QProgressDialog(m_w);
	m_progressDialog->setLabelText(tr("I/O in progress"));
	m_progressDialog->setMinimum(0);
	m_progressDialog->setMaximum(100);
	m_progressDialog->setCancelButton(0);
}

void Plotter::shutdownPlugin()
{
	m_plotModel->rootPlot()->serialize();

	delete m_progressDialog;

	// Shutdown all ROS Subscribers
	m_sub_plot.shutdown();
	m_js_plot->shutdown();
}

void Plotter::handlePaused(bool checked)
{
	m_plotModel->rootPlot()->setPaused(checked);
	updateTimeWarp();
	updatePlayEnabled();
}

bool Plotter::canPlay() const
{
	bool rangeValid = (m_rangeStart == 0.0 || m_rangeEnd == 0.0 || m_rangeStart < m_rangeEnd);
	return (m_plotModel->rootPlot()->paused() && rangeValid);
}

void Plotter::updatePlayEnabled()
{
	if(canPlay())
	{
		m_ui->playButton->setEnabled(true);
	}
	else
	{
		m_ui->playButton->setEnabled(false);
		setPlaying(false);
	}
}

void Plotter::handlePlay(bool checked)
{
	if(m_playing || !canPlay())
		setPlaying(false);
	else
		setPlaying(true);
}

void Plotter::handleSelectionChanged(double start, double end, bool haveFixed)
{
	m_rangeStart = start;
	m_rangeEnd = end;
	m_haveFixed = haveFixed;
	
	updatePlayEnabled();
}

void Plotter::setPlaying(bool play)
{
	if(play == m_playing) return;
	m_playing = play;
	m_ui->plot->setPlaying(m_playing);
	if(m_playing)
	{
		m_ui->playButton->setText("Stop");
	}
	else
	{
		m_ui->playButton->setText("Play");
	}
}

void Plotter::updateTimeWarp()
{
	ros::Time time = m_ui->plot->currentTime();

	timewarp::TimeWarpControl ctrl;
	ctrl.request.live = !m_ui->pauseButton->isChecked();
	ctrl.request.time = time;

	ros::service::call("/tw/control", ctrl);

	m_plotModel->setCurrentTime(time);
}

void Plotter::loadTimeWarp(const std::string& bagPath)
{
	timewarp::TimeWarpLoad msg;
	msg.request.bagPath = bagPath;
	
	ros::service::call("/tw/load", msg);
}

void Plotter::saveTimeWarp(const std::string& bagPath, ros::Time startTime, ros::Time stopTime)
{
	timewarp::TimeWarpSave msg;
	msg.request.bagPath = bagPath;
	msg.request.startTime = startTime;
	msg.request.stopTime = stopTime;
	msg.request.append = true;
	
	ros::service::call("/tw/save", msg);
}

void Plotter::updateTreeGeometry()
{
	int w = 0;
	for(int i = 1; i < m_plotModel->columnCount(); ++i)
	{
		m_ui->plotView->resizeColumnToContents(i);

		if(i != m_plotModel->columnCount()-1)
			w += m_ui->plotView->columnWidth(i);
	}

	m_ui->plotView->setColumnWidth(0, m_ui->plotView->width() - w - 100);
}

void Plotter::handlePlotData(const plot_msgs::PlotConstPtr& data)
{
	if(m_blocked)
		return;

	for(size_t i = 0; i < data->points.size(); ++i)
	{
		const plot_msgs::PlotPoint& point = data->points[i];
		QString name = QString::fromStdString(point.name);

		if(name.isEmpty())
			continue;

		Plot* plot = m_plotModel->rootPlot()->findOrCreatePlotByPath(name);
		plot->put(data->header.stamp, point.value);
	}

	for(size_t i = 0; i < data->events.size(); ++i)
	{
		const std::string& event = data->events[i];
		QString name = QString::fromStdString(event);

		if(name.isEmpty())
			continue;

		Plot* plot = m_plotModel->rootPlot()->findOrCreatePlotByPath(name);
		plot->setIsEventChannel(true);
		plot->put(data->header.stamp, NAN, m_labelY);
		m_labelY = (m_labelY + 1) % 2;
	}
}

bool Plotter::eventFilter(QObject* obj, QEvent* event)
{
	if(event->type() == QEvent::Resize)
		QTimer::singleShot(0, this, SLOT(updateTreeGeometry()));

	return false;
}

void Plotter::save()
{
	QString dir = m_settings.value("logDir").toString();
	QString path = QFileDialog::getSaveFileName(m_w, tr("Save data"), dir, tr("All supported types (*.bag *.csv);;Bag files (*.bag);;CSV files (*.csv)"));
	if(path.isNull())
		return;

	m_blocked = true;

	m_settings.setValue("logDir", QFileInfo(path).dir().path());

	m_progressDialog->setValue(0);
	m_progressDialog->show();
	qApp->processEvents();

	if(!m_io->write(path, m_ui->plot->selectionStart(), m_ui->plot->selectionEnd()))
	{
		QMessageBox::critical(m_w,
			tr("I/O error"),
			tr("An error occured during the saving. Look in stderr for inspiration.")
		);
		m_progressDialog->hide();
		m_blocked = false;
		return;
	}

	m_blocked = false;
	m_progressDialog->hide();

	QPixmap pixmap(m_ui->plot->size());
	m_ui->plot->render(&pixmap);
	pixmap.save(path + ".png");
}

void Plotter::load()
{
	QString dir = m_settings.value("logDir").toString();
	QString path = QFileDialog::getOpenFileName(m_w, tr("Load data"), dir, tr("Bag files (*.bag)"));
	if(path.isNull())
		return;

	m_blocked = true;

	m_settings.setValue("logDir", QFileInfo(path).dir().path());

	m_progressDialog->setValue(0);
	m_progressDialog->show();
	qApp->processEvents();

	if(!m_io->read(path))
	{
		QMessageBox::critical(m_w,
			tr("I/O error"),
			tr("An error occured during the loading. Look in stderr for inspiration.")
		);
	}

	m_progressDialog->hide();
	m_blocked = false;
}

void Plotter::refresh()
{
	m_ui->plot->refresh();
}

void Plotter::ioProgress(double progress)
{
	m_progressDialog->setValue(100 * progress);
	qApp->processEvents();
}

}

PLUGINLIB_EXPORT_CLASS(plotter::Plotter, rqt_gui_cpp::Plugin)
// EOF