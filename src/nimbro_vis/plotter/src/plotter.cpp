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

#include <timewarp/TimeWarpLoad.h>
#include <timewarp/TimeWarpSave.h>
#include <std_srvs/Empty.h>

#include <QtCore/QDebug>

#include <QPushButton>
#include <QProgressDialog>
#include <QFileDialog>
#include <QMessageBox>

#include "ui_plotter.h"

namespace plotter
{

Plotter::Plotter()
 : m_ui(0)
 , m_settings("de.uni-bonn", "plotter")
 , m_loadMenuAction(NULL)
 , m_saveMenuAction(NULL)
 , m_clearMenuAction(NULL)
 , m_clearPlotMenuAction(NULL)
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

	new JointStatePlot(getNodeHandle(), m_plotModel->rootPlot()); // The created JointStatePlot gets deleted when the root plot does, or all of its child plots

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

	m_sub_plot = getNodeHandle().subscribe("/plot", 100, &Plotter::plotDataReceived, this);
	qRegisterMetaType<plot_msgs::PlotConstPtr>("plot_msgs::PlotConstPtr");
	connect(this, SIGNAL(plotDataReceived(plot_msgs::PlotConstPtr)), SLOT(handlePlotData(plot_msgs::PlotConstPtr)), Qt::QueuedConnection);

	m_sub_timewarpStatus = getNodeHandle().subscribe("/tw/control_status", 1, &Plotter::timewarpStatusReceived, this);
	qRegisterMetaType<timewarp::TimeWarpControlConstPtr>("timewarp::TimeWarpControlConstPtr");
	connect(this, SIGNAL(timewarpStatusReceived(timewarp::TimeWarpControlConstPtr)), SLOT(handleTimewarpStatus(timewarp::TimeWarpControlConstPtr)), Qt::QueuedConnection);

	m_pub_timewarpControl = getNodeHandle().advertise<timewarp::TimeWarpControl>("/tw/control", 1);

	m_ui->plotView->installEventFilter(this);

	handleSelectionChanged(0.0, 0.0, false);
	connect(m_ui->plot, SIGNAL(timeChanged()), SLOT(updateTimeWarp()));
	connect(m_ui->plot, SIGNAL(stopPlaying()), SLOT(handleStopPlaying()));
	connect(m_ui->plot, SIGNAL(selectionChanged(double,double,bool)), SLOT(handleSelectionChanged(double,double,bool)));

	QMenu* menu = new QMenu(m_w); // Standard icon names: https://specifications.freedesktop.org/icon-naming-spec/icon-naming-spec-latest.html#names
	m_loadMenuAction = menu->addAction(QIcon::fromTheme("document-open"), "Load data", this, SLOT(load()));
	m_saveMenuAction = menu->addAction(QIcon::fromTheme("document-save"), "Save data", this, SLOT(save()));
	m_assignColoursAction = menu->addAction(QIcon::fromTheme("applications-graphics"), "Assign colours", this, SLOT(assignColours()));
	m_clearMenuAction = menu->addAction(QIcon::fromTheme("edit-delete"), "Clear data", this, SLOT(clear()));
	m_clearPlotMenuAction = menu->addAction(QIcon::fromTheme("edit-clear"), "Clear plot", this, SLOT(clearPlot()));
	m_refreshMenuAction = menu->addAction(QIcon::fromTheme("view-refresh"), "Refresh", this, SLOT(refresh()));
	m_ui->menuButton->setMenu(menu);

	connect(m_ui->pauseButton, SIGNAL(clicked(bool)), m_loadMenuAction, SLOT(setEnabled(bool)));
	connect(m_ui->pauseButton, SIGNAL(clicked(bool)), m_saveMenuAction, SLOT(setEnabled(bool)));
	m_loadMenuAction->setDisabled(true);
	m_saveMenuAction->setDisabled(true);

	m_io = new PlotIO(getNodeHandle(), m_plotModel->rootPlot(), this);
	connect(m_io, SIGNAL(progress(double)), SLOT(ioProgress(double)));

	m_progressDialog = new QProgressDialog(m_w);
	m_progressDialog->setLabelText(tr("I/O in progress"));
	m_progressDialog->setMinimum(0);
	m_progressDialog->setMaximum(100);
	m_progressDialog->setAutoClose(false);
	m_progressDialog->setAutoReset(false);
	m_progressDialog->setCancelButton(0);
}

void Plotter::shutdownPlugin()
{
	delete m_progressDialog;

	m_plotModel->rootPlot()->serialize();

	// Shutdown all ROS Subscribers
	m_plotModel->rootPlot()->shutdown();
	m_sub_plot.shutdown();
	m_sub_timewarpStatus.shutdown();
	m_pub_timewarpControl.shutdown();
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
		m_ui->playButton->setEnabled(true);
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
		m_ui->playButton->setText("Stop");
	else
		m_ui->playButton->setText("Play");
}

void Plotter::updateTimeWarp()
{
	ros::Time time = m_ui->plot->currentTime();

	timewarp::TimeWarpControl msg;
	msg.live = !m_ui->pauseButton->isChecked();
	msg.time = time;
	m_pub_timewarpControl.publish(msg);

	m_plotModel->setCurrentTime(time);
}

void Plotter::clearTimewarp()
{
	std_srvs::Empty srv;
	ros::service::call("/tw/clear", srv);
}

void Plotter::loadTimeWarp(const std::string& bagPath)
{
	timewarp::TimeWarpLoad srv;
	srv.request.bagPath = bagPath;
	
	ros::service::call("/tw/load", srv);
}

void Plotter::saveTimeWarp(const std::string& bagPath, ros::Time startTime, ros::Time stopTime)
{
	timewarp::TimeWarpSave srv;
	srv.request.bagPath = bagPath;
	srv.request.startTime = startTime;
	srv.request.stopTime = stopTime;
	srv.request.append = true;
	
	ros::service::call("/tw/save", srv);
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
	if(m_blocked || m_ui->pauseButton->isChecked())
		return;

	for(std::size_t i = 0; i < data->points.size(); ++i)
	{
		const plot_msgs::PlotPoint& point = data->points[i];
		QString name = QString::fromStdString(point.name);

		if(name.isEmpty())
			continue;

		Plot* plot = m_plotModel->rootPlot()->findOrCreatePlotByPath(name);
		plot->put(data->header.stamp, point.value, NULL, true, false);
	}

	for(std::size_t i = 0; i < data->events.size(); ++i)
	{
		const std::string& event = data->events[i];
		QString name = QString::fromStdString(event);

		if(name.isEmpty())
			continue;

		Plot* plot = m_plotModel->rootPlot()->findOrCreatePlotByPath(name);
		plot->setIsEventChannel(true);
		plot->put(data->header.stamp, NAN, &m_labelY, true, false);
	}
}

void Plotter::handleTimewarpStatus(const timewarp::TimeWarpControlConstPtr& msg)
{
	bool isLive = (!m_ui->pauseButton->isChecked());
	if(!isLive && (msg->live != isLive  || msg->time != m_ui->plot->currentTime()))
		updateTimeWarp();
}

bool Plotter::eventFilter(QObject* obj, QEvent* event)
{
	if(event->type() == QEvent::Resize)
		QTimer::singleShot(0, this, SLOT(updateTreeGeometry()));

	return false;
}

void Plotter::load()
{
	QString dir = m_settings.value("logDir").toString();
	QString path = QFileDialog::getOpenFileName(m_w, tr("Load data"), dir, tr("Bag files (*.bag)"));
	if(path.isNull())
		return;

	m_ui->plot->blockPainting();
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
	m_progressDialog->reset();
	m_blocked = false;
	m_ui->plot->unblockPainting();
	m_ui->plot->refresh();
}

void Plotter::save()
{
	QString dir = m_settings.value("logDir").toString();
	QString path = QFileDialog::getSaveFileName(m_w, tr("Save data"), dir, tr("All supported types (*.bag *.csv);;Bag files (*.bag);;CSV files (*.csv)"));
	if(path.isNull())
		return;

	m_ui->plot->blockPainting();
	m_blocked = true;

	m_settings.setValue("logDir", QFileInfo(path).dir().path());

	m_progressDialog->setValue(0);
	m_progressDialog->show();
	qApp->processEvents();

	bool success = m_io->write(path, m_ui->plot->selectionStart(), m_ui->plot->selectionEnd());
	if(!success)
	{
		QMessageBox::critical(m_w,
			tr("I/O error"),
			tr("An error occured during the saving. Look in stderr for inspiration.")
		);
	}

	m_progressDialog->hide();
	m_progressDialog->reset();
	m_blocked = false;
	m_ui->plot->unblockPainting();
	m_ui->plot->refresh();

	if(!success) return;

	QPixmap pixmap(m_ui->plot->size());
	m_ui->plot->render(&pixmap);
	pixmap.save(path + ".png");
}

void Plotter::assignColours()
{
	m_ui->plot->blockPainting();

	m_plotModel->rootPlot()->assignColorRec(0, false);

	m_ui->plot->unblockPainting();
	m_ui->plot->refresh();
}

void Plotter::clear()
{
	m_ui->plot->blockPainting();
	m_blocked = true;

	setPlaying(false);
	m_ui->plot->clearSelection();

	QList<Plot*> childrenList = m_plotModel->rootPlot()->childPlots();
	qDeleteAll(childrenList);
	new JointStatePlot(getNodeHandle(), m_plotModel->rootPlot()); // The created JointStatePlot gets deleted when the root plot does, or all of its child plots

	clearTimewarp();

	m_blocked = false;
	m_ui->plot->unblockPainting();
	m_ui->plot->refresh();
}

void Plotter::clearPlot()
{
	m_ui->plot->blockPainting();
	m_blocked = true;

	setPlaying(false);
	m_ui->plot->clearSelection();

	m_plotModel->rootPlot()->clearDataRec(false);

	clearTimewarp();

	m_blocked = false;
	m_ui->plot->unblockPainting();
	m_ui->plot->refresh();
}

void Plotter::refresh()
{
	m_ui->plot->refresh();
}

void Plotter::ioProgress(double progress)
{
	if(progress < 0.0)
		progress = 0.0;
	int progVal = (int)(100.0*progress + 0.5);
	if(progVal > 100)
		progVal = 100;
	m_progressDialog->setValue(progVal);
	qApp->processEvents();
}

}

PLUGINLIB_EXPORT_CLASS(plotter::Plotter, rqt_gui_cpp::Plugin)
// EOF