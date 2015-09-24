// Color Calibration GUI
// Author: Max Schwarz <Max@x-quadraht.de>

#include "calibration.h"

#include <QtGui/QGraphicsView>
#include <QtGui/QDockWidget>
#include <QtGui/QMenuBar>
#include <QtGui/QKeyEvent>
#include <QtGui/QMessageBox>
#include <QtGui/QFileDialog>
#include <QtGui/QToolBar>
#include <QtCore/QFile>

#include "calibwidget.h"
#include "calibscene.h"
#include "cameraparamswidget.h"
#include "calibimagewidget.h"
#include "histwidget.h"
#include "playbackcontrol.h"

#include <imagewidget.h>

#include <stdio.h>

#include <ros/ros.h>

const char* SETTINGS_DEFAULT_DIR = "defaultDir";

Calibration::Calibration()
 : QMainWindow()
 , m_scene(new CalibScene(this))
 , m_histogram_uv(256*256)
 , m_histogram_y(256)
 , m_settings("nimbro", "calibration")
 , m_viewMode(VM_RAW)
 , m_calculateClasses(false)
{
	QWidget* w = new QWidget(this);
	setCentralWidget(w);
	m_ui.setupUi(w);

	connect(
		m_ui.display, SIGNAL(selectionChanged()),
		SLOT(generateHistograms())
	);

	connect(
		this, SIGNAL(histogramsChanged()), m_scene, SLOT(update())
	);
	m_scene->setHistogram(&m_histogram_uv);

	setupActions();
	m_menu_view = menuBar()->addMenu(tr("&View"));
	setupDockWidgets();

	setWindowTitle("Calibration");
	restoreGeometry(m_settings.value("mainWindow/geometry").toByteArray());
	restoreState(m_settings.value("mainWindow/state").toByteArray());
}

Calibration::~Calibration()
{
}

void Calibration::setupActions()
{
	QMenu* file = menuBar()->addMenu(tr("&File"));
	QToolBar* ftool = new QToolBar("File", this);
	ftool->setObjectName("toolBar_file");
	addToolBar(ftool);

	QAction* open = new QAction(QIcon::fromTheme("document-open"), tr("&Open"), this);
	connect(open, SIGNAL(triggered(bool)), SLOT(file_open()));
	open->setShortcut(QKeySequence::Open);
	file->addAction(open);
	ftool->addAction(open);

	QAction* save = new QAction(QIcon::fromTheme("document-save"), tr("Save"), this);
	connect(save, SIGNAL(triggered(bool)), SLOT(file_save()));
	save->setShortcut(QKeySequence::Save);
	file->addAction(save);
	ftool->addAction(save);

	QAction* saveAs = new QAction(QIcon::fromTheme("document-save-as"), tr("Save as"), this);
	connect(saveAs, SIGNAL(triggered(bool)), SLOT(file_saveAs()));
	saveAs->setShortcut(QKeySequence::SaveAs);
	file->addAction(saveAs);


	// Image widget actions
	QAction* zoomIn = m_ui.display->zoomInAction();
	QAction* zoomOut = m_ui.display->zoomOutAction();
	QAction* zoomOriginal = m_ui.display->zoomOriginalAction();

	QToolBar* imageToolBar = new QToolBar("Image", this);
	imageToolBar->setObjectName("toolBar_image");
	imageToolBar->addAction(zoomIn);
	imageToolBar->addAction(zoomOut);
	imageToolBar->addAction(zoomOriginal);
	addToolBar(imageToolBar);

	QMenu* imageMenu = menuBar()->addMenu(tr("&Image"));
	imageMenu->addAction(zoomIn);
	imageMenu->addAction(zoomOut);
	imageMenu->addAction(zoomOriginal);

	m_act_updateHistograms = new QAction(QIcon::fromTheme("view-refresh"), tr("Auto update histograms"), this);
	m_act_updateHistograms->setCheckable(true);
	m_act_accumulateHistograms = new QAction(tr("Accumulate histograms"), this);
	m_act_accumulateHistograms->setCheckable(true);

	m_act_save_luts = new QAction(tr("Save Look-up table"), this);
	connect(m_act_save_luts, SIGNAL(triggered(bool)), SLOT(save_lut()));


	QToolBar* histToolBar = new QToolBar(tr("Histograms"), this);
	histToolBar->setObjectName("toolBar_histograms");
	histToolBar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	histToolBar->addAction(m_act_accumulateHistograms);
	histToolBar->addAction(m_act_updateHistograms);
	histToolBar->addAction(m_act_save_luts);

	addToolBar(histToolBar);

	QMenu* histMenu = menuBar()->addMenu(tr("&Histograms"));
	histMenu->addAction(m_act_accumulateHistograms);
	histMenu->addAction(m_act_updateHistograms);
}

QDockWidget* Calibration::createDockWidget(const QString& name, const QString& title, QWidget* widget, Qt::DockWidgetArea area)
{
	QDockWidget* dock = new QDockWidget(title);
	dock->setWidget(widget);
	dock->setObjectName(name);
	addDockWidget(area, dock);
	m_menu_view->addAction(dock->toggleViewAction());

	m_settings.beginGroup(name);
	if(m_settings.contains("geometry"))
	{
		ROS_INFO("Resizing dock widget '%s'", name.toLocal8Bit().constData());
// 		widget->restoreGeometry(m_settings.value("geometry").toByteArray());
		widget->resize(m_settings.value("size").toSize());
	}
	m_settings.endGroup();

	return dock;
}

void Calibration::setupDockWidgets()
{
	setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

	createDockWidget(
		"yuvdock", tr("YUV space"),
		new CalibWidget(m_scene),
		Qt::RightDockWidgetArea
	);

	HistWidget* hist = new HistWidget(m_scene, &m_histogram_y);
	createDockWidget("y_hist", tr("Y histogram"), hist, Qt::BottomDockWidgetArea);

	connect(this, SIGNAL(histogramsChanged()), hist, SLOT(update()));

	connect(m_scene, SIGNAL(selectionChanged()), hist, SLOT(update()));
	connect(hist, SIGNAL(selectionChanged()), m_scene, SLOT(generateLUT()));

	CameraParamsWidget* params = new CameraParamsWidget();
	createDockWidget(
		"camParams", tr("Camera Parameters"),
		params, Qt::RightDockWidgetArea
	);

	qRegisterMetaType<camera_v4l2::SetCameraParamRequest::Ptr>("camera_v4l2::SetCameraParamRequest::Ptr");
	qRegisterMetaType<camera_v4l2::EnumerateCameraParamsResponse::Ptr>("camera_v4l2::EnumerateCameraParamsResponse::Ptr");
	connect(
		params, SIGNAL(setParam(camera_v4l2::SetCameraParamRequest::Ptr)),
		this, SIGNAL(setParam(camera_v4l2::SetCameraParamRequest::Ptr))
	);
	connect(
		this, SIGNAL(setParamInfo(camera_v4l2::EnumerateCameraParamsResponse::Ptr)),
		params, SLOT(setParamInfo(camera_v4l2::EnumerateCameraParamsResponse::Ptr))
	);

	m_control = new PlaybackControl(this);
	createDockWidget(
		"playback", tr("Playback control"),
		m_control, Qt::RightDockWidgetArea
	);
	menuBar()->addMenu(m_control->menu());
}

void Calibration::takeLiveProcessedImage(const sensor_msgs::Image::Ptr& img)
{
	m_control->takeLiveProcessedImage(img);
}

void Calibration::takeLiveRawImage(const sensor_msgs::Image::Ptr& img)
{
	m_control->takeLiveRawImage(img);
}

void Calibration::displayClasses(const sensor_msgs::Image::Ptr& classes)
{
	if(m_viewMode != VM_PROCESSED)
		return;

	m_ui.display->updateWith(classes);
	m_ui.display->update();
}

void Calibration::displayImage(const sensor_msgs::Image::Ptr& img, bool calculateClasses)
{
	m_rawImg = img;
	m_calculateClasses = calculateClasses;

	if(m_act_updateHistograms->isChecked())
		generateHistograms();

	if(calculateClasses)
		emit classify(img);

	if(m_viewMode != VM_RAW)
		return;

	m_ui.display->updateWith(img);
	m_ui.display->update();
}


void Calibration::keyReleaseEvent(QKeyEvent* event)
{
	if(event->key() == Qt::Key_Space)
	{
		m_control->toggleViewMode();
	}
	else
		QWidget::keyReleaseEvent(event);
}

void Calibration::generateHistograms()
{
	const QRect& selection = m_ui.display->selection();

	if(!m_act_accumulateHistograms->isChecked())
	{
		m_histogram_uv.reset();
		m_histogram_y.reset();
	}

	if(!m_rawImg)
		return;

	uint8_t* data = &m_rawImg->data[0];
	for(int y = selection.top(); y < selection.bottom(); ++y)
	{
		uint8_t* line = data + y * m_rawImg->step;
		for(int x = selection.left() & ~1; x < selection.right()-1; x += 2)
		{
			uint8_t y1, y2, u, v;

			u =  line[2*x + 1];
			y1 = line[2*x + 0];
			v  = line[2*x + 3];
			y2 = line[2*x + 2];

			m_histogram_uv.hit(256*u + v);

			m_histogram_y.hit(y1);
			m_histogram_y.hit(y2);
		}
	}

	emit histogramsChanged();
}

void Calibration::file_open()
{
	QString filename = QFileDialog::getOpenFileName(
		this, tr("Open"),
		m_settings.value(SETTINGS_DEFAULT_DIR).toString(),
		"Calibration files (*.calib)"
	);

	if(filename.isNull())
		return;

	QDir dir;
	m_settings.setValue(SETTINGS_DEFAULT_DIR, dir.absoluteFilePath(filename));

	QFile file(filename);
	if(!file.open(QIODevice::ReadOnly))
	{
		QMessageBox::critical(
			this, tr("Error"),
			tr("Could not open file: %1").arg(file.errorString())
		);
		return;
	}

	if(!m_scene->deserialize(file.readAll()))
	{
		QMessageBox::critical(
			this, tr("Error"),
			tr("Could not load input file. Inconsistent YAML?")
		);
	}

	m_filename = filename;
}

void Calibration::file_save()
{
	if(m_filename.isNull())
	{
		file_saveAs();
		return;
	}

	save(m_filename);
}

void Calibration::file_saveAs()
{
	QString filename = QFileDialog::getSaveFileName(
		this, tr("Save"),
		m_settings.value(SETTINGS_DEFAULT_DIR).toString(),
		"Calibration files (*.calib)"
	);

	if(filename.isNull())
		return;

	QDir dir;
	m_settings.setValue(SETTINGS_DEFAULT_DIR, dir.absoluteFilePath(filename));

	save(filename);
}

void Calibration::save(const QString& filename)
{
	QFile file(filename);

	if(!file.open(QIODevice::WriteOnly))
	{
		QMessageBox::critical(
			this, tr("Error"),
			tr("Could not open file: %1").arg(file.errorString())
		);
		return;
	}

	file.write(m_scene->serialize().toLocal8Bit());
	file.write("\n");

	m_filename = filename;
}

void Calibration::closeEvent(QCloseEvent* event)
{
	m_settings.setValue("mainWindow/state", saveState());
	m_settings.setValue("mainWindow/geometry", saveGeometry());

	foreach(QObject* obj, children())
	{
		QDockWidget* w = qobject_cast<QDockWidget*>(obj);

		if(!w)
			continue;

		m_settings.beginGroup(w->objectName());
		m_settings.setValue("size", w->widget()->size());
		m_settings.endGroup();
	}

	QWidget::closeEvent(event);
}

void Calibration::setViewMode(Calibration::ViewMode vm)
{
	m_viewMode = vm;
}

void Calibration::takeClassificationResult(const sensor_msgs::Image::Ptr& classes)
{
	displayClasses(classes);
}

void Calibration::notifyLUTUpdateComplete()
{
	if(!m_calculateClasses)
		return;

	emit classify(m_rawImg);
}

void Calibration::save_lut(){

	QString filename = QFileDialog::getSaveFileName(
		this, tr("Save LUT"),
		m_settings.value(SETTINGS_DEFAULT_DIR).toString(),
		"LUT files (*.lut)"
	);


	if(filename.isNull())
		return;

	QDir dir;
	//dir.absoluteFilePath(filename).toStdString().c_str());

	if (!m_scene->saveLUTIntoAFile(dir.absoluteFilePath(filename).toStdString())) {

		QMessageBox::critical(
			this, tr("Error"),
			tr("Could not save the look-up table file.")
		);
	}


}
