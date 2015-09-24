// Color Calibration GUI
// Author: Max Schwarz <Max@x-quadraht.de>

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <QtGui/QMainWindow>
#include <QtCore/QSettings>

#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>

#include "ui_calibration.h"
#include "histogram.h"

#include <calibration/UpdateLUT.h>
#include <camera_v4l2/EnumerateCameraParams.h>
#include <camera_v4l2/SetCameraParam.h>

class CalibScene;
class PlaybackControl;

class Calibration : public QMainWindow
{
Q_OBJECT
public:
	enum ViewMode {
		VM_RAW,
		VM_PROCESSED
	};

	Calibration();
	virtual ~Calibration();

	virtual void keyReleaseEvent(QKeyEvent*);
	virtual void closeEvent(QCloseEvent* );

	void setViewMode(ViewMode vm);
	inline ViewMode viewMode() const
	{ return m_viewMode; }

	inline CalibScene* scene()
	{ return m_scene; }
	inline const QVector<int>& histUV() const
	{ return m_histogram_uv; }
	inline const QVector<int>& histY() const
	{ return m_histogram_y; }
public slots:
	void takeLiveProcessedImage(const sensor_msgs::Image::Ptr& img);
	void takeLiveRawImage(const sensor_msgs::Image::Ptr& img);
	void takeClassificationResult(const sensor_msgs::Image::Ptr& classes);

	void displayImage(const sensor_msgs::Image::Ptr& img, bool calculateClasses = false);
	void displayClasses(const sensor_msgs::Image::Ptr& classes);

	void generateHistograms();

	void notifyLUTUpdateComplete();

	void file_open();
	void file_save();
	void file_saveAs();

	void save_lut();
signals:
	void histogramsChanged();
	void setParamInfo(const camera_v4l2::EnumerateCameraParamsResponse::Ptr& resp);
	void setParam(const camera_v4l2::SetCameraParamRequest::Ptr& req);
	void classify(const sensor_msgs::Image::Ptr& img);
private:
	QDockWidget* createDockWidget(const QString& name, const QString& title,
		QWidget* widget, Qt::DockWidgetArea area);
	void setupDockWidgets();
	void setupActions();
	void save(const QString& filename);

	Ui_Calibration m_ui;
	CalibScene* m_scene;
	PlaybackControl* m_control;

	QMenu* m_menu_view;

	Histogram m_histogram_uv;
	Histogram m_histogram_y;
	sensor_msgs::Image::Ptr m_rawImg;

	QString m_filename;

	QAction* m_act_updateHistograms;
	QAction* m_act_accumulateHistograms;
	QAction* m_act_save_luts;

	QSettings m_settings;



	ViewMode m_viewMode;
	bool m_calculateClasses;
};

#endif
