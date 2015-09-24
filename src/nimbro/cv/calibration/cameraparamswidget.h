// Provides access to camera settings (e.g. white balance)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CAMERAPARAMSWIDGET_H
#define CAMERAPARAMSWIDGET_H

#include <QtGui/QScrollArea>
#include <camera_v4l2/EnumerateCameraParams.h>
#include <camera_v4l2/SetCameraParam.h>

class CameraParamsWidget : public QScrollArea
{
Q_OBJECT
public:
	explicit CameraParamsWidget(QWidget* parent = 0);
	virtual ~CameraParamsWidget();

public slots:
	void setParamInfo(const camera_v4l2::EnumerateCameraParamsResponse::Ptr& resp);
signals:
	void setParam(const camera_v4l2::SetCameraParamRequest::Ptr& req);
private slots:
	void updateSlider(int value);
	void updateCheckBox(bool checked);
	void updateComboBox();
private:
	camera_v4l2::EnumerateCameraParamsResponse::Ptr m_paramInfo;
	QWidget *m_w;

	QWidget* createEditorFor(const camera_v4l2::CameraParam& param);
};

#endif
