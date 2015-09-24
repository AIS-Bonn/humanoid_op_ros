// Color Calibration Node
// Author: Max Schwarz <Max@x-quadraht.de>

#ifndef CALIBRATIONNODELET_H
#define CALIBRATIONNODELET_H

#include "calibration.h"

#include <camera_v4l2/EnumerateCameraParams.h>
#include <camera_v4l2/SetCameraParam.h>

#include <QtGui/QApplication>
#include <qtguithread/qtguithread.h>

#include <nodelet/nodelet.h>

class CalibrationNodelet : public QObject, public nodelet::Nodelet, public WidgetProvider
{
Q_OBJECT
public:
	void onInit();

	virtual QWidget* createWidget();
	void receiveRawImage(const sensor_msgs::Image::Ptr& img);
	void receiveProcessedImage(const sensor_msgs::Image::Ptr& img);
public slots:
	void sendLUT(const calibration::UpdateLUTRequest::Ptr& lut);

	void requestCameraParamEnumeration();
	void setCameraParam(const camera_v4l2::SetCameraParamRequest::Ptr& param);
	void classify(const sensor_msgs::Image::Ptr& img);
signals:
	void rawImageReceived(const sensor_msgs::Image::Ptr& img);
	void processedImageReceived(const sensor_msgs::Image::Ptr& img);

	void gotCameraParams(const camera_v4l2::EnumerateCameraParamsResponse::Ptr& response);

	void gotClassificationResult(const sensor_msgs::Image::Ptr& classes);

	void LUTUpdateComplete();
private:
	Calibration* m_calib;
	ros::Subscriber m_sub_raw;
	ros::Subscriber m_sub_processed;

	ros::ServiceClient m_client_enumerate;

	ros::Time m_lastStamp;
};

#endif
