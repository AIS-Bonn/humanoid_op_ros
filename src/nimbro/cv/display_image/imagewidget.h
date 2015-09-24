// Displays a sensor_msgs::Image using Qt4
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef IMAGEWIDGET_H
#define IMAGEWIDGET_H

#include <QtGui/QWidget>
#include <QtCore/QMutex>

#include <sensor_msgs/Image.h>

class ImageWidget : public QWidget
{
Q_OBJECT
public:
	ImageWidget(QWidget* parent = 0);
	virtual ~ImageWidget();

	virtual void paintEvent(QPaintEvent*);

	//! This method is thread-safe!
	virtual void updateWith(const sensor_msgs::Image::Ptr& img);

	inline float zoom() const
	{ return m_zoom; }
	void setZoom(float zoom);

	QAction* zoomInAction();
	QAction* zoomOutAction();
	QAction* zoomOriginalAction();

	virtual QSize sizeHint() const;

public slots:
	void zoomIn();
	void zoomOut();
	void zoomOriginal();
private:
	QMutex m_mutex;
	QImage m_img;
	float m_zoom;
};

#endif
