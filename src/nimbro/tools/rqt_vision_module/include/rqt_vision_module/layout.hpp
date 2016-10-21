//This package is based on rqt_image_view package. (for copyright notes, please read the ReadMe.txt)
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <rqt_vision_module/mouseevent.hpp>
#include <QLayout>
#include <QFrame>
#include <QLayoutItem>
#include <QSize>
#include <QMutex>
#include <QRect>
#include <QImage>
#include <QPainter>
#include <QMouseEvent>
#include <QDebug>


using namespace cv;

namespace rqt_vision_module
{

class RatioLayoutedFrame: public QFrame
{

Q_OBJECT

public:

	mutable QMutex output_mutex_;

	Point2d ClickedPoint;

	QList<QString> listText;

	RatioLayoutedFrame(QWidget* parent, Qt::WFlags flags = 0);

	virtual ~RatioLayoutedFrame();

	const QImage& getImage() const;

	QImage getImageCopy() const;

	void setImage(const QImage& image);

	QRect getAspectRatioCorrectPaintArea();

	void resizeToFitAspectRatio();

	void setInnerFrameMinimumSize(const QSize& size);

	void setInnerFrameMaximumSize(const QSize& size);

	void setInnerFrameFixedSize(const QSize& size);

	signals:

	void delayed_update();
	void mouseEvent(MouseEvent event, float x, float y);

protected:

	void setAspectRatio(unsigned short width, unsigned short height);
	
	virtual void mousePressEvent(QMouseEvent * event);
	virtual void mouseReleaseEvent(QMouseEvent * event);
	virtual void wheelEvent(QWheelEvent* event);

	virtual void paintEvent(QPaintEvent* event);
	virtual bool eventFilter(QObject *obj, QEvent *event);
	
private:
	static int greatestCommonDivisor(int a, int b);
	Point2f getMousePos(QMouseEvent *event);
	
private:
	QSize aspect_ratio_;

	mutable QMutex qimage_mutex_;
	QImage qimage_;
};

}
