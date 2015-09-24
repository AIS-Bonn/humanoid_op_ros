// Displays a sensor_msgs::Image using Qt4
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "imagewidget.h"

#include <QtGui/QPainter>

#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <QtGui/QAction>

inline QRgb yuv(float y, float u, float v)
{
	int b = y + (u-128) / 0.493;
	int r = y + (v-128) / 0.877;
	int g = 1.7*y - 0.509*r - 0.194*b;

	if(r < 0)
		r = 0;
	if(r > 255)
		r = 255;
	if(g < 0)
		g = 0;
	if(g > 255)
		g = 255;
	if(b < 0)
		b = 0;
	if(b > 255)
		b = 255;
	return qRgb(r, g, b);
}

ImageWidget::ImageWidget(QWidget* parent)
 : QWidget(parent)
 , m_zoom(1.0)
{
	setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

ImageWidget::~ImageWidget()
{
}

void ImageWidget::paintEvent(QPaintEvent*)
{
	QMutexLocker locker(&m_mutex);

	QPainter painter(this);
	painter.scale(m_zoom, m_zoom);
	painter.drawImage(0, 0, m_img);
}

void ImageWidget::updateWith(const sensor_msgs::Image::Ptr& img)
{
	if(!m_mutex.tryLock())
		return; // Don't hold the processing thread

	QSize lastSize = m_img.size();

	if(img->encoding == "YUYV")
	{
		m_img = QImage(img->width, img->height, QImage::Format_RGB32);

		for(size_t y = 0; y < img->height; ++y)
		{
			for(size_t x = 0; x < img->width-1; x += 2)
			{
				unsigned char* base = &img->data[y*img->step+x*2];
				float y1 = base[0];
				float u  = base[1];
				float y2 = base[2];
				float v  = base[3];
				m_img.setPixel(x, y, yuv(y1, u, v));
				m_img.setPixel(x+1, y, yuv(y2, u, v));
			}
		}
	}
	else if(img->encoding == "UYVY")
	{
		m_img = QImage(img->width, img->height, QImage::Format_RGB32);

		for(size_t y = 0; y < img->height; ++y)
		{
			for(size_t x = 0; x < img->width-1; x += 2)
			{
				unsigned char* base = &img->data[y*img->step+x*2];
				float y1 = base[1];
				float u  = base[0];
				float y2 = base[3];
				float v  = base[2];
				m_img.setPixel(x, y, yuv(y1, u, v));
				m_img.setPixel(x+1, y, yuv(y2, u, v));
			}
		}
	}
	else if(img->encoding == sensor_msgs::image_encodings::MONO8)
	{
		m_img = QImage(img->width, img->height, QImage::Format_RGB32);
		for(size_t y = 0; y < img->height; ++y)
		{
			for(size_t x = 0; x < img->width; ++x)
			{
				uint8_t c = img->data[y*img->step+x];
				m_img.setPixel(x, y, qRgb(c, c, c));
			}
		}
	}
	else if(img->encoding == sensor_msgs::image_encodings::RGB8)
	{
		m_img = QImage(img->width, img->height, QImage::Format_RGB32);
		for(size_t y = 0; y < img->height; ++y)
		{
			memcpy(m_img.scanLine(y), &img->data[y*img->step], img->width*4);
		}
	}
	else if(img->encoding == "classes")
	{
		m_img = QImage(img->width, img->height, QImage::Format_RGB32);
		for(size_t y = 0; y < img->height; ++y)
		{
			for(size_t x = 0; x < img->width; ++x)
			{
				QRgb color;
				switch(img->data[y*img->step + x])
				{
					case 0:  color = qRgb(255,   0,   0); break;
					case 1:  color = qRgb(  0, 255,   0); break;
					case 2:  color = qRgb(  0,   0, 255); break;
					case 3:  color = qRgb(  0, 255, 255); break;
					case 4:  color = qRgb(255,   0, 255); break;
					case 5:  color = qRgb(255, 255,   0); break;
					case 6:  color = qRgb(255, 255, 255); break;
					default: color = qRgb(  0,   0,   0); break;
				}
				m_img.setPixel(x, y, color);
			}
		}
	}
	else
	{
		m_img = QImage(800, 600, QImage::Format_ARGB32);
		QPainter painter(&m_img);

		painter.drawText(0,0,
			QString("Unknown encoding '%s'").arg(img->encoding.c_str())
		);
	}

	if(lastSize != m_img.size())
		updateGeometry();

	m_mutex.unlock();
}

void ImageWidget::setZoom(float zoom)
{
	m_zoom = zoom;
	updateGeometry();
	update();
}

void ImageWidget::zoomIn()
{
	m_zoom *= 1.5;
	updateGeometry();
	update();
}

void ImageWidget::zoomOut()
{
	m_zoom /= 1.5;
	updateGeometry();
	update();
}

void ImageWidget::zoomOriginal()
{
	m_zoom = 1.0;
	updateGeometry();
	update();
}

QAction* ImageWidget::zoomInAction()
{
	QAction* action = new QAction(QIcon::fromTheme("zoom-in"), "Zoom in", this);
	connect(action, SIGNAL(triggered(bool)), SLOT(zoomIn()));
	action->setShortcut(QKeySequence::ZoomIn);
	return action;
}

QAction* ImageWidget::zoomOriginalAction()
{
	QAction* action = new QAction(QIcon::fromTheme("zoom-original"), "Original size", this);
	connect(action, SIGNAL(triggered(bool)), SLOT(zoomOriginal()));
	return action;
}

QAction* ImageWidget::zoomOutAction()
{
	QAction* action = new QAction(QIcon::fromTheme("zoom-out"), "Zoom out", this);
	connect(action, SIGNAL(triggered(bool)), SLOT(zoomOut()));
	action->setShortcut(QKeySequence::ZoomOut);
	return action;
}

QSize ImageWidget::sizeHint() const
{
	return QSize(
		m_zoom * m_img.size().width(),
		m_zoom * m_img.size().height()
	);
}






