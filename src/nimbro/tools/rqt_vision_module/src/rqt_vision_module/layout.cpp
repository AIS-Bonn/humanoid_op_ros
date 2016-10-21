//This package is based on rqt_image_view package. (for copyright notes, please read the ReadMe.txt)
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <rqt_vision_module/layout.hpp>
#include <assert.h>
#include <iostream>

namespace rqt_vision_module
{

RatioLayoutedFrame::RatioLayoutedFrame(QWidget* parent, Qt::WFlags flags) :
		QFrame(), aspect_ratio_(4, 3)
{
	connect(this, SIGNAL(delayed_update()), this, SLOT(update()),
			Qt::QueuedConnection);
	
	this->setMouseTracking(true);
	this->installEventFilter(this);
}

RatioLayoutedFrame::~RatioLayoutedFrame()
{
	
}

const QImage& RatioLayoutedFrame::getImage() const
{
	return qimage_;
}

QImage RatioLayoutedFrame::getImageCopy() const
{
	QImage img;
	qimage_mutex_.lock();
	img = qimage_.copy();
	qimage_mutex_.unlock();
	return img;
}

void RatioLayoutedFrame::setImage(const QImage& image) //, QMutex* image_mutex)
{
	qimage_mutex_.lock();
	qimage_ = image.copy();
	qimage_mutex_.unlock();
	setAspectRatio(qimage_.width(), qimage_.height());
	emit delayed_update();
}

void RatioLayoutedFrame::resizeToFitAspectRatio()
{
	QRect rect = contentsRect();

	// reduce longer edge to aspect ration
	double width = double(rect.width());
	double height = double(rect.height());
	if (width * aspect_ratio_.height() / height > aspect_ratio_.width())
	{
		// too large width
		width = height * aspect_ratio_.width() / aspect_ratio_.height();
		rect.setWidth(int(width + 0.5));
	}
	else
	{
		// too large height
		height = width * aspect_ratio_.height() / aspect_ratio_.width();
		rect.setHeight(int(height + 0.5));
	}

	// resize taking the border line into account
	int border = lineWidth();
	resize(rect.width() + 2 * border, rect.height() + 2 * border);
}

void RatioLayoutedFrame::setInnerFrameMinimumSize(const QSize& size)
{
	int border = lineWidth();
	QSize new_size = size;
	new_size += QSize(2 * border, 2 * border);
	setMinimumSize(new_size);
	emit delayed_update();
}

void RatioLayoutedFrame::setInnerFrameMaximumSize(const QSize& size)
{
	int border = lineWidth();
	QSize new_size = size;
	new_size += QSize(2 * border, 2 * border);
	setMaximumSize(new_size);
	emit delayed_update();
}

void RatioLayoutedFrame::setInnerFrameFixedSize(const QSize& size)
{
	setInnerFrameMinimumSize(size);
	setInnerFrameMaximumSize(size);
}

void RatioLayoutedFrame::setAspectRatio(unsigned short width,
		unsigned short height)
{
	int divisor = greatestCommonDivisor(width, height);
	if (divisor != 0)
	{
		aspect_ratio_.setWidth(width / divisor);
		aspect_ratio_.setHeight(height / divisor);
	}
}

void RatioLayoutedFrame::paintEvent(QPaintEvent* event)
{
	QPainter painter(this);
	qimage_mutex_.lock();
	if (!qimage_.isNull())
	{
		resizeToFitAspectRatio();
		// TODO: check if full draw is really necessary
		//QPaintEvent* paint_event = dynamic_cast<QPaintEvent*>(event);
		//painter.drawImage(paint_event->rect(), qimage_);
		painter.drawImage(contentsRect(), qimage_);
	}
	else
	{
		// default image with gradient
		QLinearGradient gradient(0, 0, frameRect().width(),
				frameRect().height());
		gradient.setColorAt(0, Qt::white);
		gradient.setColorAt(1, Qt::gray);
		painter.setBrush(gradient);
		painter.drawRect(0, 0, frameRect().width() + 1,
				frameRect().height() + 1);
	}
	qimage_mutex_.unlock();
}

int RatioLayoutedFrame::greatestCommonDivisor(int a, int b)
{
	if (b == 0)
	{
		return a;
	}
	return greatestCommonDivisor(b, a % b);
}

// Calculate cursor position in scale from 0 to 1
Point2f RatioLayoutedFrame::getMousePos(QMouseEvent *event)
{
	Point2f pos;
	pos.x = event->pos().x() / (float)frameRect().width();
	pos.y = event->pos().y() / (float)frameRect().height();
	
	return pos;
}

void RatioLayoutedFrame::mousePressEvent(QMouseEvent *event)
{
	output_mutex_.lock();
	Point2f mouse_pos = getMousePos(event);
	
	// Publish respective event
	if(event->button() == Qt::LeftButton)
	{
        //std::cout << "left mouse click " << std::endl;
		mouseEvent(LeftClick, mouse_pos.x, mouse_pos.y);
	}
    else if (event->button() == Qt::RightButton)
	{
        //std::cout << "right mouse click " << std::endl;
		mouseEvent(RightClick, mouse_pos.x, mouse_pos.y);
	}
    else if (event->button() == Qt::MidButton)
	{
        //std::cout << "middle mouse click " << std::endl;
		mouseEvent(MiddleClick, mouse_pos.x, mouse_pos.y);
	}

	output_mutex_.unlock();
}

void RatioLayoutedFrame::mouseReleaseEvent(QMouseEvent* event)
{
	output_mutex_.lock();
	Point2f mouse_pos = getMousePos(event);
	
	// Publish respective event
	if(event->button() == Qt::LeftButton)
	{
        //std::cout << "left mouse release " << std::endl;
		mouseEvent(LeftRelease, mouse_pos.x, mouse_pos.y);
	}
    else if (event->button() == Qt::RightButton)
	{
        //std::cout << "right mouse release " << std::endl;
		mouseEvent(RightRelease, mouse_pos.x, mouse_pos.y);
	}
    else if (event->button() == Qt::MidButton)
	{
        //std::cout << "middle mouse release " << std::endl;
		mouseEvent(MiddleRelease, mouse_pos.x, mouse_pos.y);
	}

	output_mutex_.unlock();
}

void RatioLayoutedFrame::wheelEvent(QWheelEvent* event)
{
	output_mutex_.lock();
	
	float x = event->pos().x() / (float)frameRect().width();
	float y = event->pos().y() / (float)frameRect().height();
	
	if(event->delta() < 0)
		mouseEvent(MiddleRotatedForward, x, y);
	else
		mouseEvent(MiddleRotatedBackwards, x, y);
	
	output_mutex_.unlock();
}

bool RatioLayoutedFrame::eventFilter(QObject* obj, QEvent* event)
{
	if (event->type() == QEvent::MouseMove)
    {
		QMouseEvent *mEvent = (QMouseEvent*)event;
		Point2f mouse_pos = getMousePos(mEvent);
		
		//std::cout << "move " << x << "   " << y << std::endl;
		mouseEvent(Move, mouse_pos.x, mouse_pos.y);
    }
	
	return QObject::eventFilter(obj, event);
}

}
