// Calibration graphics scene
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "calibscene.h"
#include "calibellipse.h"
#include "histogram.h"

#include <calibration/CalibrationLUT.h>

#include <QtGui/QPainter>
#include <QtCore/QFile>

#include <boost/make_shared.hpp>

#include <math.h>
#include <sstream>
#include <fstream>
#include <iostream>

#include <yaml-cpp/yaml.h>

#include <ros/console.h>

inline QRgb yuv_to_rgb(float y, float u, float v)
{
	int b = 255.0*y + 255.0*u / 0.493;
	int r = 255.0*y + 255.0*v / 0.877;
	int g = 1.7*255.0*y - 0.509*r - 0.194*b;

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
	return qRgba(r, g, b, 0xff);
}


CalibScene::CalibScene(QObject* parent)
 : QGraphicsScene(parent)
 , m_histogram(0)
 , m_maxColor(-1)
{
	setSceneRect(-CALIB_SCALING, -CALIB_SCALING, 2.0*CALIB_SCALING, 2.0*CALIB_SCALING);

	generateLUT();
}

CalibScene::~CalibScene()
{
}

void CalibScene::drawBackground(QPainter* painter, const QRectF& rect)
{
	QImage img(painter->device()->width(), painter->device()->height(),
		QImage::Format_ARGB32);

	for(int x = 0; x < img.width(); ++x)
	{
		for(int y = 0; y < img.height(); ++y)
		{
			float rect_x = (rect.width() * x) / img.width();
			float rect_y = (rect.height() * y) / img.height();

			float u = (rect.left() + rect_x) / CALIB_SCALING;
			float v = -(rect.top() + rect_y) / CALIB_SCALING;

			int int_u = 128 + u*128;
			int int_v = 128 + v*128;

			if(m_histogram && int_u >= 0 && int_u < 256 && int_v >= 0 && int_v < 256)
			{
				int histValue = (*m_histogram)[256*int_u + int_v];
				if(histValue != 0)
				{
					int gray = 0x80 - 0x80*histValue / m_histogram->maximum();
					img.setPixel(x,y, qRgba(gray,gray,gray,0xFF));
					continue;
				}
			}

			if(fabs(u) > 1.0 || fabs(v) > 1.0)
				img.setPixel(x,y, qRgba(0xFF,0xFF,0xFF,0xFF));
			else
				img.setPixel(x,y, yuv_to_rgb(0.5, u, v));
		}
	}

	painter->drawImage(rect, img);
	painter->drawLine(-CALIB_SCALING, 0.0, CALIB_SCALING, 0.0);
	painter->drawLine(0.0, -CALIB_SCALING, 0.0, CALIB_SCALING);
	painter->drawRect(-CALIB_SCALING,-CALIB_SCALING, 2.0*CALIB_SCALING, 2.0*CALIB_SCALING);
}

void CalibScene::generateLUT()
{



		calibration::UpdateLUTRequest::Ptr req = boost::make_shared<calibration::UpdateLUTRequest>();
		calibration::CalibrationLUT* lut = &req->lut;

		lut->lut.resize(256*256);
		for(int u = 0; u < 256; ++u)
		{
			for(int v = 0; v < 256; ++v)
			{
				float scene_x = CALIB_SCALING * (u-128) / 128;
				float scene_y = -CALIB_SCALING * (v-128) / 128;

				CalibEllipse* e = ellipseAt(QPointF(scene_x, scene_y));
				if(e)
					lut->lut[256*u + v] = e->id();
				else
					lut->lut[256*u + v] = 255;
			}
		}

		lut->colors.resize(m_maxColor + 1);
		foreach(QGraphicsItem* item, items())
		{
			if(item->type() != CalibEllipse::Type)
				continue;

			CalibEllipse* e = (CalibEllipse*)item;
			lut->colors[e->id()].maxY = e->maxY();
			lut->colors[e->id()].minY = e->minY();
		}

		emit LUT_updated(req);

}


bool CalibScene::saveLUTIntoAFile(const std::string filename)
{



	std::ofstream  yuvLUTfile(filename.c_str());
	if ( yuvLUTfile.is_open() ) {

		calibration::UpdateLUTRequest::Ptr req = boost::make_shared<calibration::UpdateLUTRequest>();
		calibration::CalibrationLUT* lut = &req->lut;

		lut->lut.resize(256*256);
		for(int u = 0; u < 256; ++u)
		{
			for(int v = 0; v < 256; ++v)
			{
				float scene_x = CALIB_SCALING * (u-128) / 128;
				float scene_y = -CALIB_SCALING * (v-128) / 128;

				CalibEllipse* e = ellipseAt(QPointF(scene_x, scene_y));
				if(e)
					lut->lut[256*u + v] = e->id();
				else
					lut->lut[256*u + v] = 255;
			}
		}

		lut->colors.resize(m_maxColor + 1);


		//255 is reserved for positions without a color class
			yuvLUTfile	<< "Color class: name=none"
						<< " colorid=255"
						<< "\n";


		foreach(QGraphicsItem* item, items())
		{
			if(item->type() != CalibEllipse::Type)
				continue;

			CalibEllipse* e = (CalibEllipse*)item;
			lut->colors[e->id()].maxY = e->maxY();
			lut->colors[e->id()].minY = e->minY();

			yuvLUTfile	<< "Color class: name=" << e->name().toStdString().c_str()
						<< " colorid=" << e->id()
						<< " miny=" << (int)e->minY()
						<< " maxy=" << (int)e->maxY()
						<< "\n";

		}

		//Printing the vector values into a file
		for(int u = 0; u < 256; ++u)
		{
			for(int v = 0; v < 256; ++v)
			{
				if (lut->lut[256*u + v]!=255)
					yuvLUTfile	<< "u_v_id values: "
								<< u << " "
								<< v << " "
								<< (int) lut->lut[256*u + v]
								<< "\n";
			}
		}
		yuvLUTfile.close();

		emit LUT_updated(req);
		return true;
	}
	else{return false;}
}


bool CalibScene::deserialize(const QString& data)
{
	QByteArray c_str = data.toLocal8Bit();
	std::stringstream stream(c_str.constData());

	try
	{
		YAML::Node doc;

		doc = YAML::Load(stream.str());

		clear();

		for(YAML::const_iterator it = doc.begin(); it != doc.end(); ++it)
		{
			const YAML::Node& data = *it;

			CalibEllipse* e = new CalibEllipse();
			addItem(e);
			data >> *e;

			connect(e, SIGNAL(changed()), SLOT(generateLUT()));

			if(e->id() > m_maxColor)
				m_maxColor = e->id();
		}

		generateLUT();

		return true;
	}
	catch(YAML::Exception)
	{
		return false;
	}
}

QString CalibScene::serialize()
{
	YAML::Emitter out;

	out << YAML::BeginSeq;

	foreach(QGraphicsItem* item, items())
	{
		if(item->type() != CalibEllipse::Type)
			continue;

		CalibEllipse* e = (CalibEllipse*)item;
		ROS_INFO("Serializing ellipse %s", e->name().toLocal8Bit().constData());

		out << *e;
	}

	out << YAML::EndSeq;

	return QString(out.c_str());
}

void CalibScene::setHistogram(Histogram* hist)
{
	m_histogram = hist;
}

CalibEllipse* CalibScene::ellipseAt(const QPointF& p) const
{
	QList<QGraphicsItem*> l = items(p);
	for(int i = 0; i < l.size(); ++i)
	{
		if(l[i]->type() == CalibEllipse::Type)
			return (CalibEllipse*)l[i];
	}

	return 0;
}

QList<CalibEllipse*> CalibScene::ellipsesWithID(int id) const
{
	QList<CalibEllipse*> list;
	foreach(QGraphicsItem* item, items())
	{
		if(item->type() != CalibEllipse::Type)
			continue;

		CalibEllipse* e = (CalibEllipse*)item;
		if(e->id() == id)
			list << e;
	}

	return list;
}

