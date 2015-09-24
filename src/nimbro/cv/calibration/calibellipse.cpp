// Calibration ellipse scene node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "calibellipse.h"

#include <QtGui/QPainter>
#include <QtGui/QGraphicsSceneMouseEvent>
#include <QtGui/QCursor>
#include <QtGui/QMenu>
#include <QtGui/QGraphicsScene>
#include <QtGui/QMessageBox>
#include <QtGui/QGraphicsView>

#include <math.h>

#include <nodelet/nodelet.h>

#include "calibscene.h"

CalibEllipse::CalibEllipse(QGraphicsItem* parent)
 : QGraphicsObject(parent)
 , m_angle(0)
 , m_cov_x(0)
 , m_cov_y(0)
 , m_resize_x(false)
 , m_resize_y(false)
{
	setFlag(ItemIsMovable, 1);
	setFlag(ItemIsSelectable, 1);
	setAcceptHoverEvents(true);

	m_labelContainer = new QGraphicsItemGroup(this);
	m_label = new QGraphicsSimpleTextItem(m_labelContainer);
	m_labelContainer->setFlag(ItemIgnoresTransformations, 1);
	QFont font = m_label->font();
	font.setPointSize(8);
	m_label->setFont(font);
}

CalibEllipse::~CalibEllipse()
{
}

QRectF CalibEllipse::boundingRect() const
{
	QPen pen;
	QRectF rect(
		-CALIB_SCALING*m_cov_x/2, -CALIB_SCALING*m_cov_y/2,
		CALIB_SCALING*m_cov_x, CALIB_SCALING*m_cov_y
	);
	float w = pen.widthF() / 2.0;

	return rect.adjusted(-w,-w,w,w);
}

void CalibEllipse::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
	QBrush brush(QColor(0x80,0x80,0x80,0x80));
	QPen pen;

	if(isSelected())
	{
		brush.setColor(brush.color().darker());
		pen.setStyle(Qt::DashLine);
		pen.setColor(Qt::red);
		painter->setPen(pen);
	}

	painter->setBrush(brush);
	painter->drawEllipse(
		QPointF(0,0),
		CALIB_SCALING*m_cov_x/2, CALIB_SCALING*m_cov_y/2
	);
}

void CalibEllipse::setMean(const QPointF& mean)
{
	setPos(mean.x()*CALIB_SCALING, -mean.y()*CALIB_SCALING);
	update();
}

QPointF CalibEllipse::mean() const
{
	return QPointF(pos().x(), -pos().y()) / CALIB_SCALING;
}

void CalibEllipse::setAngle(float angle)
{
	while(angle >= M_PI)
		angle -= 2.0*M_PI;
	while(angle <= -M_PI)
		angle += 2.0*M_PI;

	m_angle = angle;
	setRotation( -(180.0/M_PI)*angle);
	update();
}

void CalibEllipse::setCov(float cov_x, float cov_y)
{
	prepareGeometryChange();
	m_cov_x = cov_x;
	m_cov_y = cov_y;
	update();
}

void CalibEllipse::mousePressEvent(QGraphicsSceneMouseEvent* event)
{
	QPointF pos = event->pos();

	m_mouseMoved = false;

	if(event->buttons() & Qt::LeftButton)
	{
		m_resize_x = fabs(pos.x())/CALIB_SCALING > 0.8*m_cov_x/2;
		m_resize_y = fabs(pos.y())/CALIB_SCALING > 0.8*m_cov_y/2;

		if(!m_resize_x && !m_resize_y)
			QGraphicsItem::mousePressEvent(event);
	}
	else if(event->buttons() & Qt::RightButton)
	{
		m_rotateAngle = atan2(-event->pos().y(), event->pos().x());
		event->accept();
		grabMouse();
	}
}

void CalibEllipse::mouseReleaseEvent(QGraphicsSceneMouseEvent* event)
{
	QGraphicsItem::mouseReleaseEvent(event);

	m_resize_x = false;
	m_resize_y = false;
	ungrabMouse();

	if(m_mouseMoved)
		emit changed();

	if(!m_mouseMoved && event->button() == Qt::RightButton)
		contextMenu(event->screenPos());
}

void CalibEllipse::mouseMoveEvent(QGraphicsSceneMouseEvent* event)
{
	m_mouseMoved = true;

	if(m_resize_x)
	{
		prepareGeometryChange();
		m_cov_x = 2.0 * fabs(event->pos().x()) / CALIB_SCALING;
	}
	else if(m_resize_y)
	{
		prepareGeometryChange();
		m_cov_y = 2.0 * fabs(event->pos().y()) / CALIB_SCALING;
	}
	else if(event->buttons() & Qt::RightButton)
	{
		float delta = atan2(-event->pos().y(), event->pos().x()) - m_rotateAngle;
		setAngle(angle() + delta);
	}
	else
		QGraphicsItem::mouseMoveEvent(event);
}

void CalibEllipse::hoverMoveEvent(QGraphicsSceneHoverEvent* event)
{
	QPointF pos = event->pos();

	if(fabs(pos.x()) / CALIB_SCALING > 0.8*m_cov_x/2)
		setCursor(Qt::SizeHorCursor);
	else if(fabs(pos.y()) / CALIB_SCALING > 0.8*m_cov_y/2)
		setCursor(Qt::SizeVerCursor);
	else
		setCursor(QCursor());
}

void CalibEllipse::contextMenu(const QPoint& screenPos)
{
	QMenu menu;

	QAction* clone = menu.addAction(QIcon::fromTheme("edit-copy"), tr("Clone Ellipse"));
	QAction* del = menu.addAction(QIcon::fromTheme("edit-delete"), tr("Delete Ellipse"));

	QAction* selected = menu.exec(screenPos);

	if(selected == clone)
	{
		CalibEllipse* e = new CalibEllipse;
		scene()->addItem(e);

		e->setName(m_name);
		e->setColorID(m_id);
		e->setMean(mean());
		e->setCov(0.5, 0.5);
		e->setYRange(minY(), maxY());
	}
	else if(selected == del)
	{
		// Check if this ellipse is the last one of this color
		bool lastOne = true;
		foreach(QGraphicsItem* item, scene()->items())
		{
			if(item == this || item->type() != Type)
				continue;

			CalibEllipse* e = (CalibEllipse*)item;
			if(e->id() == m_id)
			{
				lastOne = false;
				break;
			}
		}

		if(lastOne)
		{
			int ret = QMessageBox::question(scene()->views()[0],
				tr("Warning"),
				tr("This is the last ellipse of the color class '%1'. "
				   "Are you sure you want to remove it?").arg(m_name),
				QMessageBox::Yes | QMessageBox::No
			);

			if(ret == QMessageBox::Yes)
				deleteLater();
		}
		else
			deleteLater();
	}

	delete clone;
	delete del;
}


QPainterPath CalibEllipse::shape() const
{
	QPainterPath path;
	path.addEllipse(QPointF(0,0), CALIB_SCALING*m_cov_x/2, CALIB_SCALING*m_cov_y/2);
	return path;
}

void CalibEllipse::setName(const QString& name)
{
	m_name = name;
	m_label->setText(QString("%1 (%2)").arg(name).arg(m_id));
	QRectF brect = m_label->boundingRect();
	m_label->setPos(-brect.width()/2, -brect.height()/2);
}

void CalibEllipse::setColorID(int id)
{
	m_id = id;
	setName(m_name);
}

void CalibEllipse::setMaxY(uint8_t max)
{
	m_maxY = max;
}

void CalibEllipse::setMinY(uint8_t min)
{
	m_minY = min;
}

void CalibEllipse::setYRange(uint8_t min, uint8_t max)
{
	m_minY = min;
	m_maxY = max;
}

int CalibEllipse::type() const
{
	return Type;
}






// SERIALIZATION / DESERIALIZATION
const YAML::Node& operator>>(const YAML::Node& node, CalibEllipse& e)
{
	std::string name = node["name"].as<std::string>();
	const YAML::Node& mean = node["mean"];
	const YAML::Node& cov = node["cov"];
	const YAML::Node& Yrange = node["yrange"];
	float angle = node["angle"].as<float>();
	int id = node["colorID"].as<int>();

	e.setMean(QPointF(mean[0].as<float>(), mean[1].as<float>()));
	e.setCov(cov[0].as<float>(), cov[1].as<float>());
	e.setAngle(angle);
	e.setColorID(id);
	e.setName(QString(name.c_str()));
	e.setYRange((int)Yrange[0].as<int>(), (int)Yrange[1].as<int>());

	return node;
}

YAML::Emitter& operator<<(YAML::Emitter& em, CalibEllipse& e)
{
	em << YAML::BeginMap;

	em
		<< YAML::Key << "name"
		<< YAML::Value << e.name().toLocal8Bit().constData()

		<< YAML::Key << "colorID"
		<< YAML::Value << e.id()

		<< YAML::Key << "mean"
		<< YAML::Value
			<< YAML::BeginSeq
			<< e.mean().x() << e.mean().y()
			<< YAML::EndSeq

		<< YAML::Key << "cov"
		<< YAML::Value
			<< YAML::BeginSeq
			<< e.covX() << e.covY()
			<< YAML::EndSeq

		<< YAML::Key << "angle"
		<< YAML::Value << e.angle()

		<< YAML::Key << "yrange"
		<< YAML::Value
			<< YAML::BeginSeq
			<< (int)e.minY() << (int)e.maxY()
			<< YAML::EndSeq
	;

	em << YAML::EndMap;

	return em;
}

