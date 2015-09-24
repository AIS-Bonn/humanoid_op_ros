// Calibration ellipse scene node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CALIBELLIPSE_H
#define CALIBELLIPSE_H

#include <QtGui/QGraphicsObject>

#include <yaml-cpp/yaml.h>

#include <stdint.h>

class CalibEllipse : public QGraphicsObject
{
Q_OBJECT
public:
	CalibEllipse(QGraphicsItem* parent = 0);
	virtual ~CalibEllipse();

	enum { Type = UserType + 1 };

	virtual QRectF boundingRect() const;
	virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = 0);
	virtual void hoverMoveEvent(QGraphicsSceneHoverEvent* event);
	virtual void mousePressEvent(QGraphicsSceneMouseEvent* event);
	virtual void mouseMoveEvent(QGraphicsSceneMouseEvent* event);
	virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent* event);
	virtual void contextMenu(const QPoint& screenPos);
	virtual QPainterPath shape() const;
	virtual int type() const;

	void setMean(const QPointF& mean);
	void setAngle(float angle);
	void setCov(float cov_x, float cov_y);
	void setName(const QString& name);
	void setColorID(int id);
	void setYRange(uint8_t min, uint8_t max);
	void setMinY(uint8_t min);
	void setMaxY(uint8_t max);

	QPointF mean() const;
	float angle() const
	{ return m_angle; }
	float covX() const
	{ return m_cov_x; }
	float covY() const
	{ return m_cov_y; }
	int id() const
	{ return m_id; }
	uint8_t minY() const
	{ return m_minY; }
	uint8_t maxY() const
	{ return m_maxY; }
	const QString& name() const
	{ return m_name; }
signals:
	void changed();
private:
	float m_angle;
	float m_cov_x;
	float m_cov_y;
	QString m_name;
	int m_id;
	uint8_t m_minY;
	uint8_t m_maxY;

	bool m_resize_x;
	bool m_resize_y;
	float m_rotateAngle;

	QGraphicsItem* m_labelContainer;
	QGraphicsSimpleTextItem* m_label;

	bool m_mouseMoved;
};

const YAML::Node& operator>>(const YAML::Node& node, CalibEllipse& ellipse);
YAML::Emitter& operator<<(YAML::Emitter& em, CalibEllipse& ellipse);

#endif
