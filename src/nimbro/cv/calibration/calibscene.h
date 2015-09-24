// Calibration graphics scene
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CALIBSCENE_H
#define CALIBSCENE_H

#include <QtGui/QGraphicsScene>
#include <calibration/UpdateLUT.h>

// Due to a Qt4 bug we cannot use an unity coordinate system
const float CALIB_SCALING = 100;

class CalibEllipse;
class Histogram;

class CalibScene : public QGraphicsScene
{
Q_OBJECT
public:
	CalibScene(QObject* parent = 0);
	virtual ~CalibScene();

	virtual void drawBackground(QPainter* painter, const QRectF& rect);

	void setHistogram(Histogram* hist);

	CalibEllipse* ellipseAt(const QPointF& p) const;
	QList<CalibEllipse*> ellipsesWithID(int id) const;

	QString serialize();
public slots:
	void generateLUT();
	bool saveLUTIntoAFile(const std::string filename);
	bool deserialize(const QString& data);
signals:
	void LUT_updated(const calibration::UpdateLUTRequest::Ptr& lut);
private:
	Histogram* m_histogram;
	int m_maxColor;
};

#endif
