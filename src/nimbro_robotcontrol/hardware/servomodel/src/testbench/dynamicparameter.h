// Dynamic parameter interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DYNAMICPARAMETER_H
#define DYNAMICPARAMETER_H

#include <QtCore/QObject>

class ParametrizedObject;
class QSettings;

class DynamicParameter : public QObject
{
Q_OBJECT
public:
	DynamicParameter(const QString& _property, double _min, double _step, double _max, const QString& _name, ParametrizedObject* parent);

	QString property;
	QString name;
	double min;
	double max;
	double step;

	double get() const;

	void serialize(QSettings* dest) const;
	void deserialize(const QSettings& src);
public Q_SLOTS:
	void set(double value);
	void setFromSlider(int value);
	void notify();
Q_SIGNALS:
	void valueChanged(const QString& txt);
	void valueChangedForSlider(int value);
};

class ParametrizedObject : public QObject
{
Q_OBJECT
public:
	explicit ParametrizedObject(QObject* parent = 0);
	QList<DynamicParameter*> parameters();
	void addParameter(const QString& property, double min, double step, double max, const QString& name);

	void serialize(QSettings* dest) const;
	void deserialize(QSettings* src);
Q_SIGNALS:
	void changed();
private:
	QList<DynamicParameter*> m_parameters;
};

#endif
