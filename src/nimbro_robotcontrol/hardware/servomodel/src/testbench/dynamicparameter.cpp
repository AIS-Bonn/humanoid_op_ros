// Dynamic parameter interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "dynamicparameter.h"

#include <QtCore/QVariant>
#include <qsettings.h>

DynamicParameter::DynamicParameter(const QString& _property, double _min, double _step, double _max, const QString& _name, ParametrizedObject* parent)
 : QObject(parent)
 , property(_property)
 , name(_name.isNull() ? _property : _name)
 , min(_min)
 , max(_max)
 , step(_step)
{
	connect(parent, SIGNAL(changed()), SLOT(notify()));
}

void DynamicParameter::setFromSlider(int value)
{
	parent()->setProperty(property.toAscii().constData(),
		(1.0 / 10000.0) * value
	);
}

double DynamicParameter::get() const
{
	return parent()->property(property.toAscii().constData()).toDouble();
}

void DynamicParameter::set(double value)
{
	parent()->setProperty(property.toAscii().constData(), value);
	notify();
}

void DynamicParameter::notify()
{
	double value = get();
	valueChanged(QString::number(value, 'f', 5));
	valueChangedForSlider(10000.0 * value);
}

void DynamicParameter::serialize(QSettings* dest) const
{
	dest->setValue(property, get());
}

void DynamicParameter::deserialize(const QSettings& src)
{
	if(src.contains(property))
		set(src.value(property).toDouble());
}

ParametrizedObject::ParametrizedObject(QObject* parent)
 : QObject(parent)
{
}

void ParametrizedObject::addParameter(const QString& property, double min, double step, double max, const QString& name)
{
	m_parameters <<
		new DynamicParameter(property, min, step, max, name, this);
}

QList< DynamicParameter* > ParametrizedObject::parameters()
{
	return m_parameters;
}

void ParametrizedObject::serialize(QSettings* dest) const
{
	dest->beginGroup(metaObject()->className() + objectName());
	for(int i = 0; i < m_parameters.count(); ++i)
	{
		m_parameters[i]->serialize(dest);
	}
	dest->endGroup();
}

void ParametrizedObject::deserialize(QSettings* src)
{
	src->beginGroup(metaObject()->className() + objectName());
	for(int i = 0; i < m_parameters.count(); ++i)
	{
		m_parameters[i]->deserialize(*src);
	}
	src->endGroup();
}



