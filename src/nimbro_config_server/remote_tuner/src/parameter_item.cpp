//subclass for slider
//Author: Sebastian Schüller

#include "parameter_item.h"

#include <QHBoxLayout>
#include <QEvent>
#include <QMessageBox>

#include <ros/console.h>
#include <ros/service.h>

#include <boost/lexical_cast.hpp>

#include <config_server/SetParameter.h>


namespace remote_tuner
{

WheelFilter::WheelFilter(QObject* parent): QObject(parent)
{}

bool WheelFilter::eventFilter(QObject* object, QEvent* event )
{
	if (event->type() == QEvent::Wheel)
		return true;
	return false;
}

ParameterWidgetBase::ParameterWidgetBase()
{
}

void ParameterWidgetBase::setValue(const config_server::ParameterValue& value)
{
	m_name = value.name;
	update(value);
}

void ParameterWidgetBase::notify()
{
	setRequested(QString::fromStdString(m_name), value());
}

FloatParameterWidget::FloatParameterWidget()
 : ParameterWidgetBase()
{
	QHBoxLayout* layout = new QHBoxLayout(this);
	m_slider = new QSlider(Qt::Horizontal,this);
	m_spinbox = new QDoubleSpinBox(this);
	layout->addWidget(m_slider);
	layout->addWidget(m_spinbox);

	layout->setContentsMargins(QMargins());

	WheelFilter* filter = new WheelFilter(this);

	m_slider->installEventFilter(filter);
	m_spinbox->installEventFilter(filter);

	connect(m_slider, SIGNAL(sliderReleased()), this, SLOT(handleSlider()));
	connect(m_spinbox, SIGNAL(editingFinished()), this, SLOT(handleSpinbox()));
}

FloatParameterWidget::~FloatParameterWidget()
{
}

void FloatParameterWidget::update(const config_server::ParameterValue& value)
{
	m_slider->blockSignals(true);
	m_spinbox->blockSignals(true);

	m_sliderStepRatio = 1.0 / (double)value.step;

	m_slider->setMinimum(value.min * m_sliderStepRatio);
	m_slider->setMaximum(value.max * m_sliderStepRatio);
	m_slider->setSingleStep(1);

	m_spinbox->setMinimum(value.min);
	m_spinbox->setMaximum(value.max);
	m_spinbox->setSingleStep(value.step);
	m_spinbox->setDecimals(4);
	m_spinbox->setMinimumWidth(80);
	m_spinbox->setAlignment(Qt::AlignRight);

	try
	{
		float v = boost::lexical_cast<float>(value.value);
		m_slider->setValue(v * m_sliderStepRatio);
		m_spinbox->setValue(v);
	}
	catch(std::bad_cast&)
	{
	}

	m_slider->blockSignals(false);
	m_spinbox->blockSignals(false);
}

void FloatParameterWidget::DecValue()
{
	m_spinbox->setValue(m_spinbox->value() - m_spinbox->singleStep());
	handleSpinbox();
}

void FloatParameterWidget::IncValue()
{
	m_spinbox->setValue(m_spinbox->value() + m_spinbox->singleStep());
	handleSpinbox();
}

void FloatParameterWidget::handleSpinbox()
{
	float value = m_spinbox->value();
	m_slider->blockSignals(true);
	m_slider->setValue(value * m_sliderStepRatio);
	m_slider->blockSignals(false);
	notify();
}

void FloatParameterWidget::handleSlider()
{
	float value = ((float)m_slider->value()) / m_sliderStepRatio;
	m_spinbox->blockSignals(true);
	m_spinbox->setValue(value);
	m_spinbox->blockSignals(false);
	notify();
}

QString FloatParameterWidget::value() const
{
	return QString::number(m_spinbox->value());
}

IntParameterWidget::IntParameterWidget()
 : ParameterWidgetBase()
{
	QHBoxLayout* layout = new QHBoxLayout(this);
	m_slider = new QSlider(Qt::Horizontal,this);
	m_spinbox = new QSpinBox(this);
	layout->addWidget(m_slider);
	layout->addWidget(m_spinbox);

	layout->setContentsMargins(QMargins());

	m_spinbox->setMinimumWidth(80);
	m_spinbox->setAlignment(Qt::AlignRight);

	connect(m_slider, SIGNAL(sliderReleased()), this, SLOT(handleSlider()));
	connect(m_spinbox, SIGNAL(editingFinished()), this, SLOT(handleSpinbox()));
	connect(this, SIGNAL(called(int)), this, SLOT(handleCallback(int)), Qt::QueuedConnection);
}

IntParameterWidget::~IntParameterWidget()
{
}

void IntParameterWidget::update(const config_server::ParameterValue& value)
{
	m_slider->setMinimum(value.min);
	m_slider->setMaximum(value.max);
	m_slider->setSingleStep(value.step);

	m_spinbox->setMinimum(value.min);
	m_spinbox->setMaximum(value.max);
	m_spinbox->setSingleStep(value.step);

	m_slider->blockSignals(true);
	m_spinbox->blockSignals(true);

	try
	{
		int v = boost::lexical_cast<int>(value.value);
		m_slider->setValue(v);
		m_spinbox->setValue(v);
	}
	catch(std::bad_cast&)
	{
	}

	m_slider->blockSignals(false);
	m_spinbox->blockSignals(false);
}

void IntParameterWidget::DecValue()
{
	m_slider->setValue(m_slider->value() - 1);
	handleSpinbox();
}


void IntParameterWidget::IncValue()
{
	m_slider->setValue(m_slider->value() + 1);
	handleSpinbox();
}

void IntParameterWidget::handleSlider()
{
	int value = m_slider->value();
	m_spinbox->blockSignals(true);
	m_spinbox->setValue(value);
	m_spinbox->blockSignals(false);
	notify();
}

void IntParameterWidget::handleSpinbox()
{
	int value = m_spinbox->value();
	m_slider->blockSignals(true);
	m_slider->setValue(value);
	m_slider->blockSignals(false);
	notify();
}

QString IntParameterWidget::value() const
{
	return QString::number(m_spinbox->value());
}



StringParameterWidget::StringParameterWidget()
 : ParameterWidgetBase()
{
	QHBoxLayout* layout = new QHBoxLayout(this);
	m_lineEdit = new QLineEdit(this);
	layout->addWidget(m_lineEdit);

	layout->setContentsMargins(QMargins());

	connect(m_lineEdit, SIGNAL(editingFinished()), this, SLOT(handleLineEdit()));
}

StringParameterWidget::~StringParameterWidget()
{}

void StringParameterWidget::DecValue()
{}

void StringParameterWidget::IncValue()
{}

void StringParameterWidget::update(const config_server::ParameterValue& value)
{
	m_lineEdit->blockSignals(true);
	m_lineEdit->setText(QString::fromStdString(value.value));
	m_lineEdit->blockSignals(false);
}

void StringParameterWidget::handleLineEdit()
{
	notify();
}

QString StringParameterWidget::value() const
{
	return m_lineEdit->text();
}


BoolParameterWidget::BoolParameterWidget()
 : ParameterWidgetBase()
{
	QHBoxLayout* layout = new QHBoxLayout(this);
	m_checkBox = new QCheckBox(this);
	layout->addWidget(m_checkBox);

	connect(m_checkBox, SIGNAL(toggled(bool)), this, SLOT(handleCheckbox()));
}

BoolParameterWidget::~BoolParameterWidget()
{}

void BoolParameterWidget::update(const config_server::ParameterValue& value)
{
	m_checkBox->blockSignals(true);
	m_checkBox->setChecked(value.value == "1");

	m_checkBox->blockSignals(false);
}

void BoolParameterWidget::handleCheckbox()
{
	notify();
}

void BoolParameterWidget::DecValue()
{
	m_checkBox->setChecked(false);
}

void BoolParameterWidget::IncValue()
{
	m_checkBox->setChecked(true);
}

QString BoolParameterWidget::value() const
{
	return m_checkBox->isChecked() ? "1" : "0";
}

}
