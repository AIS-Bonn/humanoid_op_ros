// Parameter item
// Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>
//         Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

#include <parameter_tuner/parameteritem.h>
#include <QHBoxLayout>
#include <QEvent>
#include <cmath>

using namespace parametertuner;

//
// Float parameter widget
//

FloatParameterWidget::FloatParameterWidget(QTreeWidgetItem* treeItem, ros::NodeHandle& nh, const config_server::ParameterDescription& description)
 : ParameterWidgetBase(treeItem)
 , m_desc(description)
 , m_parameter(description, &nh, false)
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

	m_slider->setMinimum(floor(description.min / m_desc.step));
	m_slider->setMaximum(ceil(description.max / m_desc.step));
	m_slider->setSingleStep(1);
	m_slider->setValue(sliderValueToTicks(m_parameter()));

	m_spinbox->setDecimals(4);
	m_spinbox->setMinimum(description.min); // Note: This minimum value will be internally rounded based on the number of decimals, so don't trust it!
	m_spinbox->setMaximum(description.max); // Note: This maximum value will be internally rounded based on the number of decimals, so don't trust it!
	m_spinbox->setSingleStep(description.step);
	m_spinbox->setValue(m_parameter());
	m_spinbox->setMinimumWidth(80);
	m_spinbox->setAlignment(Qt::AlignRight);

	m_parameter.setCallback(boost::bind(&FloatParameterWidget::called, this, _1));

	m_history.setTreeItem(treeItem);
	m_history.addFilterWidget(m_slider);
	m_history.addFilterWidget(m_spinbox);
	connect(&m_history, SIGNAL(valueChanged(QVariant)), this, SLOT(handleUndoRedo(QVariant)));

	connect(m_slider, SIGNAL(valueChanged(int)), this, SLOT(handleSlider()));
	connect(m_spinbox, SIGNAL(editingFinished()), this, SLOT(handleSpinbox()));
	connect(this, SIGNAL(called(float)), this, SLOT(handleCallback(float)), Qt::QueuedConnection);
}

void FloatParameterWidget::decValue()
{
	m_slider->setValue(m_slider->value() - m_slider->singleStep());
	handleSlider();
}

void FloatParameterWidget::incValue()
{
	m_slider->setValue(m_slider->value() + m_slider->singleStep());
	handleSlider();
}

void FloatParameterWidget::handleSlider()
{
	double value = sliderTicksToValue(m_slider->value());
	m_spinbox->blockSignals(true);
	m_spinbox->setValue(value);
	m_spinbox->blockSignals(false);
	m_parameter.set(value);
}

void FloatParameterWidget::handleSpinbox()
{
	double value = m_spinbox->value();
	if(value > m_desc.max)
		value = m_desc.max;
	else if(value < m_desc.min)
		value = m_desc.min;
	int ticks = sliderValueToTicks(value);
	m_slider->blockSignals(true);
	m_spinbox->blockSignals(true);
	m_slider->setValue(ticks);
	m_spinbox->setValue(value);
	m_slider->blockSignals(false);
	m_spinbox->blockSignals(false);
	m_parameter.set(value);
}

void FloatParameterWidget::handleCallback(float value)
{
	double val = value;
	if(val > m_desc.max)
		val = m_desc.max;
	else if(val < m_desc.min)
		val = m_desc.min;
	int ticks = sliderValueToTicks(val);
	m_slider->blockSignals(true);
	m_spinbox->blockSignals(true);
	m_slider->setValue(ticks);
	m_spinbox->setValue(val);
	m_slider->blockSignals(false);
	m_spinbox->blockSignals(false);
	m_history.addNewValue(QVariant(val));
}

void FloatParameterWidget::handleUndoRedo(QVariant variant)
{
	double value = variant.toDouble();
	if(value > m_desc.max)
		value = m_desc.max;
	else if(value < m_desc.min)
		value = m_desc.min;
	int ticks = sliderValueToTicks(value);
	m_slider->blockSignals(true);
	m_spinbox->blockSignals(true);
	m_slider->setValue(ticks);
	m_spinbox->setValue(value);
	m_slider->blockSignals(false);
	m_spinbox->blockSignals(false);
	m_parameter.set(value);
}

double FloatParameterWidget::sliderTicksToValue(int ticks)
{
	double value = ticks * m_desc.step;
	if(value > m_desc.max)
		value = m_desc.max;
	else if(value < m_desc.min)
		value = m_desc.min;
	return value;
}

int FloatParameterWidget::sliderValueToTicks(double value)
{
	int ticks = round(value / m_desc.step);
	if(value >= m_desc.max || ticks > m_slider->maximum())
		ticks = m_slider->maximum();
	else if(value <= m_desc.min || ticks < m_slider->minimum())
		ticks = m_slider->minimum();
	return ticks;
}

//
// Int parameter widget
//

IntParameterWidget::IntParameterWidget(QTreeWidgetItem* treeItem, ros::NodeHandle& nh, const config_server::ParameterDescription& description)
 : ParameterWidgetBase(treeItem)
 , m_parameter(description, &nh, false)
{
	QHBoxLayout* layout = new QHBoxLayout(this);
	m_slider = new QSlider(Qt::Horizontal,this);
	m_spinbox = new QSpinBox(this);
	layout->addWidget(m_slider);
	layout->addWidget(m_spinbox);

	layout->setContentsMargins(QMargins());

	WheelFilter* filter = new WheelFilter(this);
	m_slider->installEventFilter(filter);
	m_spinbox->installEventFilter(filter);

	m_slider->setMinimum(description.min);
	m_slider->setMaximum(description.max);
	m_slider->setSingleStep(description.step);
	m_slider->setValue(m_parameter());

	m_spinbox->setMinimum(description.min);
	m_spinbox->setMaximum(description.max);
	m_spinbox->setSingleStep(description.step);
	m_spinbox->setValue(m_parameter());
	m_spinbox->setMinimumWidth(80);
	m_spinbox->setAlignment(Qt::AlignRight);

	m_parameter.setCallback(boost::bind(&IntParameterWidget::called, this, _1));

	m_history.setTreeItem(treeItem);
	m_history.addFilterWidget(m_slider);
	m_history.addFilterWidget(m_spinbox);
	connect(&m_history, SIGNAL(valueChanged(QVariant)), this, SLOT(handleUndoRedo(QVariant)));

	connect(m_slider, SIGNAL(valueChanged(int)), this, SLOT(handleSlider()));
	connect(m_spinbox, SIGNAL(editingFinished()), this, SLOT(handleSpinbox()));
	connect(this, SIGNAL(called(int)), this, SLOT(handleCallback(int)), Qt::QueuedConnection);
}

void IntParameterWidget::decValue()
{
	m_slider->setValue(m_slider->value() - m_slider->singleStep());
	handleSlider();
}

void IntParameterWidget::incValue()
{
	m_slider->setValue(m_slider->value() + m_slider->singleStep());
	handleSlider();
}

void IntParameterWidget::handleSlider()
{
	int value = m_slider->value();
	m_spinbox->blockSignals(true);
	m_spinbox->setValue(value);
	m_spinbox->blockSignals(false);
	m_parameter.set(value);
}

void IntParameterWidget::handleSpinbox()
{
	int value = m_spinbox->value();
	m_slider->blockSignals(true);
	m_slider->setValue(value);
	m_slider->blockSignals(false);
	m_parameter.set(value);
}

void IntParameterWidget::handleCallback(int value)
{
	m_slider->blockSignals(true);
	m_spinbox->blockSignals(true);
	m_slider->setValue(value);
	m_spinbox->setValue(value);
	m_slider->blockSignals(false);
	m_spinbox->blockSignals(false);
	m_history.addNewValue(QVariant(m_spinbox->value()));
}

void IntParameterWidget::handleUndoRedo(QVariant variant)
{
	int value = variant.toInt();
	m_slider->blockSignals(true);
	m_spinbox->blockSignals(true);
	m_slider->setValue(value);
	m_spinbox->setValue(value);
	m_slider->blockSignals(false);
	m_spinbox->blockSignals(false);
	m_parameter.set(value);
}

//
// String parameter widget
//

StringParameterWidget::StringParameterWidget(QTreeWidgetItem* treeItem, ros::NodeHandle& nh, const config_server::ParameterDescription& description)
 : ParameterWidgetBase(treeItem)
 , m_parameter(description, &nh, false)
{
	QHBoxLayout* layout = new QHBoxLayout(this);
	m_lineEdit = new QLineEdit(this);
	layout->addWidget(m_lineEdit);

	layout->setContentsMargins(QMargins());

	m_lineEdit->setText(QString::fromStdString(m_parameter()));

	m_parameter.setCallback(boost::bind(&StringParameterWidget::called, this, _1));

	m_history.setTreeItem(treeItem);
	m_history.addFilterWidget(m_lineEdit);
	connect(&m_history, SIGNAL(valueChanged(QVariant)), this, SLOT(handleUndoRedo(QVariant)));
	
	connect(m_lineEdit, SIGNAL(editingFinished()), this, SLOT(handleLineEdit()));
	connect(this, SIGNAL(called(std::string)), this, SLOT(handleCallback(std::string)), Qt::QueuedConnection);
}

void StringParameterWidget::handleLineEdit()
{
	std::string text = m_lineEdit->text().toStdString();
	m_parameter.set(text);
}

void StringParameterWidget::handleCallback(std::string text)
{
	m_lineEdit->blockSignals(true);
	m_lineEdit->setText(QString::fromStdString(text));
	m_lineEdit->blockSignals(false);
	m_history.addNewValue(QVariant(m_lineEdit->text()));
}

void StringParameterWidget::handleUndoRedo(QVariant variant)
{
	QString text = variant.toString();
	m_lineEdit->blockSignals(true);
	m_lineEdit->setText(text);
	m_lineEdit->blockSignals(false);
	m_parameter.set(text.toStdString());
}

//
// Bool parameter widget
//

BoolParameterWidget::BoolParameterWidget(QTreeWidgetItem* treeItem, ros::NodeHandle& nh, const config_server::ParameterDescription& description)
 : ParameterWidgetBase(treeItem)
 , m_parameter(description, &nh, false)
{
	QHBoxLayout* layout = new QHBoxLayout(this);
	m_checkBox = new QCheckBox(this);
	layout->addWidget(m_checkBox);

	m_checkBox->setChecked(m_parameter());

	m_parameter.setCallback(boost::bind(&BoolParameterWidget::called, this, _1));

	m_history.setTreeItem(treeItem);
	m_history.addFilterWidget(m_checkBox);
	connect(&m_history, SIGNAL(valueChanged(QVariant)), this, SLOT(handleUndoRedo(QVariant)));

	connect(m_checkBox, SIGNAL(toggled(bool)), this, SLOT(handleCheckbox()));
	connect(this, SIGNAL(called(bool)), this, SLOT(handleCallback(bool)), Qt::QueuedConnection);
}

void BoolParameterWidget::decValue()
{
	m_checkBox->setChecked(false);
}

void BoolParameterWidget::incValue()
{
	m_checkBox->setChecked(true);
}

void BoolParameterWidget::handleCheckbox()
{
	bool value = m_checkBox->isChecked();
	m_parameter.set(value);
}

void BoolParameterWidget::handleCallback(bool value)
{
	m_checkBox->blockSignals(true);
	m_checkBox->setChecked(value);
	m_checkBox->blockSignals(false);
	m_history.addNewValue(QVariant(m_checkBox->isChecked()));
}

void BoolParameterWidget::handleUndoRedo(QVariant variant)
{
	bool checked = variant.toBool();
	m_checkBox->blockSignals(true);
	m_checkBox->setChecked(checked);
	m_checkBox->blockSignals(false);
	m_parameter.set(checked);
}
// EOF