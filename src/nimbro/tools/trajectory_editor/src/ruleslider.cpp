// Widget to control one rule 
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <trajectory_editor/ruleslider.h>

const double ratio = 100; // Ratio to convert between slider/spin

RuleSlider::RuleSlider(const int rule_id, const std::string name, QWidget* parent): QWidget(parent)
{
	// Init
	m_rule_id = rule_id;
	m_prev_value = 0;
	
	m_name_label  = new QLabel(QString::fromStdString(name));
	m_zero_button = new QPushButton(tr("Set 0"));
	
	m_layout = new QHBoxLayout();
	m_spin   = new QDoubleSpinBox();
	m_slider = new QSlider();
	
	// Configure gui
	float min = -1;
	float max = 1;
	
	m_slider->setMaximum(max * ratio);
	m_slider->setMinimum(min * ratio);
	m_slider->setValue(0);
	m_slider->setOrientation(Qt::Horizontal);
	
	m_spin->setMaximum(max);
	m_spin->setMinimum(min);
	m_spin->setSingleStep(0.01);
	m_spin->setValue(0);
	
	m_name_label->setMinimumSize(150, 20);
	
	// Set up layout
	m_layout->addWidget(m_name_label);
	m_layout->addWidget(m_spin);
	m_layout->addWidget(m_slider);
	m_layout->addWidget(m_zero_button);
	
	m_layout->setContentsMargins(0, 0, 0, 0);
	setLayout(m_layout);
	
	// Set up connections
	connect(m_spin, SIGNAL(valueChanged(double)), this, SLOT(spinChanged()));
	connect(m_slider, SIGNAL(valueChanged(int)), this, SLOT(sliderChanged()));
	connect(m_zero_button, SIGNAL(pressed()), this, SLOT(setZero()));
}

void RuleSlider::handleValueChange(double new_value)
{
	double delta = new_value - m_prev_value;
	m_prev_value = new_value;
	
	applyRule(delta, m_rule_id);
}

void RuleSlider::setZero()
{
	m_spin->blockSignals(true);
	m_slider->blockSignals(true);
	
	m_spin->setValue(0);
	m_slider->setValue(0);
	m_prev_value = 0;
	
	m_spin->blockSignals(false);
	m_slider->blockSignals(false);
}

void RuleSlider::sliderChanged()
{
	m_spin->blockSignals(true);
	m_spin->setValue(m_slider->value() / ratio);
	m_spin->blockSignals(false);
	
	handleValueChange(m_spin->value());
}

void RuleSlider::spinChanged()
{
	m_slider->blockSignals(true);
	m_slider->setValue(m_spin->value() * ratio);
	m_slider->blockSignals(false);
	
	handleValueChange(m_spin->value());
}

RuleSlider::~RuleSlider()
{

}
