#include <trajectory_editor/spaces/positionview.h>

#include <QGridLayout>
#include <QLabel>
#include <QEvent>

#include <math.h>
#include <stdio.h>

const double ratio = 10000;

PositionView::PositionView(PositionView::Alignment alignment, PositionView::Type type
, std::string jointName, QWidget *parent)
	: QWidget(parent)
{
	this->jointName = jointName;
	
	QGridLayout *layout = new QGridLayout(this);
	
	positionSlider = new QSlider(Qt::Horizontal);
	positionSpin = new QDoubleSpinBox();
	
	min = -M_PI;
	max = M_PI;
	
	positionSlider->setMinimum(min*ratio);
	positionSlider->setMaximum(max*ratio);
	positionSlider->setValue(0);
	positionSlider->setToolTip("Rate");
	positionSlider->installEventFilter(this);

	positionSpin->setMinimum(min);
	positionSpin->setMaximum(max);
	positionSpin->setValue(0);
	positionSpin->setSingleStep(0.01);
	positionSpin->setToolTip("Rate");
	positionSpin->installEventFilter(this);
	
	m_prev_value = 0;
	
	if(alignment == LEFT)
	{
		layout->addWidget(positionSpin, 1, 2);
		layout->addWidget(positionSlider, 1, 3);
	}
	else
	{
		layout->addWidget(positionSpin, 1, 1);
		layout->addWidget(positionSlider, 1, 0);
	}

	layout->setContentsMargins(5,0,5,0);
	this->setLayout(layout);
	
	connect(positionSlider, SIGNAL(valueChanged(int)), this, SLOT(positionSliderChanged()));
	connect(positionSpin, SIGNAL(valueChanged(double)), this, SLOT(positionSpinChanged()));
}

bool PositionView::withinRange(double value)
{
	return (value >= min && value <= max);
}

void PositionView::positionSliderChanged()
{
	positionSpin->blockSignals(true);
	positionSpin->setValue(positionSlider->value() / ratio);
	positionSpin->blockSignals(false);

	positionChanged(jointName, m_prev_value, positionSpin->value());
	m_prev_value = positionSpin->value();
}

void PositionView::positionSpinChanged()
{
	positionSlider->blockSignals(true);
	positionSlider->setValue(positionSpin->value() * ratio);
	positionSlider->blockSignals(false);
	
	positionChanged(jointName, m_prev_value, positionSpin->value());
	m_prev_value = positionSpin->value();
}

void PositionView::setPosition(double position)
{
	positionSpin->blockSignals(true);
	positionSlider->blockSignals(true);
	
	positionSpin->setValue(position);
	positionSlider->setValue(position * ratio);
	
	m_prev_value = positionSpin->value();
	
	positionSpin->blockSignals(false);
	positionSlider->blockSignals(false);
}

double PositionView::getPosition()
{
	return positionSpin->value();
}

bool PositionView::eventFilter(QObject *object, QEvent *event)
{
    if(event->type() == QEvent::Wheel) // Ignore wheel scrolling
	{
		if(object == positionSlider || object == positionSpin)
			return true;
    }
    
    return false;
}

PositionView::~PositionView()
{
	
}