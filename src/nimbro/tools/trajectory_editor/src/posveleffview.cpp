#include <trajectory_editor/spaces/posveleffview.h>

#include <QLabel>
#include <QEvent>
#include <QKeyEvent>

#include <ros/package.h>
#include <ros/console.h>

const double ratio = 10000; // Ratio to convert between position slider/spin

PosVelEffView::PosVelEffView(BasicSmallView::Alignment alignment, BasicSmallView::Type type
							,std::string  jointName, bool shiftMirrored, QWidget *parent)
	: BasicSmallView(alignment, type, jointName, shiftMirrored, parent)
{
	// Init gui
	positionSlider = new QSlider(Qt::Horizontal);
	positionSpin   = new QDoubleSpinBox();
	effortSpin     = new QDoubleSpinBox();
	velocitySpin   = new QDoubleSpinBox();
	
	positionHistory = new HistoryKeeper(10, 1000, positionSpin, positionSlider);
	effortHistory   = new HistoryKeeper(10, 1000, effortSpin);
	velocityHistory = new HistoryKeeper(10, 1000, velocitySpin);
	
	// Configure position view
	positionSlider->setMinimum(min * ratio);
	positionSlider->setMaximum(max * ratio);
	positionSlider->setValue(0);
	positionSlider->setToolTip("Position");

	positionSpin->setMinimum(min);
	positionSpin->setMaximum(max);
	positionSpin->setValue(0);
	positionSpin->setSingleStep(0.01);
	positionSpin->setDecimals(3);
	positionSpin->setToolTip("Position");
	
	// Configure effort view
	effortSpin->setMaximum(2);
	effortSpin->setMinimum(0);
	effortSpin->setValue(0);
	effortSpin->setSingleStep(0.01);
	effortSpin->setToolTip("Effort");

	// Configure velocity view
	velocitySpin->setMaximum(6);
	velocitySpin->setMinimum(-6);
	velocitySpin->setValue(0);
	velocitySpin->setSingleStep(0.1);
	velocitySpin->setToolTip("Velocity");
	
	// Set up widgets alignment and layout
	std::vector<QWidget*> widgets;
	widgets.push_back(effortSpin);
	widgets.push_back(velocitySpin);
	widgets.push_back(positionSpin);
	widgets.push_back(positionSlider);
	
	setUpLayout(widgets, alignment);
	
	// Set up connections
	connect(positionSlider, SIGNAL(valueChanged(int)), this, SLOT(positionSliderChanged()));
	connect(positionSpin, SIGNAL(valueChanged(double)), this, SLOT(positionSpinChanged()));
	
	connect(velocitySpin, SIGNAL(valueChanged(double)), this, SLOT(handleVelocityChanged()));
	connect(effortSpin, SIGNAL(valueChanged(double)), this, SLOT(handleEffortChanged()));
}

void PosVelEffView::clearHistoryOfChanges()
{
	effortHistory->clearHistory();
	positionHistory->clearHistory();
	velocityHistory->clearHistory();
}

void PosVelEffView::setEffort(double effort)
{
	effortSpin->blockSignals(true);
	effortSpin->setValue(effort);
	effortSpin->blockSignals(false);
	
	effortHistory->valueChanged(effort);
}

void PosVelEffView::setVelocity(double velocity)
{
	velocitySpin->blockSignals(true);
	velocitySpin->setValue(velocity);
	velocitySpin->blockSignals(false);
	
	velocityHistory->valueChanged(velocity);
}

void PosVelEffView::setPosition(double rate)
{
	positionSpin->blockSignals(true);
	positionSlider->blockSignals(true);
	
	positionSpin->setValue(rate);
	positionSlider->setValue(rate * ratio);
	
	positionSpin->blockSignals(false);
	positionSlider->blockSignals(false);
	
	positionHistory->valueChanged(rate);
}

double PosVelEffView::getPosition()
{
	return positionSpin->value();
}

double PosVelEffView::getEffort()
{
	return effortSpin->value();
}

double PosVelEffView::getVelocity()
{
	return velocitySpin->value();
}

void PosVelEffView::setField(Field field, double value)
{
	if(field == PosVelEffView::POSITION)
		setPosition(value);
	else if(field == PosVelEffView::EFFORT)
		setEffort(value);
	else if(field == PosVelEffView::VELOCITY)
		setVelocity(value);
	
	fieldChanged(field, jointName);
	return;
}

void PosVelEffView::positionSliderChanged()
{
	positionSpin->blockSignals(true);
	positionSpin->setValue(positionSlider->value() / ratio);
	positionSpin->blockSignals(false);
	
	positionHistory->valueChanged(positionSpin->value());
	fieldChanged(PosVelEffView::POSITION, jointName);
	
	if(isShiftPressed() && !m_inverse_joint_name.empty())
	{
		if(onShiftMirrored)
			changeForInverse(m_inverse_joint_name, PosVelEffView::POSITION, -positionSpin->value());
		else
			changeForInverse(m_inverse_joint_name, PosVelEffView::POSITION, positionSpin->value());
	}
}

void PosVelEffView::positionSpinChanged()
{
	positionSlider->blockSignals(true);
	positionSlider->setValue(positionSpin->value() * ratio);
	positionSlider->blockSignals(false);
	
	positionHistory->valueChanged(positionSpin->value());
	fieldChanged(PosVelEffView::POSITION, jointName);
	
	if(isShiftPressed() && !m_inverse_joint_name.empty())
	{
		if(onShiftMirrored)
			changeForInverse(m_inverse_joint_name, PosVelEffView::POSITION, -positionSpin->value());
		else
			changeForInverse(m_inverse_joint_name, PosVelEffView::POSITION, positionSpin->value());
	}
}

void PosVelEffView::handleEffortChanged()
{
	effortHistory->valueChanged(effortSpin->value());
	fieldChanged(PosVelEffView::EFFORT, jointName);
	
	if(isShiftPressed() && !m_inverse_joint_name.empty())
		changeForInverse(m_inverse_joint_name, PosVelEffView::EFFORT, effortSpin->value());
}

void PosVelEffView::handleVelocityChanged()
{
	velocityHistory->valueChanged(velocitySpin->value());
	fieldChanged(PosVelEffView::VELOCITY, jointName);
	
	if(isShiftPressed() && !m_inverse_joint_name.empty())
		changeForInverse(m_inverse_joint_name, PosVelEffView::VELOCITY, velocitySpin->value());
}

PosVelEffView::~PosVelEffView()
{
	delete effortHistory;
	delete velocityHistory;
	delete positionHistory;
}
