// Widget to edit rate and ange rate of certain joint
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <trajectory_editor/spaces/rateangleview.h>

#include <QLabel>
#include <QEvent>
#include <QKeyEvent>

#include <ros/package.h>
#include <ros/console.h>

const double ratio = 10000;

RateAngleView::RateAngleView(BasicSmallView::Alignment alignment, BasicSmallView::Type type, std::string jointName, bool shiftMirrored, QWidget *parent)
	: BasicSmallView(alignment, type, jointName, shiftMirrored, parent)
{
	rateSlider    = new QSlider(Qt::Horizontal);
	rateSpin      = new QDoubleSpinBox();
	angleRateSpin = new QDoubleSpinBox();
	
	rateHistory      = new HistoryKeeper(10, 1000, rateSpin, rateSlider);
	angleRateHistory = new HistoryKeeper(10, 1000, angleRateSpin);
	
	// Set up Rate
	rateSlider->setMinimum(min*ratio);
	rateSlider->setMaximum(max*ratio);
	rateSlider->setValue(0);
	rateSlider->setToolTip("Rate");

	rateSpin->setMinimum(min);
	rateSpin->setMaximum(max);
	rateSpin->setDecimals(3);
	rateSpin->setValue(0);
	rateSpin->setSingleStep(0.01);
	rateSpin->setToolTip("Rate");
	
	// Set up Angle Rate
	// Temporaly unused!
	angleRateSpin->setMaximum(0);
	angleRateSpin->setMinimum(0);
	angleRateSpin->setValue(0);
	angleRateSpin->setSingleStep(0.1);
	angleRateSpin->setToolTip("Angle Rate");
	
	// Set up layout and alignmnet
	std::vector<QWidget*> widgets;
	widgets.push_back(angleRateSpin);
	widgets.push_back(rateSpin);
	widgets.push_back(rateSlider);

	setUpLayout(widgets, alignment);
	
	connect(rateSlider, SIGNAL(valueChanged(int)), this, SLOT(rateSliderChanged()));
	connect(rateSpin, SIGNAL(valueChanged(double)), this, SLOT(rateSpinChanged()));
}

void RateAngleView::clearHistoryOfChanges()
{
	rateHistory->clearHistory();
	angleRateHistory->clearHistory();
}

void RateAngleView::rateSliderChanged()
{
	rateSpin->blockSignals(true);
	rateSpin->setValue(rateSlider->value() / ratio);
	rateSpin->blockSignals(false);
	
	rateHistory->valueChanged(rateSpin->value());
	rateChanged(jointName);
	
	if(isShiftPressed() && !m_inverse_joint_name.empty())
	{
		if(onShiftMirrored)
			changeForInverse(m_inverse_joint_name, RateAngleView::RATE, -rateSpin->value());
		else
			changeForInverse(m_inverse_joint_name, RateAngleView::RATE, rateSpin->value());
	}
}

void RateAngleView::rateSpinChanged()
{
	rateSlider->blockSignals(true);
	rateSlider->setValue(rateSpin->value() * ratio);
	rateSlider->blockSignals(false);
	
	rateHistory->valueChanged(rateSpin->value());
	rateChanged(jointName);
	
	if(isShiftPressed() && !m_inverse_joint_name.empty())
	{
		if(onShiftMirrored)
			changeForInverse(m_inverse_joint_name, RateAngleView::RATE, -rateSpin->value());
		else
			changeForInverse(m_inverse_joint_name, RateAngleView::RATE, rateSpin->value());
	}
}

void RateAngleView::setField(RateAngleView::Field field, double value)
{
	if(field == RateAngleView::RATE)
		setRate(value);
}

void RateAngleView::setRate(double rate)
{
	rateSpin->blockSignals(true);
	rateSlider->blockSignals(true);
	
	rateSpin->setValue(rate);
	rateSlider->setValue(rate * ratio);
	
	rateSpin->blockSignals(false);
	rateSlider->blockSignals(false);
	
	rateHistory->valueChanged(rate);
}

double RateAngleView::getRate()
{
	return rateSpin->value();
}

RateAngleView::~RateAngleView()
{
	delete rateHistory;
	delete angleRateHistory;
}