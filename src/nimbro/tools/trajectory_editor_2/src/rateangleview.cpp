#include <trajectory_editor_2/rateangleview.h>

#include <QGridLayout>
#include <QLabel>
#include <QEvent>
#include <QKeyEvent>

#include <math.h>
#include <ros/package.h>
#include <ros/console.h>

const double angToTic = 1000.0 / M_PI;
const double ticToAng = M_PI / 1000.0;
const double ratio = 10000;

RateAngleView::RateAngleView(RateAngleView::Alignment alignment, RateAngleView::Type type
, std::string jointName, QWidget *parent)
	: QWidget(parent)
{
	this->jointName = jointName;
	
	rateHistory = new HistoryKeeper(10, 1000);
	angleRateHistory = new HistoryKeeper(10, 1000);
	
	QGridLayout *layout = new QGridLayout(this);
	
	rateSlider = new QSlider(Qt::Horizontal);
	rateSpin = new QDoubleSpinBox();
	angleRateSpin = new QDoubleSpinBox();
	
	if(type == RateAngleView::REGULAR)
	{
		min = -M_PI;
		max = M_PI;
	}
	else if(type == RateAngleView::EXTENSION)
	{
		min = 0;
		max = 1;
	}
	else if(type == RateAngleView::LEG)
	{
		min = -M_PI/2;
		max = M_PI/2;
	}
	
	rateSlider->setMinimum(min*ratio);
	rateSlider->setMaximum(max*ratio);
	rateSlider->setValue(0);
	rateSlider->setToolTip("Rate");
	rateSlider->installEventFilter(this);

	rateSpin->setMinimum(min);
	rateSpin->setMaximum(max);
	rateSpin->setValue(0);
	rateSpin->setSingleStep(0.01);
	rateSpin->setToolTip("Rate");
	rateSpin->installEventFilter(this);

	angleRateSpin->setMaximum(0);
	angleRateSpin->setMinimum(0);
	angleRateSpin->setValue(0);
	angleRateSpin->setSingleStep(0.1);
	angleRateSpin->setToolTip("Angle Rate");
	angleRateSpin->installEventFilter(this);
	
	if(alignment == LEFT)
	{
		layout->addWidget(angleRateSpin, 1, 1);
		layout->addWidget(rateSpin, 1, 2);
		layout->addWidget(rateSlider, 1, 3);
	}
	else
	{
		layout->addWidget(angleRateSpin, 1, 2);
		layout->addWidget(rateSpin, 1, 1);
		layout->addWidget(rateSlider, 1, 0);
	}

	layout->setContentsMargins(5,0,5,0);
	this->setLayout(layout);
	
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
	rateChanged();
}

void RateAngleView::rateSpinChanged()
{
	rateSlider->blockSignals(true);
	rateSlider->setValue(rateSpin->value() * ratio);
	rateSlider->blockSignals(false);
	
	rateHistory->valueChanged(rateSpin->value());
	rateChanged();
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

bool RateAngleView::eventFilter(QObject *object, QEvent *event)
{
	if(event->type() == QEvent::KeyPress)
	{
		QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
		
		if(keyEvent->key() == Qt::Key_Z && keyEvent->modifiers() == Qt::ControlModifier) // Undo
		{
			if(rateSpin->hasFocus() || rateSlider->hasFocus())
			{
				ROS_INFO("Ctrl + Z on rate");
				
				double rate = rateHistory->rollBack();
				
				if(rate == -999)
					return false;
				
				rateSpin->setValue(rate);
				rateHistory->setRecorded(true);
			}
			else if(angleRateSpin->hasFocus())
			{
				ROS_INFO("Ctrl + Z on angle rate");
				
				double angleRate = angleRateHistory->rollBack();
				
				if(angleRate == -999)
					return false;
				
				angleRateSpin->setValue(angleRate);
				angleRateHistory->setRecorded(true);
			}
		}
	}
	
    if(event->type() == QEvent::Wheel) // Ignore wheel scrolling
	{
		if(object == rateSlider || object == rateSpin || object == angleRateSpin)
			return true;
    }
    
    return false;
}

RateAngleView::~RateAngleView()
{
	delete rateHistory;
	delete angleRateHistory;
}