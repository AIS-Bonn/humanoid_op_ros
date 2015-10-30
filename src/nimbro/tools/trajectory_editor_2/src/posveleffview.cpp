#include <trajectory_editor_2/posveleffview.h>

#include <QGridLayout>
#include <QLabel>
#include <QEvent>
#include <QKeyEvent>
#include <QApplication>

#include <ros/package.h>
#include <ros/console.h>

#include <math.h>

const double ratio = 10000; // Ratio to convert between position slider/spin

PosVelEffView::PosVelEffView(PosVelEffView::Alignment alignment, std::string  jointName, int id, QWidget *parent)
	: QWidget(parent)
{
	// Init members
	this->jointName = jointName;
	this->id = id;
	
	onShiftMirrored = false;
	
	if(alignment == NO_PAIR)
		connectedID = id;
	else if(alignment == LEFT)
		connectedID = ++id;
	else if(alignment == RIGHT)
		connectedID = --id;
	
	min = -M_PI;
	max = M_PI;
	
	effortHistory   = new HistoryKeeper(10, 1000);
	positionHistory = new HistoryKeeper(10, 1000);
	velocityHistory = new HistoryKeeper(10, 1000);
	
	positionSlider = new QSlider(Qt::Horizontal);
	positionSpin   = new QDoubleSpinBox();
	effortSpin     = new QDoubleSpinBox();
	velocitySpin   = new QDoubleSpinBox();
	
	QGridLayout *layout = new QGridLayout(this);
	
	// Configure position view
	positionSlider->setMinimum(min * ratio);
	positionSlider->setMaximum(max * ratio);
	positionSlider->setValue(0);
	positionSlider->setToolTip("Position");
	positionSlider->installEventFilter(this);

	positionSpin->setMinimum(min);
	positionSpin->setMaximum(max);
	positionSpin->setValue(0);
	positionSpin->setSingleStep(0.01);
	positionSpin->setToolTip("Position");
	positionSpin->installEventFilter(this);
	
	// Configure effort view
	effortSpin->setMaximum(1);
	effortSpin->setMinimum(0);
	effortSpin->setValue(0);
	effortSpin->setSingleStep(0.01);
	effortSpin->setToolTip("Effort");
	effortSpin->installEventFilter(this);

	// Configure velocity view
	velocitySpin->setMaximum(6);
	velocitySpin->setMinimum(-6);
	velocitySpin->setValue(0);
	velocitySpin->setSingleStep(0.1);
	velocitySpin->setToolTip("Velocity");
	velocitySpin->installEventFilter(this);
	
	// Set up widgets alignment and layout
	if(alignment == LEFT || alignment == NO_PAIR)
	{
		layout->addWidget(effortSpin, 1, 0);
		layout->addWidget(velocitySpin, 1, 1);
		layout->addWidget(positionSpin, 1, 2);
		layout->addWidget(positionSlider, 1, 3);
	}
	else if(alignment == RIGHT)
	{
		layout->addWidget(effortSpin, 1, 3);
		layout->addWidget(velocitySpin, 1, 2);
		layout->addWidget(positionSpin, 1, 1);
		layout->addWidget(positionSlider, 1, 0);
	}

	layout->setContentsMargins(5,0,5,0);
	this->setLayout(layout);
	
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

int PosVelEffView::getID()
{
	return id;
}

bool PosVelEffView::isShiftPressed()
{
	Qt::KeyboardModifiers modifier = QApplication::keyboardModifiers();
	return modifier == Qt::ShiftModifier;
}

void PosVelEffView::setOnShiftMirrored(bool mirrored)
{
	onShiftMirrored = mirrored;
}

void PosVelEffView::setField(Field field, float value)
{
	if(field == PosVelEffView::POSITION)
	{
		setPosition(value);
		positionChanged();
		return;
	}
	else if(field == PosVelEffView::EFFORT)
	{
		setEffort(value);
		effortChanged();
		return;
	}
	else if(field == PosVelEffView::VELOCITY)
	{
		setVelocity(value);
		velocityChanged();
		return;
	}
}

void PosVelEffView::positionSliderChanged()
{
	positionSpin->blockSignals(true);
	positionSpin->setValue(positionSlider->value() / ratio);
	positionSpin->blockSignals(false);
	
	positionHistory->valueChanged(positionSpin->value());
	positionChanged();
	
	if(isShiftPressed() && (id != connectedID))
	{
		if(onShiftMirrored)
			changeForID(connectedID, PosVelEffView::POSITION, -positionSpin->value());
		else
			changeForID(connectedID, PosVelEffView::POSITION, positionSpin->value());
	}
}

void PosVelEffView::positionSpinChanged()
{
	positionSlider->blockSignals(true);
	positionSlider->setValue(positionSpin->value() * ratio);
	positionSlider->blockSignals(false);
	
	positionHistory->valueChanged(positionSpin->value());
	positionChanged();
	
	if(isShiftPressed() && id != connectedID)
	{
		if(onShiftMirrored)
			changeForID(connectedID, PosVelEffView::POSITION, -positionSpin->value());
		else
			changeForID(connectedID, PosVelEffView::POSITION, positionSpin->value());
	}
}

void PosVelEffView::handleEffortChanged()
{
	effortHistory->valueChanged(effortSpin->value());
	effortChanged();
	
	if(isShiftPressed() && id != connectedID)
		changeForID(connectedID, PosVelEffView::EFFORT, effortSpin->value());
}

void PosVelEffView::handleVelocityChanged()
{
	velocityHistory->valueChanged(velocitySpin->value());
	velocityChanged();

	if(isShiftPressed() && id != connectedID)
		changeForID(connectedID, PosVelEffView::VELOCITY, velocitySpin->value());
}

bool PosVelEffView::eventFilter(QObject *object, QEvent *event)
{
	if(event->type() == QEvent::KeyPress)
	{
		QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
		
		if(keyEvent->key() == Qt::Key_Z && keyEvent->modifiers() == Qt::ControlModifier) // Undo
		{
			if(effortSpin->hasFocus())
			{
				ROS_INFO("Ctrl + Z on effort");
				
				double effort = effortHistory->rollBack();
				
				if(effort == -999)
					return false;
				
				effortSpin->setValue(effort);
				effortHistory->setRecorded(true);
			}
			else if(positionSpin->hasFocus() || positionSlider->hasFocus())
			{
				ROS_INFO("Ctrl + Z on position");
				
				double position = positionHistory->rollBack();
				
				if(position == -999)
					return false;
				
				positionSpin->setValue(position);
				positionHistory->setRecorded(true);
			}
			else if(velocitySpin->hasFocus())
			{
				double velocity = velocityHistory->rollBack();
				
				if(velocity == -999)
					return false;
				
				velocitySpin->setValue(velocity);
				velocityHistory->setRecorded(true);
			}
		}
	}
	
    if(event->type() == QEvent::Wheel) // Ignore wheel scrolling
	{
		if(object == positionSlider || object == positionSpin || object == effortSpin || object == velocitySpin)
			return true;
    }
    
    return false;
}

PosVelEffView::~PosVelEffView()
{
	delete effortHistory;
	delete velocityHistory;
	delete positionHistory;
}