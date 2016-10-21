#include <trajectory_editor/historykeeper.h>

#include <stdio.h>

HistoryKeeper::HistoryKeeper(unsigned int maxStackSize_, double minElapsedTime_, QDoubleSpinBox *spin_)
{
	init(maxStackSize_, minElapsedTime_);
	
	spin = spin_;
	spin->installEventFilter(this);
	
	spinOnly = true;
}

HistoryKeeper::HistoryKeeper(unsigned int maxStackSize_, double minElapsedTime_, QDoubleSpinBox *spin_, QSlider *slider_)
{
	init(maxStackSize_, minElapsedTime_);
	
	spin = spin_;
	spin->installEventFilter(this);
	
	slider = slider_;
	slider->installEventFilter(this);
	
	spinOnly = false;
}

HistoryKeeper::HistoryKeeper(unsigned int maxStackSize_, double minElapsedTime_)
{
	init(maxStackSize_, minElapsedTime_);
}

void HistoryKeeper::init(unsigned int maxStackSize_, double minElapsedTime_)
{
	maxStackSize =   maxStackSize_;
	minElapsedTime = minElapsedTime_;
	
	wasRecorded = true;
	currentPosition = 0;
	
	mainLoopTimer = new QTimer();
	elapsedTimer = new QElapsedTimer();
	
	connect(mainLoopTimer, SIGNAL(timeout()), this, SLOT(loop()));
    mainLoopTimer->start(200);
	clearHistory();
}

void HistoryKeeper::clearHistory()
{
	stack.clear();
	wasRecorded = false;
	elapsedTimer->restart();
}

void HistoryKeeper::setRecorded(bool recorded)
{
	wasRecorded = recorded;
}

double HistoryKeeper::rollBack()
{
	if(currentPosition <= 0)
		return -999;
	
	currentPosition--;
	return stack.at(currentPosition);
}

double HistoryKeeper::rollForward()
{
	if(currentPosition >= stack.size()-1)
		return -999;
	
	currentPosition++;
	return stack.at(currentPosition);
}

void HistoryKeeper::valueChanged(double newValue)
{
	currentValue = newValue;
	
	elapsedTimer->restart();
	wasRecorded = false;
}

void HistoryKeeper::loop()
{
	if(elapsedTimer->elapsed() < minElapsedTime || wasRecorded)
		return;
	
	if(currentPosition < stack.size()-1)
	{
		while(currentPosition != stack.size()-1)
		{
			stack.pop();
		}
	}
	
	if(stack.size() >= maxStackSize)
		stack.remove(0);

	stack.push(currentValue);
	currentPosition = stack.size()-1;
	wasRecorded = true;
}

void HistoryKeeper::handleRedo()
{
	double value = rollForward();
				
	if(value == -999)
		return;
				
	spin->setValue(value);
	setRecorded(true);
}

void HistoryKeeper::handleUndo()
{		
	double value = rollBack();
				
	if(value == -999)
		return;
				
	spin->setValue(value);
	setRecorded(true);
}

bool HistoryKeeper::eventFilter(QObject *object, QEvent *event)
{
	if(event->type() != QEvent::KeyPress)
		return false;
	
	QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
	
	// Redo
	if(keyEvent->modifiers().testFlag(Qt::ControlModifier) && keyEvent->modifiers().testFlag(Qt::ShiftModifier)
		&& keyEvent->key() == Qt::Key_Z) 
	{
		if(spinOnly)
		{
			if(spin->hasFocus())
				handleRedo();
		}
		else
		{
			if(spin->hasFocus() || slider->hasFocus())
				handleRedo();
		}
		
		return false;
	}
		
	// Undo
	if(keyEvent->key() == Qt::Key_Z && keyEvent->modifiers() == Qt::ControlModifier) 
	{
		if(spinOnly)
		{
			if(spin->hasFocus())
				handleUndo();
		}
		else
		{
			if(spin->hasFocus() || slider->hasFocus())
				handleUndo();
		}
	}
	
    return false;
}

HistoryKeeper::~HistoryKeeper()
{
	delete mainLoopTimer;
	delete elapsedTimer;
}