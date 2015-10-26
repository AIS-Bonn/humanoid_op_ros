#include <trajectory_editor_2/historykeeper.h>

HistoryKeeper::HistoryKeeper()
{
	maxStackSize = 10;
	minElapsedTime = 1000;
	
	wasRecorded = true;
	
	mainLoopTimer = new QTimer();
	elapsedTimer = new QElapsedTimer();
	
	connect(mainLoopTimer, SIGNAL(timeout()), this, SLOT(loop()));
    mainLoopTimer->start(200);
	clearHistory();
}

HistoryKeeper::HistoryKeeper(unsigned int maxStackSize_, double minElapsedTime_)
{
	maxStackSize =   maxStackSize_;
	minElapsedTime = minElapsedTime_;
	
	wasRecorded = true;
	
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
	if(stack.size() <= 0)
		return -999;
				
	if(wasRecorded && stack.size() > 1)
		stack.pop();
				
	return stack.last();
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
	
	if(stack.size() >= maxStackSize)
		stack.remove(0);

	stack.push(currentValue);
	wasRecorded = true;
	//ROS_INFO("Position Recorded after: %d", (int)effortTimer->elapsed());
}

HistoryKeeper::~HistoryKeeper()
{
	delete mainLoopTimer;
	delete elapsedTimer;
}