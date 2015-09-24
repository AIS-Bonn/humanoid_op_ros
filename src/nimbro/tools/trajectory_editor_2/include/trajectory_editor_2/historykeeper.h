#ifndef HISTORYKEEPER_H
#define HISTORYKEEPER_H

// Class which handles recording of history of changes of some variable
// In order to use it - call "valueChanged(double newValue)" every time your variable changes its value
// Call "double rollBack()" to do "Ctrl+z". You will get previous recorded value of variable
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <QObject>
#include <QStack>
#include <QElapsedTimer>
#include <QTimer>

class HistoryKeeper : public QObject
{
    Q_OBJECT
public:
	
	HistoryKeeper();
	HistoryKeeper(unsigned int maxStackSize_, double minElapsedTime_);
	~HistoryKeeper();
	
	void setRecorded(bool recorded);
	
	void clearHistory();
	void valueChanged(double newValue);
	double rollBack(); // Returns previously recorded value. If there is no recorded value - returns -999
	
Q_SIGNALS:
	
private Q_SLOTS:
	void loop(); // Main loop. Is called every 0.2 seconds
	
private:
	QTimer        *mainLoopTimer;
	QElapsedTimer *elapsedTimer; // Counts elapsed time since last edit
	
	QStack<double> stack; // Stack to record history of changes
	
	double currentValue;
	bool   wasRecorded; // True if currentValue was already stored into stack
	
	int maxStackSize;   // How deep the history will be recorded
	double       minElapsedTime; // Time which must be elapced before recording new changes
};

#endif // HISTORYKEEPER_H
