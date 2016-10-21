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
#include <QEvent>
#include <QKeyEvent>

#include <QSlider>
#include <QDoubleSpinBox>

class HistoryKeeper : public QObject
{
    Q_OBJECT
public:
	HistoryKeeper(unsigned int maxStackSize_, double minElapsedTime_);
	HistoryKeeper(unsigned int maxStackSize_, double minElapsedTime_, QDoubleSpinBox *spin_);
	HistoryKeeper(unsigned int maxStackSize_, double minElapsedTime_, QDoubleSpinBox *spin_, QSlider *slider_);
	~HistoryKeeper();
	
	void setRecorded(bool recorded);
	
	void clearHistory();
	void valueChanged(double newValue);
	double rollBack(); // Returns previously recorded value. If there is no recorded value - returns -999
	double rollForward();
	
	virtual bool eventFilter(QObject *object, QEvent *event);
	
Q_SIGNALS:
	
private Q_SLOTS:
	void loop(); // Main loop. Is called every 0.2 seconds
	
private:
	void init(unsigned int maxStackSize_, double minElapsedTime_);
	
	void handleUndo();
	void handleRedo();
	
private:
	QTimer        *mainLoopTimer; // TODO: reimplement it without this instant loop
	QElapsedTimer *elapsedTimer; // Counts elapsed time since last edit
	double minElapsedTime; // Time which must be elapsed before recording new changes
	
	QStack<double> stack; // Stack to record history of changes
	int maxStackSize;     // How deep the history will be recorded
	int currentPosition;  // Current position in the stack of changes
	
	double currentValue;
	bool   wasRecorded; // True if currentValue was already stored into stack
	
	QDoubleSpinBox *spin;
	QSlider *slider;
	bool spinOnly;
};

#endif // HISTORYKEEPER_H
