#ifndef RATEANGLEVIEW_H
#define RATEANGLEVIEW_H

// Widget to edit rate and ange rate of certain joint
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <trajectory_editor_2/historykeeper.h>

#include <QObject>
#include <QWidget>
#include <QSpinBox>
#include <QSlider>

class RateAngleView : public QWidget
{
    Q_OBJECT
public:
	
	enum Alignment // Left to right of right to left spins/sliders order
	{
		LEFT,
		RIGHT
	};
	
	enum Type // Defines intervals for spins/sliders
	{
		REGULAR,
		EXTENSION,
		LEG
	};
	
	RateAngleView(RateAngleView::Alignment alignment, RateAngleView::Type type, std::string jointName, QWidget *parent = 0);
	~RateAngleView();
	
	void clearHistoryOfChanges();
	void setRate(double rate);
	double getRate();
	
	virtual bool eventFilter(QObject *object, QEvent *event);
	
	// TODO put these to private
	QSlider        *rateSlider;
	QDoubleSpinBox *rateSpin;
	QDoubleSpinBox *angleRateSpin;
	
	std::string jointName;
	
Q_SIGNALS:
	void rateChanged();
	
private Q_SLOTS:
	void rateSliderChanged();
	void rateSpinChanged();
	
private:
	double min;
	double max;
	
	HistoryKeeper *rateHistory;
	HistoryKeeper *angleRateHistory;
};

#endif // RATEANGLEVIEW_H
