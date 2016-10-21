// Widget to edit rate and ange rate of certain joint
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef RATEANGLEVIEW_H
#define RATEANGLEVIEW_H

#include <trajectory_editor/historykeeper.h>
#include <trajectory_editor/spaces/basicsmallview.h>

#include <QObject>
#include <QWidget>
#include <QSpinBox>
#include <QSlider>

class RateAngleView : public BasicSmallView
{
    Q_OBJECT
public:
	
	enum Field
	{
		RATE,
		ANGLERATE,
	};
	
	RateAngleView(BasicSmallView::Alignment alignment, BasicSmallView::Type type, std::string jointName, bool shiftMirrored,  QWidget *parent = 0);
	~RateAngleView();
	
	void clearHistoryOfChanges();
	
	void setField(RateAngleView::Field field, double value);
	void setRate(double rate);
	double getRate();
	
Q_SIGNALS:
	void rateChanged(std::string jointName);
	void changeForInverse(const std::string joint_name, const RateAngleView::Field field, const double value);
	
private Q_SLOTS:
	void rateSliderChanged();
	void rateSpinChanged();
	
private:
	QSlider        *rateSlider;
	QDoubleSpinBox *rateSpin;
	QDoubleSpinBox *angleRateSpin;
	
	HistoryKeeper *rateHistory;
	HistoryKeeper *angleRateHistory;
};

#endif // RATEANGLEVIEW_H
