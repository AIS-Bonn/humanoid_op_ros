#ifndef PIDVIEW_H
#define PIDVIEW_H

// Widget to edit PID of certain joint
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <QSpinBox>
#include <QRadioButton>

#include <trajectory_editor/historykeeper.h>
#include <trajectory_editor/spaces/basicsmallview.h>

#include <motion_file/motionfile.h>

class PIDView : public BasicSmallView
{
    Q_OBJECT
public:
	
	enum Field
	{
		P_VALUE,
		I_VALUE,
		D_VALUE,
		
		LIMIT,
		PID_FLAG
	};
	
	PIDView(BasicSmallView::Alignment alignment, BasicSmallView::Type type, std::string jointName, bool shiftMirrored, QWidget *parent = 0);
	~PIDView();
	
	void   setField(PIDView::Field field, double value);
	double getPGain();
	double getIGain();
	double getDGain();
	double getLimit();
	kf_player::gainSelectEnum getFlag();
	
	void clearHistoryOfChanges();
	
Q_SIGNALS:
	void fieldChanged(PIDView::Field);
	
private Q_SLOTS:
	void p_SpinChanged();
	void i_SpinChanged();
	void d_SpinChanged();
	
	void limitSpinChanged();
	
	void p_RadioChanged();
	void i_RadioChanged();
	void d_RadioChanged();

private:
	void blockRadioButtons(bool block);
	void updateBackgroundColor(QDoubleSpinBox *spin, float boundary);
	
private:
	QDoubleSpinBox *p_Spin;
	QDoubleSpinBox *i_Spin;
	QDoubleSpinBox *d_Spin;
	
	QDoubleSpinBox *limit_Spin;
	
	QRadioButton *p_Radio;
	QRadioButton *i_Radio;
	QRadioButton *d_Radio;
	
	HistoryKeeper *p_History;
	HistoryKeeper *i_History;
	HistoryKeeper *d_History;
	
	// Boundary for PID gains after what the value is considered as "large" and is colored accordingly
	static const float p_ColorBoundary;
	static const float i_ColorBoundary;
	static const float d_ColorBoundary;
	
	// StyleSheets which corespond to: zero value, value < boundaty, value >= boundary
	QString zero_sheet;
	QString medium_sheet;
	QString large_sheet;
};

#endif // PIDVIEW_H
