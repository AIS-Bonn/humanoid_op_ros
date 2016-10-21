#ifndef POSITIONVIEW_H
#define POSITIONVIEW_H

// Widget to edit position of certain joint
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <QObject>
#include <QWidget>
#include <QSpinBox>
#include <QSlider>

class PositionView : public QWidget
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
	
	PositionView(PositionView::Alignment alignment, PositionView::Type type, std::string jointName, QWidget *parent = 0);
	~PositionView();
	
	void setPosition(double position);
	double getPosition();
	
	bool withinRange(double value);
	
	virtual bool eventFilter(QObject *object, QEvent *event);
	
	QSlider        *positionSlider;
	QDoubleSpinBox *positionSpin;
	
	std::string jointName;
	
Q_SIGNALS:
	void positionChanged(const std::string joint_name, const double prev, const double current);
	
private Q_SLOTS:
	void positionSliderChanged();
	void positionSpinChanged();
	
private:
	double min;
	double max;
	
	double m_prev_value;
};

#endif // POSITIONVIEW_H
