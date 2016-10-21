// Widget to control one rule 
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef RULESLIDER_H
#define RULESLIDER_H

#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QLabel>

class RuleSlider : public QWidget
{
Q_OBJECT
public:
	RuleSlider(const int rule_id, const std::string name, QWidget *parent = 0);
	virtual ~RuleSlider();
	
Q_SIGNALS:
	void applyRule(double delta, int rule_id);
	
private Q_SLOTS:
	void sliderChanged();
	void spinChanged();
	void setZero();
	
private:
	void handleValueChange(double new_value);
	
private:
	QHBoxLayout    *m_layout;
	QDoubleSpinBox *m_spin;
	QSlider        *m_slider;
	QLabel         *m_name_label;
	QPushButton    *m_zero_button;
	
	int m_rule_id;
	double m_prev_value;
};


#endif // RULESLIDER_H
