// Widget to apply rules in quick way
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef RULEVIEW_H
#define RULEVIEW_H

#include <trajectory_editor/ruleslider.h>
#include <motion_file/motionfile.h>
#include <QWidget>

namespace Ui
{
	class RuleView; 
}

class RuleView : public QWidget
{
Q_OBJECT
public:
	RuleView(QWidget *parent = 0);
	virtual ~RuleView();
	
public Q_SLOTS:
	void handleRulesLoaded(const std::vector<motionfile::Rule> &rules);
	
Q_SIGNALS:
	void applyRule(double delta, int rule_id);
	
private Q_SLOTS:
	void handleApplyRule(double delta, int rule_id);
	
private:
	Ui::RuleView *m_ui;
};


#endif // RULEVIEW_H
