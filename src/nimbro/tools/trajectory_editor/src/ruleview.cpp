// Widget to apply rule in quick way
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include "ui_ruleview.h"
#include <trajectory_editor/ruleview.h>

RuleView::RuleView(QWidget* parent) : QWidget(parent)
{
	m_ui = new Ui::RuleView;
	m_ui->setupUi(this);
	setEnabled(false);
}

void RuleView::handleRulesLoaded(const std::vector<motionfile::Rule> &rules)
{
	// Clear layout
	if(m_ui->main_layout != NULL)
	{
		QLayoutItem* item;
		while ( (item = m_ui->main_layout->takeAt(1)) != NULL)
		{
			delete item->widget();
			delete item;
		}
	}
	
	// Check if there are rules in motion
	if(rules.size() <= 0)
	{
		m_ui->status_label->setText("There are no rules in this motion");
		setEnabled(false);
		return;
	}
	
	m_ui->status_label->setText("Rules are ready to use");
	setEnabled(true);
	
	// Set up necessary amount of sliders (one for each rule)
	for(size_t i = 0; i < rules.size(); i++)
	{
		RuleSlider *slider = new RuleSlider(i, rules[i].name, this);
		connect(slider, SIGNAL(applyRule(double,int)), this, SLOT(handleApplyRule(double,int)));
		m_ui->main_layout->addWidget(slider);
	}
}

void RuleView::handleApplyRule(double delta, int rule_id)
{
	applyRule(delta, rule_id);
}

RuleView::~RuleView()
{

}
