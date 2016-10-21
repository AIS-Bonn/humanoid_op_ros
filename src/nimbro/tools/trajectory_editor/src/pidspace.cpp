// Displays PID of loaded frame
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <trajectory_editor/spaces/pidspace.h>

#include <QEvent>
#include <math.h>

#include <ros/package.h>
#include <ros/console.h>

PIDSpace::PIDSpace(const std::vector< std::string > &modelJointList, QWidget *parent)
 : BasicSpace(modelJointList, parent)
{
	jointsLayout = new QGridLayout();
	setLayout(jointsLayout);
	
	ROS_INFO("initialized successfully");
}

void PIDSpace::handlePerspectiveUpdate(const joint_perspective::JointPerspective& perspective)
{
	setEnabled(perspective.m_joint_space_allowed);

	if(isEnabled())
		initGUI(perspective);
}

void PIDSpace::initGUI(const joint_perspective::JointPerspective& perspective)
{
	// Clear layout
	clearlayout(jointsLayout);
	jointViews.clear();

	// Init labels for joints
	QLabel *jointsLabel = new QLabel("Joint");
	jointsLabel->setAlignment(Qt::AlignCenter);
	
	QFont font = jointsLabel->font();
	font.setBold(true);
	jointsLabel->setFont(font);
	
	jointsLayout->addWidget(jointsLabel, 0, 1);
	
	createHeaderLabels(BasicSmallView::LEFT, 0);
	createHeaderLabels(BasicSmallView::RIGHT, 0);
	
	// Init joints
	int row = 1;
	BasicSmallView::Type type = BasicSmallView::MINUS_ONE_PLUS_ONE;
	
	for(unsigned i = 0; i < perspective.m_joints.size(); i++)
	{
		const joint_perspective::Joint joint = perspective.m_joints.at(i);
		
		if(joint.spacer) // Put spacer
		{
			for(int i = 0; i < 3; i++) 
				jointsLayout->addWidget(createLine(), row, i);
			
			row++;
			continue;
		}
		else // Put joint
		{
			findAndPutView(joint.joint_name, joint.visual_name, row, joint.alignment
													, type, joint.mirror_on_shift);
		
			if(joint.alignment == BasicSmallView::NO_PAIR || joint.alignment == BasicSmallView::RIGHT)
				row++;
		}
	}
	
	jointsLayout->setMargin(0);
	jointsLayout->setSpacing(0);
	jointsLayout->setContentsMargins(0,0,0,0);
}

void PIDSpace::handleFieldChanged(PIDView::Field field)
{
	if (!m_current_frame)
		return;

	for (unsigned i = 0; i < jointViews.size(); i++)
	{
		PIDView *view = jointViews.at(i);
		
		int index = motionfile::Motion::nameToIndex(m_joint_list, view->getJointName());
		if (index < 0)
			continue;
		
		if(field == PIDView::P_VALUE)
			m_current_frame->joints[index].pGain = view->getPGain();
		
		else if(field == PIDView::I_VALUE)
			m_current_frame->joints[index].iGain = view->getIGain();
		
		else if(field == PIDView::D_VALUE)
			m_current_frame->joints[index].dGain = view->getDGain();
		
		else if(field == PIDView::LIMIT)
			m_current_frame->joints[index].limit = view->getLimit();
		
		else if(field == PIDView::PID_FLAG)
			m_current_frame->joints[index].gainSelect = view->getFlag();
	}
}

void PIDSpace::findAndPutView(std::string jointName, std::string label, int row, BasicSmallView::Alignment alignment, BasicSmallView::Type type, bool shiftMirrored)
{
	for (unsigned i = 0; i < m_joint_list.size(); i++)
	{
		if(m_joint_list.at(i) == jointName)
		{
			PIDView *view = new PIDView(alignment, type, jointName, shiftMirrored, this);
				
			connect(view, SIGNAL(fieldChanged(PIDView::Field))
					, this, SLOT(handleFieldChanged(PIDView::Field)));
			
			if(alignment == BasicSmallView::LEFT || alignment == BasicSmallView::NO_PAIR)
				jointsLayout->addWidget(view, row, 0);
			else
				jointsLayout->addWidget(view, row, 2);
			
			QLabel *nameLabel = new QLabel();
			nameLabel->setAlignment(Qt::AlignCenter);
			nameLabel->setText(QString::fromStdString(label));

			jointsLayout->addWidget(nameLabel, row, 1);	
			jointViews.push_back(view);
			return;
		}
	}
	
	// Warning
	jointsLayout->addWidget(new QLabel("Error on " + QString::fromStdString(jointName)), row, 1);	
}

void PIDSpace::updateFrame()
{
	for (unsigned i = 0; i < jointViews.size(); i++)
	{
		PIDView *view = jointViews.at(i);

		int index = motionfile::Motion::nameToIndex(m_joint_list, view->getJointName());
		if (index < 0)
			continue;
		
		view->clearHistoryOfChanges();
		
		view->setField(PIDView::P_VALUE, m_current_frame->joints[index].pGain);
		view->setField(PIDView::I_VALUE, m_current_frame->joints[index].iGain);
		view->setField(PIDView::D_VALUE, m_current_frame->joints[index].dGain);
		view->setField(PIDView::LIMIT, m_current_frame->joints[index].limit);
		view->setField(PIDView::PID_FLAG, (int)m_current_frame->joints[index].gainSelect);
	}
}

void PIDSpace::createHeaderLabels(BasicSmallView::Alignment alignment, int row)
{
	QWidget *header = new QWidget(this);
	QGridLayout *layout = new QGridLayout(header);
	layout->setSpacing(10);
	
	QLabel *p_Label = new QLabel("P");
	QLabel *i_Label = new QLabel("I");
	QLabel *d_Label = new QLabel("D");
	QLabel *limit_Label = new QLabel("Limit");
	
	QFont font = p_Label->font();
	font.setBold(true);
	
	p_Label->setFont(font);
	i_Label->setFont(font);
	d_Label->setFont(font);
	limit_Label->setFont(font);
	
	if(alignment == BasicSmallView::LEFT)
	{
		p_Label->setAlignment(Qt::AlignLeft);
		i_Label->setAlignment(Qt::AlignLeft);
		d_Label->setAlignment(Qt::AlignLeft);
		limit_Label->setAlignment(Qt::AlignLeft);
		
		layout->addWidget(p_Label, 0, 0);
		layout->addWidget(i_Label, 0, 1);
		layout->addWidget(d_Label, 0, 2);
		layout->addWidget(limit_Label, 0, 3);
		layout->setAlignment(Qt::AlignLeft);
		
		header->setLayout(layout);
		jointsLayout->addWidget(header, row, 0);
	}
	else
	{
		p_Label->setAlignment(Qt::AlignRight);
		i_Label->setAlignment(Qt::AlignRight);
		d_Label->setAlignment(Qt::AlignRight);
		limit_Label->setAlignment(Qt::AlignRight);
		
		layout->addWidget(p_Label, 0, 3);
		layout->addWidget(i_Label, 0, 2);
		layout->addWidget(d_Label, 0, 1);
		layout->addWidget(limit_Label, 0, 0);
		layout->setAlignment(Qt::AlignRight);
		
		header->setLayout(layout);
		jointsLayout->addWidget(header, row, 2);
	}
	
	int margin = 22;
	
	p_Label->setContentsMargins(margin, 0, margin, 0);
	i_Label->setContentsMargins(margin, 0, margin, 0);
	d_Label->setContentsMargins(margin, 0, margin, 0);
	limit_Label->setContentsMargins(margin, 0, margin, 0);
}
