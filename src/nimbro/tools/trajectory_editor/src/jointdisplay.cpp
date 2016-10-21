// Displays joints of loaded frame
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <trajectory_editor/spaces/jointdisplay.h>

#include <QEvent>
#include <math.h>

#include <ros/package.h>
#include <ros/console.h>

JointManager::JointManager(const std::vector< std::string > &modelJointList, QWidget *parent)
 : BasicSpace(modelJointList, parent)
{
	jointsLayout = new QGridLayout();
	setLayout(jointsLayout);
	
	ROS_INFO("initialized successfully");
}

void JointManager::unsetFrame()
{
	m_current_frame = KeyframePtr();
}

void JointManager::handlePerspectiveUpdate(const joint_perspective::JointPerspective& perspective)
{
	setEnabled(perspective.m_joint_space_allowed);

	if(isEnabled())
		initGUI(perspective);
}

void JointManager::initGUI(const joint_perspective::JointPerspective& perspective)
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
													, joint.type, joint.mirror_on_shift);
		
			if(joint.alignment == BasicSmallView::NO_PAIR || joint.alignment == BasicSmallView::RIGHT)
				row++;
		}
	}
	
	jointsLayout->setMargin(0);
	jointsLayout->setSpacing(0);
	jointsLayout->setContentsMargins(0,0,0,0);
}

void JointManager::findAndPutView(std::string jointName, std::string label, int row, BasicSmallView::Alignment alignment, BasicSmallView::Type type, bool shiftMirrored)
{
	for (unsigned i = 0; i < m_joint_list.size(); i++)
	{
		if(m_joint_list.at(i) == jointName)
		{
			PosVelEffView *view = new PosVelEffView(alignment, type, jointName, shiftMirrored, this);
				
			connect(view, SIGNAL(fieldChanged(PosVelEffView::Field, std::string))
					, this, SLOT(handleFieldChanged(PosVelEffView::Field, std::string)));
			
			connect(view, SIGNAL(changeForInverse(std::string,PosVelEffView::Field,double))
					, this, SLOT(handleChangeForInverse(std::string,PosVelEffView::Field,double)));
			
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

void JointManager::updateFrame()
{
	if(!m_current_frame || isEnabled() == false)
		return;
	
	for (unsigned i = 0; i < jointViews.size(); i++)
	{
		PosVelEffView *view = jointViews.at(i);

		int index = motionfile::Motion::nameToIndex(m_joint_list, view->getJointName());
		if (index < 0)
			continue;

		double currentPos = m_current_frame->joints[index].position;
		double currentEff = m_current_frame->joints[index].effort;
		double currentVel = m_current_frame->joints[index].velocity;
		
		view->clearHistoryOfChanges();
		
		view->setPosition(currentPos);
		view->setEffort(currentEff);
		view->setVelocity(currentVel);
	}
	
	updateRobot(); // Signal that robot view should be updated
}

void JointManager::handleChangeForInverse(std::string jointName, PosVelEffView::Field field, double value)
{
	for (unsigned i = 0; i < jointViews.size(); i++)
		if(jointViews.at(i)->getJointName() == jointName)
		{
			jointViews.at(i)->setField(field, value);
			return;
		}
}

void JointManager::handleFieldChanged(PosVelEffView::Field field, std::string jointName)
{
	if (!m_current_frame)
		return;
	
	PosVelEffView *view;
	
	// Find view with requested jointName
	for (unsigned i = 0; i < jointViews.size(); i++)
	{
		view = jointViews.at(i);
		
		if(view->getJointName() == jointName)
		{
			int index = motionfile::Motion::nameToIndex(m_joint_list, view->getJointName());
			if (index < 0)
				return;
				
			if(field == PosVelEffView::EFFORT)
				m_current_frame->joints[index].effort = view->getEffort();
				
			else if(field == PosVelEffView::VELOCITY)
				m_current_frame->joints[index].velocity = view->getVelocity();
				
			else if(field == PosVelEffView::POSITION)
			{
				m_current_frame->joints[index].position = view->getPosition();
				updateRobot();
			}
			
			return;
		}
	}
}

void JointManager::createHeaderLabels(BasicSmallView::Alignment alignment, int row)
{
	QWidget *header = new QWidget(this);
	QGridLayout *layout = new QGridLayout(header);
	layout->setSpacing(10);
	
	QLabel *positionLabel = new QLabel("Position");
	QLabel *effortLabel = new QLabel("Effort");
	QLabel *velocityLabel = new QLabel("Velocity");
	
	QFont font = positionLabel->font();
	font.setBold(true);
	
	positionLabel->setFont(font);
	effortLabel->setFont(font);
	velocityLabel->setFont(font);
	
	if(alignment == BasicSmallView::LEFT)
	{
		effortLabel->setAlignment(Qt::AlignLeft);
		velocityLabel->setAlignment(Qt::AlignLeft);
		positionLabel->setAlignment(Qt::AlignLeft);
		
		effortLabel->setContentsMargins(0, 0, 8, 0);
		
		layout->addWidget(effortLabel, 0, 0);
		layout->addWidget(velocityLabel, 0, 1);
		layout->addWidget(positionLabel, 0, 2);
		layout->setAlignment(Qt::AlignLeft);
		
		header->setLayout(layout);
		jointsLayout->addWidget(header, row, 0);
	}
	else
	{
		effortLabel->setAlignment(Qt::AlignRight);
		velocityLabel->setAlignment(Qt::AlignRight);
		positionLabel->setAlignment(Qt::AlignRight);
		
		effortLabel->setContentsMargins(8, 0, 0, 0);
		
		layout->addWidget(effortLabel, 0, 2);
		layout->addWidget(velocityLabel, 0, 1);
		layout->addWidget(positionLabel, 0, 0);
		layout->setAlignment(Qt::AlignRight);
		
		header->setLayout(layout);
		jointsLayout->addWidget(header, row, 2);
	}
}
