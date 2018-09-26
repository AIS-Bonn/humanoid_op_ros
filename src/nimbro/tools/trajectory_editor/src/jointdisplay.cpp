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
	m_perspective_name = perspective.m_name;
	
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
	if(!m_current_frame || !isEnabled())
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
	
	PosVelEffView *view = findView(jointName);
	int index = motionfile::Motion::nameToIndex(m_joint_list, jointName);
	
	if (index < 0 || view == nullptr)
		return;
	
	// HACK for dynaped
	if(m_perspective_name == "dynaped_perspective")
	{
		if(jointName == "right_thigh_pitch" || jointName == "right_shank_pitch" ||
			jointName == "left_thigh_pitch" || jointName == "left_shank_pitch")
			dynapedTransform(true);
			
		if(jointName == "right_hip_pitch" || "right_knee_pitch" ||
			"left_hip_pitch" || "left_knee_pitch")
			dynapedTransform(false);
	}
	
	// HACK for adult robot
	if(m_perspective_name == "adult_perspective" || m_perspective_name == "op2x_perspective")
	{
		if(jointName == "right_hip_pitch" || jointName == "right_knee_pitch" ||
			jointName == "left_hip_pitch" || jointName == "left_knee_pitch")
		adultTransform();
	}
				
	if(field == PosVelEffView::EFFORT)
	{
		m_current_frame->joints[index].effort = view->getEffort();
	}
	else if(field == PosVelEffView::VELOCITY)
	{
		m_current_frame->joints[index].velocity = view->getVelocity();
	}
	else if(field == PosVelEffView::POSITION)
	{
		m_current_frame->joints[index].position = view->getPosition();
		updateRobot();
	}
}

void JointManager::dynapedTransform(bool flag)
{
	PosVelEffView* rtp_joint = findView("right_thigh_pitch");
	PosVelEffView* rsp_joint = findView("right_shank_pitch");
	PosVelEffView* rhp_joint = findView("right_hip_pitch");
	PosVelEffView* rkp_joint = findView("right_knee_pitch");
		
	PosVelEffView* ltp_joint = findView("left_thigh_pitch");
	PosVelEffView* lsp_joint = findView("left_shank_pitch");
	PosVelEffView* lhp_joint = findView("left_hip_pitch");
	PosVelEffView* lkp_joint = findView("left_knee_pitch");
	
	if(rtp_joint == nullptr || rsp_joint == nullptr || rhp_joint == nullptr || rkp_joint == nullptr ||
		ltp_joint == nullptr || lsp_joint == nullptr || lhp_joint == nullptr || lkp_joint == nullptr)
		return;
	
	if(flag)
	{
		rhp_joint->setPosition(rtp_joint->getPosition());
		rkp_joint->setPosition(rsp_joint->getPosition() - rtp_joint->getPosition());
		lhp_joint->setPosition(ltp_joint->getPosition());
		lkp_joint->setPosition(lsp_joint->getPosition() - ltp_joint->getPosition());
			
		setMotionFromView(rhp_joint);
		setMotionFromView(rkp_joint);
		setMotionFromView(lhp_joint);
		setMotionFromView(lkp_joint);
	}
	else
	{
		rtp_joint->setPosition(rhp_joint->getPosition());
		rsp_joint->setPosition(rkp_joint->getPosition() + rtp_joint->getPosition());
		ltp_joint->setPosition(lhp_joint->getPosition());
		lsp_joint->setPosition(lkp_joint->getPosition() + ltp_joint->getPosition());
		
		setMotionFromView(rsp_joint);
		setMotionFromView(rtp_joint);
		setMotionFromView(lsp_joint);
		setMotionFromView(ltp_joint);
	}
}

void JointManager::adultTransform()
{
	PosVelEffView* rhp = findView("right_hip_pitch");
	PosVelEffView* rkp = findView("right_knee_pitch");
	PosVelEffView* lhp = findView("left_hip_pitch");
	PosVelEffView* lkp = findView("left_knee_pitch");
	
	int rap = motionfile::Motion::nameToIndex(m_joint_list, "right_ankle_pitch");
	int lap = motionfile::Motion::nameToIndex(m_joint_list, "left_ankle_pitch");
	
	m_current_frame->joints[rap].position = -rhp->getPosition() -rkp->getPosition();
	m_current_frame->joints[lap].position = -lhp->getPosition() -lkp->getPosition();
}

void JointManager::setMotionFromView(PosVelEffView* view)
{
	if(!m_current_frame)
		return;
	
	int index = motionfile::Motion::nameToIndex(m_joint_list, view->getJointName());
	
	if(index < 0)
		return;

	m_current_frame->joints[index].effort = view->getEffort();
	m_current_frame->joints[index].velocity = view->getVelocity();
	m_current_frame->joints[index].position = view->getPosition();
}

// Find view with requested jointName
PosVelEffView* JointManager::findView(const std::string &jointName)
{
	for (unsigned i = 0; i < jointViews.size(); i++)
		if(jointViews.at(i)->getJointName() == jointName)
			return jointViews.at(i);
		
	return nullptr;
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
