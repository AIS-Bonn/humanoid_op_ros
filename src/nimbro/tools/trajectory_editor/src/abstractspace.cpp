#include <trajectory_editor/spaces/abstractspace.h>

#include <ros/package.h>
#include <ros/console.h>

AbstractSpace::AbstractSpace(const std::vector<std::string> &jointList, QWidget *parent) 
: BasicSpace(jointList, parent)
{
	abstractPose = new gait::AbstractPose();
	
	jointsLayout = new QGridLayout();
	setLayout(jointsLayout);
	initViews();
	
	ROS_INFO("initialized successfully");
}

AbstractSpace::~AbstractSpace()
{
	delete abstractPose;
}

void AbstractSpace::handlePerspectiveUpdate(const joint_perspective::JointPerspective& perspective)
{
	setEnabled(perspective.m_abstract_space_allowed);
}

void AbstractSpace::updateFrame()
{
	if(!m_current_frame || isEnabled() == false)
		return;
	
	// Update head
	int headID = motionfile::Motion::nameToIndex(m_joint_list, "head_pitch");
	int neckID = motionfile::Motion::nameToIndex(m_joint_list, "neck_yaw");
	
	updateRate(m_current_frame->joints[headID].position, "head_pitch");
	updateRate(m_current_frame->joints[neckID].position, "neck_yaw");
	
	abstractPose->setFromJointPose(p.getJointPose(m_current_frame, m_joint_list));
	
	// Update left arm
	updateRate(abstractPose->leftArm.angleY, "left_shoulder_pitch");
	updateRate(abstractPose->leftArm.angleX, "left_shoulder_roll");
	updateRate(abstractPose->leftArm.extension, "left_arm_extension");
	
	// Update right arm
	updateRate(abstractPose->rightArm.angleY, "right_shoulder_pitch");
	updateRate(abstractPose->rightArm.angleX, "right_shoulder_roll");
	updateRate(abstractPose->rightArm.extension, "right_arm_extension");
	
	// Update left leg
	updateRate(abstractPose->leftLeg.angleZ, "left_hip_yaw");
	updateRate(abstractPose->leftLeg.angleX, "left_hip_roll");
	updateRate(abstractPose->leftLeg.angleY, "left_hip_pitch");
	updateRate(abstractPose->leftLeg.extension, "left_leg_extension");
	updateRate(abstractPose->leftLeg.footAngleY, "left_ankle_pitch");
	updateRate(abstractPose->leftLeg.footAngleX, "left_ankle_roll");
	
	// Update right leg
	updateRate(abstractPose->rightLeg.angleZ, "right_hip_yaw");
	updateRate(abstractPose->rightLeg.angleX, "right_hip_roll");
	updateRate(abstractPose->rightLeg.angleY, "right_hip_pitch");
	updateRate(abstractPose->rightLeg.extension, "right_leg_extension");
	updateRate(abstractPose->rightLeg.footAngleY, "right_ankle_pitch");
	updateRate(abstractPose->rightLeg.footAngleX, "right_ankle_roll");
}

void AbstractSpace::updateRate(double rate, std::string jointName)
{
	for(unsigned i = 0; i < jointViews.size(); i++)
	{
		RateAngleView *view = jointViews.at(i);
		
		if(view->getJointName() == jointName) // Update position
		{
			view->clearHistoryOfChanges();
			view->setRate(rate);
			break;
		}
	}
}

void AbstractSpace::updateRateFromUI(std::string name, double rate)
{
	if(name == "head_pitch" || name == "neck_yaw")
	{
		int id = motionfile::Motion::nameToIndex(m_joint_list, name);
		m_current_frame->joints[id].position = rate;
	}
	
	// Left arm
	if(name == "left_shoulder_pitch")
		abstractPose->leftArm.angleY = rate;
	else if(name == "left_shoulder_roll")
		abstractPose->leftArm.angleX = rate;
	else if(name == "left_arm_extension")
		abstractPose->leftArm.extension = rate;
	
	// Right arm
	if(name == "right_shoulder_pitch")
		abstractPose->rightArm.angleY = rate;
	else if(name == "right_shoulder_roll")
		abstractPose->rightArm.angleX = rate;
	else if(name == "right_arm_extension")
		abstractPose->rightArm.extension = rate;
	
	// Left left leg
	if(name == "left_hip_yaw")
		abstractPose->leftLeg.angleZ = rate;
	else if(name == "left_hip_roll")
		abstractPose->leftLeg.angleX = rate;
	else if(name == "left_hip_pitch")
		abstractPose->leftLeg.angleY = rate;
	else if(name == "left_leg_extension")
		abstractPose->leftLeg.extension = rate;
	else if(name == "left_ankle_pitch")
		abstractPose->leftLeg.footAngleY = rate;
	else if(name == "left_ankle_roll")
		abstractPose->leftLeg.footAngleX = rate;

	// Right leg
	if(name == "right_hip_yaw")
		abstractPose->rightLeg.angleZ = rate;
	else if(name == "right_hip_roll")
		abstractPose->rightLeg.angleX = rate;
	else if(name == "right_hip_pitch")
		abstractPose->rightLeg.angleY = rate;
	else if(name == "right_leg_extension")
		abstractPose->rightLeg.extension = rate;
	else if(name == "right_ankle_pitch")
		abstractPose->rightLeg.footAngleY = rate;
	else if(name == "right_ankle_roll")
		abstractPose->rightLeg.footAngleX = rate;
}

void AbstractSpace::handleRateChanged(std::string jointName)
{
	if(!m_current_frame)
		return;
	
	for(unsigned i = 0; i < jointViews.size(); i++)
	{
		RateAngleView *view = jointViews.at(i);
		
		if(view->getJointName() == jointName)
		{
			updateRateFromUI(jointName, view->getRate());
			break;
		}
	}
	
	p.updateFrame(*abstractPose, m_current_frame, m_joint_list);
	updateRobot();
}

void AbstractSpace::handleChangeForInverse(const std::string joint_name, const RateAngleView::Field field, const double value)
{
	for (unsigned i = 0; i < jointViews.size(); i++)
		if(jointViews.at(i)->getJointName() == joint_name)
		{
			jointViews.at(i)->setField(field, value);
			updateRateFromUI(joint_name, jointViews.at(i)->getRate());
			return;
		}
		
	p.updateFrame(*abstractPose, m_current_frame, m_joint_list);
	updateRobot();
}

void AbstractSpace::findAndPutView(std::string jointName, std::string label, int row
			, RateAngleView::Alignment alignment, RateAngleView::Type type, bool shiftMirrored)
{
	//for (unsigned i = 0; i < m_joint_list.size(); i++)
	//{
		//if(m_joint_list.at(i) == jointName)
		//{
			RateAngleView *view = new RateAngleView(alignment, type, jointName, shiftMirrored, this);
			
			connect(view, SIGNAL(rateChanged(std::string)), this, SLOT(handleRateChanged(std::string)));
			
			connect(view, SIGNAL(changeForInverse(const std::string, const RateAngleView::Field, const double))
				, this, SLOT(handleChangeForInverse(const std::string, const RateAngleView::Field, const double)));
			
			if(alignment == RateAngleView::LEFT || alignment == RateAngleView::NO_PAIR)
				jointsLayout->addWidget(view, row, 0);
			else
				jointsLayout->addWidget(view, row, 2);
			
			QLabel *nameLabel = new QLabel();
			nameLabel->setAlignment(Qt::AlignCenter);
			nameLabel->setText(QString::fromStdString(label));

			jointsLayout->addWidget(nameLabel, row, 1);	
			jointViews.push_back(view);
			//return;
		//}
	//}
	
	// Warning
	//jointsLayout->addWidget(new QLabel("Error on " + QString::fromStdString(jointName)), row, 1);	
}

void AbstractSpace::initViews()
{
	// Init labels for joints
	QLabel *jointsLabel = new QLabel("Joint");
	jointsLabel->setAlignment(Qt::AlignCenter);
	
	QFont font = jointsLabel->font();
	font.setBold(true);
	jointsLabel->setFont(font);
	
	jointsLayout->addWidget(jointsLabel, 0, 1);
	
	createHeaderLabels(RateAngleView::LEFT, 0);
	createHeaderLabels(RateAngleView::RIGHT, 0);
	
	// Init joints
	int row = 1;
	
	BasicSmallView::Alignment left   = BasicSmallView::LEFT;
	BasicSmallView::Alignment right  = BasicSmallView::RIGHT;
	
	BasicSmallView::Type regular   = BasicSmallView::REGULAR;
	BasicSmallView::Type extension = BasicSmallView::EXTENSION;
	BasicSmallView::Type leg       = BasicSmallView::LEG;
	
	findAndPutView("head_pitch", "Head Angle Y", row++, BasicSmallView::NO_PAIR, regular, false);
	findAndPutView("neck_yaw", "Head Angle Z", row++, BasicSmallView::NO_PAIR, regular, false);
	
	for(int i = 0; i < 3; i++) // gap between blocks
		jointsLayout->addWidget(createLine(), row, i); 
	row++;
	
	findAndPutView("left_shoulder_pitch", "Arm Angle Y",  row, left, regular, false);
	findAndPutView("right_shoulder_pitch", "Arm Angle Y", row++, right, regular, false);
	findAndPutView("left_shoulder_roll", "Arm Angle X",   row, left, regular, true);
	findAndPutView("right_shoulder_roll", "Arm Angle X",  row++, right, regular, true);
	
	findAndPutView("left_arm_extension", "Arm Extension", row, left, extension, false);
	findAndPutView("right_arm_extension", "Arm Extension",row++, right, extension, false);
	
	for(int i = 0; i < 3; i++) // gap between blocks
		jointsLayout->addWidget(createLine(), row, i); 
	row++;
	
	findAndPutView("left_hip_yaw", "Leg Angle Z",    row, left, leg, true);
	findAndPutView("right_hip_yaw", "Leg Angle Z",   row++, right, leg, true);
	findAndPutView("left_hip_roll", "Leg Angle X",   row, left, leg, true);
	findAndPutView("right_hip_roll", "Leg Angle X",  row++, right, leg, true);
	findAndPutView("left_hip_pitch", "Leg Angle Y",  row, left, leg, false);
	findAndPutView("right_hip_pitch", "Leg Angle Y", row++, right, leg, false);
	
	for(int i = 0; i < 3; i++) // gap between blocks
		jointsLayout->addWidget(createLine(), row, i); 
	row++;
	
	findAndPutView("left_leg_extension", "Leg Extension", row, left, extension, false);
	findAndPutView("right_leg_extension", "Leg Extension",row++, right, extension, false);
	
	findAndPutView("left_ankle_pitch", "Foot Angle Y",  row, left, leg, false);
	findAndPutView("right_ankle_pitch", "Foot Angle Y", row++, right, leg, false);
	findAndPutView("left_ankle_roll", "Foot Angle X",   row, left, leg, true);
	findAndPutView("right_ankle_roll", "Foot Angle X",  row++, right, leg, true);
	
	jointsLayout->setMargin(0);
	jointsLayout->setSpacing(0);
	jointsLayout->setContentsMargins(0,0,0,0);
}

void AbstractSpace::createHeaderLabels(RateAngleView::Alignment alignment, int row)
{
	QWidget *header = new QWidget(this);
	QGridLayout *layout = new QGridLayout(header);
	layout->setSpacing(10);
	
	QLabel *angleLabel = new QLabel("Angle");
	QLabel *rateLabel = new QLabel("Rate");
	
	QFont font = angleLabel->font();
	font.setBold(true);
	
	angleLabel->setFont(font);
	rateLabel->setFont(font);
	
	if(alignment == RateAngleView::LEFT)
	{
		rateLabel->setAlignment(Qt::AlignLeft);
		angleLabel->setAlignment(Qt::AlignLeft);
		
		rateLabel->setContentsMargins(0, 0, 22, 0);
		
		layout->addWidget(rateLabel, 0, 1);
		layout->addWidget(angleLabel, 0, 2);
		layout->setAlignment(Qt::AlignLeft);
		
		header->setLayout(layout);
		jointsLayout->addWidget(header, row, 0);
	}
	else
	{
		rateLabel->setAlignment(Qt::AlignRight);
		angleLabel->setAlignment(Qt::AlignRight);
		
		rateLabel->setContentsMargins(22, 0, 0, 0);
		
		layout->addWidget(rateLabel, 0, 1);
		layout->addWidget(angleLabel, 0, 0);
		layout->setAlignment(Qt::AlignRight);
		
		header->setLayout(layout);
		jointsLayout->addWidget(header, row, 2);
	}
}