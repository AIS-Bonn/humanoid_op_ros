#include <trajectory_editor_2/abstractspace.h>

#include <trajectory_editor_2/poseconverter.h>

#include <ros/package.h>
#include <ros/console.h>

AbstractSpace::AbstractSpace(const std::vector<std::string> &jointList, QWidget *parent) : QWidget(parent)
{
	jointsLayout = new QGridLayout();
	setLayout(jointsLayout);
	
	abstractPose = new AbstractPose();

	this->joints = jointList;
	initViews();
}

AbstractSpace::~AbstractSpace()
{
	delete abstractPose;
}

void AbstractSpace::setFrame(KeyframePtr frame)
{
	this->frame = frame;
	updateFrame();
}

void AbstractSpace::updateJointList(const std::vector<std::string>& jointList)
{
	this->joints = jointList;
}

void AbstractSpace::updateFrame()
{
	if(!frame)
		return;
	
	// Update head
	
	int headID = indexToName("head_pitch");
	int neckID = indexToName("neck_yaw");
	
	updateRate(frame->joints[headID].position, "head_pitch");
	updateRate(frame->joints[neckID].position, "neck_yaw");
	
	abstractPose->setFromJointPose(PoseConverter::getJointPose(frame, joints));
	
	// Update left arm
	updateRate(abstractPose->leftArm.angleY, "left_shoulder_pitch");
	updateRate(abstractPose->leftArm.angleX, "left_shoulder_roll");
	updateRate(abstractPose->leftArm.extension, "left_elbow_pitch");
	
	// Update right arm
	updateRate(abstractPose->rightArm.angleY, "right_shoulder_pitch");
	updateRate(abstractPose->rightArm.angleX, "right_shoulder_roll");
	updateRate(abstractPose->rightArm.extension, "right_elbow_pitch");
	
	// Update left leg
	updateRate(abstractPose->leftLeg.angleZ, "left_hip_yaw");
	updateRate(abstractPose->leftLeg.angleX, "left_hip_roll");
	updateRate(abstractPose->leftLeg.angleY, "left_hip_pitch");
	updateRate(abstractPose->leftLeg.extension, "left_knee_pitch");
	updateRate(abstractPose->leftLeg.footAngleY, "left_ankle_pitch");
	updateRate(abstractPose->leftLeg.footAngleX, "left_ankle_roll");
	
	// Update right leg
	updateRate(abstractPose->rightLeg.angleZ, "right_hip_yaw");
	updateRate(abstractPose->rightLeg.angleX, "right_hip_roll");
	updateRate(abstractPose->rightLeg.angleY, "right_hip_pitch");
	updateRate(abstractPose->rightLeg.extension, "right_knee_pitch");
	updateRate(abstractPose->rightLeg.footAngleY, "right_ankle_pitch");
	updateRate(abstractPose->rightLeg.footAngleX, "right_ankle_roll");
}

void AbstractSpace::updateRate(double rate, std::string jointName)
{
	for(unsigned i = 0; i < jointViews.size(); i++)
	{
		RateAngleView *view = jointViews.at(i);
		
		if(view->jointName == jointName) // Update position
		{
			view->clearHistoryOfChanges();
			view->setRate(rate);
			break;
		}
	}
}

void AbstractSpace::updateRateFromUI(string name, double rate)
{
	if(name == "head_pitch" || name == "neck_yaw")
	{
		int id = indexToName(name);
		frame->joints[id].position = rate;
	}
	
	// Left arm
	if(name == "left_shoulder_pitch")
		abstractPose->leftArm.angleY = rate;
	else if(name == "left_shoulder_roll")
		abstractPose->leftArm.angleX = rate;
	else if(name == "left_elbow_pitch")
		abstractPose->leftArm.extension = rate;
	
	// Right arm
	if(name == "right_shoulder_pitch")
		abstractPose->rightArm.angleY = rate;
	else if(name == "right_shoulder_roll")
		abstractPose->rightArm.angleX = rate;
	else if(name == "right_elbow_pitch")
		abstractPose->rightArm.extension = rate;
	
	// Left left leg
	if(name == "left_hip_yaw")
		abstractPose->leftLeg.angleZ = rate;
	else if(name == "left_hip_roll")
		abstractPose->leftLeg.angleX = rate;
	else if(name == "left_hip_pitch")
		abstractPose->leftLeg.angleY = rate;
	else if(name == "left_knee_pitch")
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
	else if(name == "right_knee_pitch")
		abstractPose->rightLeg.extension = rate;
	else if(name == "right_ankle_pitch")
		abstractPose->rightLeg.footAngleY = rate;
	else if(name == "right_ankle_roll")
		abstractPose->rightLeg.footAngleX = rate;
}
	
int AbstractSpace::indexToName(std::string name)
{
	for (unsigned i = 0; i < joints.size(); i++)
	{
		if (joints[i] == name)
			return i;
	}
	return -1;
}

void AbstractSpace::handleRateChanged()
{
	if(!frame)
		return;
	
	for(unsigned i = 0; i < jointViews.size(); i++)
	{
		RateAngleView *view = jointViews.at(i);
		updateRateFromUI(view->jointName, view->getRate());
	}
	
	PoseConverter::updateFrame(*abstractPose, frame, joints);
	frameDataChanged();
}

void AbstractSpace::findAndPutView(std::string jointName, std::string label, int row
			, RateAngleView::Alignment alignment, RateAngleView::Type type)
{
	for (unsigned i = 0; i < joints.size(); i++)
	{
		if(joints.at(i) == jointName)
		{
			RateAngleView *view = new RateAngleView(alignment, type, joints.at(i), this);
			connect(view, SIGNAL(rateChanged()), this, SLOT(handleRateChanged()));
			
			if(alignment == RateAngleView::LEFT)
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
	
	findAndPutView("head_pitch", "Head Angle Y", row++, RateAngleView::LEFT, RateAngleView::REGULAR);
	findAndPutView("neck_yaw", "Head Angle Z", row++, RateAngleView::LEFT, RateAngleView::REGULAR);
	
	for(int i = 0; i < 3; i++) // gap between blocks
		jointsLayout->addWidget(createLine(), row, i); 
	row++;
	
	findAndPutView("left_shoulder_pitch", "Arm Angle Y",  row, RateAngleView::LEFT, RateAngleView::REGULAR);
	findAndPutView("right_shoulder_pitch", "Arm Angle Y", row++, RateAngleView::RIGHT, RateAngleView::REGULAR);
	findAndPutView("left_shoulder_roll", "Arm Angle X",   row, RateAngleView::LEFT, RateAngleView::REGULAR);
	findAndPutView("right_shoulder_roll", "Arm Angle X",  row++, RateAngleView::RIGHT, RateAngleView::REGULAR);
	
	findAndPutView("left_elbow_pitch", "Arm Extension", row, RateAngleView::LEFT, RateAngleView::EXTENSION);
	findAndPutView("right_elbow_pitch", "Arm Extension",row++, RateAngleView::RIGHT, RateAngleView::EXTENSION);
	
	for(int i = 0; i < 3; i++) // gap between blocks
		jointsLayout->addWidget(createLine(), row, i); 
	row++;
	
	findAndPutView("left_hip_yaw", "Leg Angle Z",    row, RateAngleView::LEFT, RateAngleView::LEG);
	findAndPutView("right_hip_yaw", "Leg Angle Z",   row++, RateAngleView::RIGHT, RateAngleView::LEG);
	findAndPutView("left_hip_roll", "Leg Angle X",   row, RateAngleView::LEFT, RateAngleView::LEG);
	findAndPutView("right_hip_roll", "Leg Angle X",  row++, RateAngleView::RIGHT, RateAngleView::LEG);
	findAndPutView("left_hip_pitch", "Leg Angle Y",  row, RateAngleView::LEFT, RateAngleView::LEG);
	findAndPutView("right_hip_pitch", "Leg Angle Y", row++, RateAngleView::RIGHT, RateAngleView::LEG);
	
	for(int i = 0; i < 3; i++) // gap between blocks
		jointsLayout->addWidget(createLine(), row, i); 
	row++;
	
	findAndPutView("left_knee_pitch", "Leg Extension", row, RateAngleView::LEFT, RateAngleView::EXTENSION);
	findAndPutView("right_knee_pitch", "Leg Extension",row++, RateAngleView::RIGHT, RateAngleView::EXTENSION);
	
	findAndPutView("left_ankle_pitch", "Foot Angle Y",  row, RateAngleView::LEFT, RateAngleView::LEG);
	findAndPutView("right_ankle_pitch", "Foot Angle Y", row++, RateAngleView::RIGHT, RateAngleView::LEG);
	findAndPutView("left_ankle_roll", "Foot Angle X",   row, RateAngleView::LEFT, RateAngleView::LEG);
	findAndPutView("right_ankle_roll", "Foot Angle X",  row++, RateAngleView::RIGHT, RateAngleView::LEG);
	
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

QFrame* AbstractSpace::createLine()
{
	QFrame *line = new QFrame();
	line->setFrameShape(QFrame::HLine);
	line->setFrameShadow(QFrame::Sunken);
	line->setMidLineWidth(2);
	
	return line;
}