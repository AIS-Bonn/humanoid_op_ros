//Displays joints of loaded frame
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include <trajectory_editor_2/jointdisplay.h>

#include <QEvent>
#include <math.h>

#include <ros/package.h>
#include <ros/console.h>

std::vector<std::string> stripList(std::vector<std::string> list)
{
	std::vector<std::string> strippedList;
	for (unsigned i = 0; i < list.size(); i++)
	{
		if (not list[i].empty())
			strippedList.push_back(list[i]);
	}
	return strippedList;
}

const double angToTic = 1000.0 / M_PI;
const double ticToAng = M_PI / 1000.0;

JointManager::JointManager(const std::vector< std::string > &modelJointList, QWidget *parent)
 : QWidget(parent)
 , m_currentFrame()
{
	jointsLayout = new QGridLayout();
	setLayout(jointsLayout);

	m_jointList = modelJointList;
	initFrames(stripList(modelJointList));
}

void JointManager::updateJointList(const std::vector< std::string > &jointList)
{
	m_jointList = jointList;
}

void JointManager::setFrame(JointManager::KeyframePtr frame)
{
	m_currentFrame = frame;
	updateFrame();
}

void JointManager::unsetFrame()
{
	m_currentFrame = KeyframePtr();
}

void JointManager::initFrames(std::vector<std::string> jointList)
{
	// Clear layout
	for (unsigned i = 0; i < jointViews.size(); i++)
		delete jointViews[i];
	
	jointViews.clear();

	// Init labels for joints
	QLabel *jointsLabel = new QLabel("Joint");
	jointsLabel->setAlignment(Qt::AlignCenter);
	
	QFont font = jointsLabel->font();
	font.setBold(true);
	jointsLabel->setFont(font);
	
	jointsLayout->addWidget(jointsLabel, 0, 1);
	
	this->createHeaderLabels(PosVelEffView::LEFT, 0);
	this->createHeaderLabels(PosVelEffView::RIGHT, 0);
	
	// Init joints
	int row = 1;
	int id = 0;
	
	findAndPutView(jointList, "head_pitch", "Head Pitch", row++, PosVelEffView::NO_PAIR, id++, false);
	findAndPutView(jointList, "neck_yaw", "Head Yaw", row++, PosVelEffView::NO_PAIR, id++, false);
	
	for(int i = 0; i < 3; i++) // gap between blocks
		jointsLayout->addWidget(createLine(), row, i); 
	row++;
	
	findAndPutView(jointList, "left_shoulder_pitch", "Shoulder Pitch",  row, PosVelEffView::LEFT, id++, false);
	findAndPutView(jointList, "right_shoulder_pitch", "Shoulder Pitch", row++, PosVelEffView::RIGHT, id++, false);
	findAndPutView(jointList, "left_shoulder_roll", "Shoulder Roll",   row, PosVelEffView::LEFT, id++, true);
	findAndPutView(jointList, "right_shoulder_roll", "Shoulder Roll",  row++, PosVelEffView::RIGHT, id++, true);
	
	findAndPutView(jointList, "left_elbow_pitch", "Elbow Pitch", row, PosVelEffView::LEFT, id++, false);
	findAndPutView(jointList, "right_elbow_pitch", "Elbow Pitch",row++, PosVelEffView::RIGHT, id++, false);
	
	for(int i = 0; i < 3; i++) // gap between blocks
		jointsLayout->addWidget(createLine(), row, i); 
	row++;
	
	findAndPutView(jointList, "left_hip_yaw", "Hip Yaw",    row, PosVelEffView::LEFT, id++, true);
	findAndPutView(jointList, "right_hip_yaw", "Hip Yaw",   row++, PosVelEffView::RIGHT, id++, true);
	findAndPutView(jointList, "left_hip_roll", "Hip Roll",   row, PosVelEffView::LEFT, id++, true);
	findAndPutView(jointList, "right_hip_roll", "Hip Roll",  row++, PosVelEffView::RIGHT, id++, true);
	findAndPutView(jointList, "left_hip_pitch", "Hip Pitch",  row, PosVelEffView::LEFT, id++, false);
	findAndPutView(jointList, "right_hip_pitch", "Hip Pitch", row++, PosVelEffView::RIGHT, id++, false);
	
	for(int i = 0; i < 3; i++) // gap between blocks
		jointsLayout->addWidget(createLine(), row, i); 
	row++;
	
	findAndPutView(jointList, "left_knee_pitch", "Knee Pitch", row, PosVelEffView::LEFT, id++, false);
	findAndPutView(jointList, "right_knee_pitch", "Knee Pitch",row++, PosVelEffView::RIGHT, id++, false);
	
	findAndPutView(jointList, "left_ankle_pitch", "Ankle Pitch",  row, PosVelEffView::LEFT, id++, false);
	findAndPutView(jointList, "right_ankle_pitch", "Ankle Pitch", row++, PosVelEffView::RIGHT, id++, false);
	findAndPutView(jointList, "left_ankle_roll", "Ankle Roll",   row, PosVelEffView::LEFT, id++, true);
	findAndPutView(jointList, "right_ankle_roll", "Ankle Roll",  row++, PosVelEffView::RIGHT, id++, true);
	
	jointsLayout->setMargin(0);
	jointsLayout->setSpacing(0);
	jointsLayout->setContentsMargins(0,0,0,0);
}

void JointManager::findAndPutView(std::vector<std::string>  &jointList, std::string jointName, std::string label
									, int row, PosVelEffView::Alignment alignment, int id, bool shiftMirrored)
{
	for (unsigned i = 0; i < jointList.size(); i++)
	{
		if(jointList.at(i) == jointName)
		{
			PosVelEffView *view = new PosVelEffView(alignment, jointList.at(i), id, this);
			view->setOnShiftMirrored(shiftMirrored);
				
			connect(view, SIGNAL(positionChanged()), this, SLOT(handlePositionChanged()));
			connect(view, SIGNAL(velocityChanged()), this, SLOT(handleVelocityChanged()));
			connect(view, SIGNAL(effortChanged()), this, SLOT(handleEffortChanged()));
			
			connect(view, SIGNAL(changeForID(int,PosVelEffView::Field,float))
					, this, SLOT(handleChangeForID(int,PosVelEffView::Field,float)));
			
			if(alignment == PosVelEffView::LEFT || alignment == PosVelEffView::NO_PAIR)
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
	for (unsigned i = 0; i < jointViews.size(); i++)
	{
		PosVelEffView *view = jointViews.at(i);

		std::string name = view->jointName;
		int index = indexToName(name);
		if (index < 0)
			continue;

		double currentPos = m_currentFrame->joints[index].position;
		double currentEff = m_currentFrame->joints[index].effort;
		double currentVel = m_currentFrame->joints[index].velocity;
		
		view->clearHistoryOfChanges();
		
		view->setPosition(currentPos);
		view->setEffort(currentEff);
		view->setVelocity(currentVel);
	}
	
	frameDataChanged(); // Signal that robot view should be updated
}

void JointManager::handleChangeForID(int id, PosVelEffView::Field field, float value)
{
	for (unsigned i = 0; i < jointViews.size(); i++)
	{
		PosVelEffView *view = jointViews.at(i);
		
		if(view->getID() == id)
		{
			view->setField(field, value);
			return;
		}
	}
}

void JointManager::handleEffortChanged()
{
	if (!m_currentFrame)
		return;

	for (unsigned i = 0; i < jointViews.size(); i++)
	{
		PosVelEffView *view = jointViews.at(i);
		
		std::string name = view->jointName;
		int index = indexToName(name);
		if (index < 0)
			continue;
		m_currentFrame->joints[index].effort = view->effortSpin->value();
	}
}

void JointManager::handleVelocityChanged()
{
	if (!m_currentFrame)
		return;

	for (unsigned i = 0; i < jointViews.size(); i++)
	{
		PosVelEffView *view = jointViews.at(i);
		
		std::string name = view->jointName;
		int index = indexToName(name);
		if (index < 0)
			continue;
		m_currentFrame->joints[index].velocity = view->velocitySpin->value();
	}
}

void JointManager::handlePositionChanged()
{
	if (!m_currentFrame)
		return;

	for (unsigned i = 0; i < jointViews.size(); i++)
	{
		PosVelEffView *view = jointViews.at(i);

		std::string name = view->jointName;
		int index = indexToName(name);
		if (index < 0)
			continue;

		m_currentFrame->joints[index].position = view->getPosition();
	}
	
	frameDataChanged(); // Signal that robot view should be updated
}

int JointManager::indexToName(std::string name)
{
	for (unsigned i = 0; i < m_jointList.size(); i++)
	{
		if (m_jointList[i] == name)
			return i;
	}
	return -1;
}

QFrame* JointManager::createLine()
{
	QFrame *line = new QFrame();
	line->setFrameShape(QFrame::HLine);
	line->setFrameShadow(QFrame::Sunken);
	line->setMidLineWidth(2);
	
	return line;
}

void JointManager::createHeaderLabels(PosVelEffView::Alignment alignment, int row)
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
	
	if(alignment == PosVelEffView::LEFT)
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
