//Displays trajectories
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QStringList>
#include <QMimeData>

#include <algorithm>
#include <stdio.h>

#include <ros/package.h>
#include <ros/console.h>

#include <trajectory_editor_2/keyframemodel.h>
#include <trajectory_editor_2/mainwindow.h>
#include <trajectory_editor_2/headerview.h>
#include <trajectory_editor_2/frameview.h>

#include <motion_player/StreamMotion.h>
#include <motion_player/PlayMotion.h>

// Types for motion player. Do not change
#define TYPE_MOTION 1
#define TYPE_FRAME 2

// Consts for drag and drop methods
const char* FRAME_MIME_TYPE = "application/vnd.nimbro.frame";
const int   DEFAULT_INSERT  = -1;

KeyframeModel::KeyframeModel(QObject* parent)
 : QAbstractListModel(parent)
 , m_fileActive(false)
{
	ros::NodeHandle nh("~");

    m_parent = (MainWindow*)parent;

	m_cl_update = nh.serviceClient<motion_player::StreamMotion>("/motion_player/update");
	m_cl_play = nh.serviceClient<motion_player::PlayMotion>("/motion_player/play");
}

void KeyframeModel::initRobot(RobotDisplay* robot)
{
	m_robot = robot;
	m_rbdl = robot->getRBDL();

	m_motion.jointList.resize(m_rbdl->mJoints.size());
	for (unsigned i = 0; i < m_rbdl->mJoints.size(); i++)
	{
		if (m_rbdl->jointName(i).empty())
			m_motion.jointList[i] = "";
		else
			m_motion.jointList[i] = m_rbdl->jointName(i);
	}
}

void KeyframeModel::initJointDisplay(QTabWidget *jointsTabWidget, QTabWidget *headerTabWidget)
{
	// Create Header tabs
	headerView = new HeaderView(headerTabWidget);
	headerView->enableEdit(false);
    headerTabWidget->addTab(createAreaWithWidget(headerView), "Header");
	
	frameView = new FrameView(m_motion.jointList, headerTabWidget);
    headerTabWidget->addTab(createAreaWithWidget(frameView), "Frame");
	
    // Create Joints tabs
	m_jointManager = new JointManager(m_motion.jointList, jointsTabWidget);
    jointsTabWidget->addTab(createAreaWithWidget(m_jointManager), "Joint Space");
	
	abstractSpace = new AbstractSpace(m_motion.jointList, jointsTabWidget);
	jointsTabWidget->addTab(createAreaWithWidget(abstractSpace), "Abstract Space");
	
	inverseSpace = new InverseSpace(m_motion.jointList, jointsTabWidget);
	jointsTabWidget->addTab(createAreaWithWidget(inverseSpace), "Inverse Space");	
	
	// Resize tab widgets
	m_jointManager->resize(800, 150);
	headerView->resize(800, 250);
	
	jointsTabWidget->updateGeometry();
	headerTabWidget->updateGeometry();
	
	// Set up connections
	
	// Spaces
	connect(jointsTabWidget, SIGNAL(currentChanged(int)), this, SLOT(handleTabChange(int)));
	
    connect(m_jointManager, SIGNAL(frameDataChanged()), this, SLOT(updateRobotDisplay()));
	connect(abstractSpace, SIGNAL(frameDataChanged()), this, SLOT(updateRobotDisplay()));
	connect(inverseSpace, SIGNAL(frameDataChanged()), this, SLOT(updateRobotDisplay()));
	
	// Header and Frame views
	connect(headerView, SIGNAL(requestUpdate()), this, SLOT(updateHeaderData()));
	
	connect(frameView, SIGNAL(updateFrame()), this, SLOT(handleUpdateFrame()));
	connect(frameView, SIGNAL(frameLoaded(KeyframePtr)), this, SLOT(handeFrameLoaded(KeyframePtr)));
	
	connect(frameView, SIGNAL(changeEffortForAllFrames(std::vector<int>))
	, this, SLOT(handleChangeEffortForAllFrames(std::vector<int>)));
}

QScrollArea* KeyframeModel::createAreaWithWidget(QWidget *widget)
{
	QScrollArea *area = new QScrollArea;
    area->setWidget(widget);
    area->setWidgetResizable(true);
	return area;
}

QVariant KeyframeModel::data(const QModelIndex& index, int role) const
{
	KeyframePtr temp = m_motion.frames[index.row()];
	
	if(role == Qt::EditRole)
		return QVariant(temp->id);
	else if(role == Qt::DisplayRole)
		return QVariant(QString::fromStdString(temp->name));
	else
		return QVariant();
}

int KeyframeModel::rowCount(const QModelIndex& parent) const
{
	return m_motion.frames.size();
}

Qt::DropActions KeyframeModel::supportedDropActions() const
{
    return Qt::CopyAction | Qt::MoveAction;
}

Qt::ItemFlags KeyframeModel::flags(const QModelIndex& index) const
{
    if (index.isValid())
		return Qt::ItemIsSelectable |
			Qt::ItemIsDragEnabled |
			Qt::ItemIsDropEnabled |
			Qt::ItemIsEnabled;
	else
		return Qt::ItemIsDropEnabled | QAbstractListModel::flags(index);
}

QStringList KeyframeModel::mimeTypes() const
{
	QStringList types;
	types << FRAME_MIME_TYPE;
	return types;
}

QMimeData* KeyframeModel::mimeData(const QModelIndexList& indexes) const
{
	QMimeData* mimeData = new QMimeData();
	QByteArray enc;
	QDataStream stream(&enc, QIODevice::WriteOnly);
	
	
	Q_FOREACH (QModelIndex i, indexes)
	{
		if(i.isValid())
		{
			int id = data(i, Qt::EditRole).toInt();
			stream << id;
		}
	}
	
	mimeData->setData(FRAME_MIME_TYPE, enc);
	return mimeData;
}

// TODO implement in a nicer way
bool KeyframeModel::dropMimeData(const QMimeData *data, Qt::DropAction action, int row, int column, const QModelIndex &parent)
{
	if (action == Qt::IgnoreAction)
		return true;
	
	if (!data->hasFormat(FRAME_MIME_TYPE))
		return false;
	
	if (column > 0)
		return false;
	
	int positionToDrop;
	
	if (row != -1)
		positionToDrop = row;
	else if (parent.isValid())
		positionToDrop = parent.row();
	else
		positionToDrop = rowCount(QModelIndex());
	
	QByteArray encodedData = data->data(FRAME_MIME_TYPE);
	QDataStream stream(&encodedData, QIODevice::ReadOnly);
	std::vector<int> ids;
	
	QItemSelection selection;
	
	while (!stream.atEnd()) // read ids which need to be dropped
	{
		int id = 0;
		stream >> id;
		ids.push_back(id);
	}
	
	for(size_t j = 0; j < ids.size(); j++)
	{
		int position = this->idToPosition(ids.at(j));
		KeyframePtr frameToDrop = m_motion.frames[position];
		
		this->removeRows(position, 1, parent); // remove, then insert
		this->insertRow(positionToDrop, frameToDrop, parent);
	}
	
	// Create selection for dropped items
	if(this->idToPosition(ids.at(0)) < positionToDrop)
		positionToDrop = positionToDrop - ids.size() + 1;
	
	for(size_t j = 0; j < ids.size(); j++) // select
	{
		QModelIndex index = this->index(positionToDrop+j, 0, QModelIndex());
		selection.select(index, index);
	}
	
	setViewSelection(selection);
	return true;
}

bool KeyframeModel::removeRows(int row, int count, const QModelIndex &parent)
{
	beginRemoveRows(parent, row, row + count);
	m_motion.frames.erase(m_motion.frames.begin() + row, m_motion.frames.begin() + row + 1);
	endRemoveRows();
	return true;
}

void KeyframeModel::insertRow(int row, KeyframePtr frame, const QModelIndex &parent)
{
	beginInsertRows(parent, row, row);
	m_motion.frames.insert(m_motion.frames.begin() + row, 1, frame);
	endInsertRows();
}

void KeyframeModel::load()
{
	// Get path of file to load
	QString dir = QString::fromStdString(ros::package::getPath("launch") + "/motions/");
    QString path = QFileDialog::getOpenFileName(m_parent, tr("open file"), dir);
	
	if(path.isEmpty())
		return;
	
	filePath = path;
	
	// Clear current motion and load new
	m_motion.jointList.clear();
	m_motion.frames.clear();

    m_motion.load(filePath.toStdString());
	reset();
	
	for(int i = 0; i < (int)m_motion.frames.size(); i++) // Set up id and name for frames
	{
		KeyframePtr temp = m_motion.frames[i];
		temp->id = i+1;
		
		std::stringstream s;
		s << temp->id;
		temp->name = std::string("Frame ").append(s.str());
	}
	// Set motion name
	m_motion.motionName = headerView->setFileNameFromPath(filePath);
	
	// Update views
	m_jointManager->updateJointList(m_motion.jointList);
	abstractSpace->updateJointList(m_motion.jointList);
	inverseSpace->updateJointList(m_motion.jointList);

	frameView->updateJointList(m_motion.jointList);
	frameView->setPathAndName(path);
	
	headerView->enableEdit(true);
	headerView->setValues(m_motion.motionName, m_motion.preState, m_motion.playState, m_motion.postState);
	headerView->setFileNameFromPath(filePath);
	
	QStringList pieces = filePath.split( "/" ); // Extract file name + extension for header
	m_parent->updateHeaderLabel(pieces.last());
    m_parent->updateStatusBar(QString("On file:  " + filePath));
}

void KeyframeModel::save()
{
	this->updateHeaderData();
	
	if (m_motion.jointList.empty()) // if joint list is empty - dont save, show error
	{
		QMessageBox messageBox;
		messageBox.setWindowTitle("Fatal error!");
		messageBox.setText("Joint list is empty!\nPlease, report the bug: dm.mark999@gmail.com");
		messageBox.exec();
		return;
	}

	if(headerView->requestSave()) // Save only if everything in header is ok
		m_motion.save(filePath.toStdString());
}

void KeyframeModel::saveAs()
{
	this->updateHeaderData();
	
	if (m_motion.jointList.empty()) // If joint list is empty - dont save, show error
	{
		QMessageBox messageBox;
		messageBox.setWindowTitle("Fatal error!");
		messageBox.setText("Joint list is empty!\nPlease, report the bug to dm.mark999@gmail.com");
		messageBox.exec();
		return;
	}
	
	// Get filename to 'save as'
	QString dir = QString::fromStdString(ros::package::getPath("launch") + "/motions/savedMotion.yaml");
	QString path = QFileDialog::getSaveFileName(m_parent, tr("Save motion as. Please, type extension")
														, dir, tr("Motion files(*.yaml)"));
	
	if(headerView->requestSave()) // Save only if everything in header is ok
	{
		m_motion.motionName = headerView->setFileNameFromPath(path);
		m_motion.save(path.toStdString());
	}
}

void KeyframeModel::saveMirroredAs()
{
	this->updateHeaderData();
	
	if (m_motion.jointList.empty()) // If joint list is empty - dont save, show error
	{
		QMessageBox messageBox;
		messageBox.setWindowTitle("Fatal error!");
		messageBox.setText("Joint list is empty!\nPlease, report the bug to dm.mark999@gmail.com");
		messageBox.exec();
		return;
	}
	
	// Get filename to 'save as'
	QString dir = QString::fromStdString(ros::package::getPath("launch") + "/motions/" 
		+ headerView->getName() + "_mirrored.yaml");
	
	QString path = QFileDialog::getSaveFileName(m_parent, tr("Save motion as. Please, type extension")
														, dir, tr("Motion files(*.yaml)"));
	
	// Prepare mirrored motion
	motionfile::Motion mirroredMotion;
	mirroredMotion.motionName = m_motion.motionName;
	mirroredMotion.jointList = m_motion.jointList;
	mirroredMotion.playState = m_motion.playState;
	mirroredMotion.preState =  m_motion.preState;
	mirroredMotion.postState = m_motion.postState;
	
	// Prepare mirrored frames
	for(unsigned i = 0; i < m_motion.frames.size(); i++)
	{
		KeyframePtr mirroredFrame = createFrame(m_motion.frames[i]);
		
		mirrorValues(m_motion.frames[i], mirroredFrame, "neck_yaw", "neck_yaw", false, true);
        
        mirrorValues(m_motion.frames[i], mirroredFrame, "left_elbow_pitch", "right_elbow_pitch", true, false);
		mirrorValues(m_motion.frames[i], mirroredFrame, "left_shoulder_pitch", "right_shoulder_pitch", true, false);
		mirrorValues(m_motion.frames[i], mirroredFrame, "left_hip_pitch", "right_hip_pitch", true, false);
		mirrorValues(m_motion.frames[i], mirroredFrame, "left_ankle_pitch", "right_ankle_pitch", true, false);
		mirrorValues(m_motion.frames[i], mirroredFrame, "left_knee_pitch", "right_knee_pitch", true, false);
		
		mirrorValues(m_motion.frames[i], mirroredFrame, "left_hip_roll", "right_hip_roll", true, true);
		mirrorValues(m_motion.frames[i], mirroredFrame, "left_shoulder_roll", "right_shoulder_roll", true, true);
		mirrorValues(m_motion.frames[i], mirroredFrame, "left_ankle_roll", "right_ankle_roll", true, true);
		mirrorValues(m_motion.frames[i], mirroredFrame, "left_hip_yaw", "right_hip_yaw", true, true);
		
		// Mirror support coefs
		if(m_motion.frames[i]->support != "")
		{
			// Parse support coefs
			QString support = QString::fromStdString(m_motion.frames[i]->support);
			QStringList pieces = support.split( " " );
			
			if(pieces.size() !=2)
			{
				mirroredMotion.frames.push_back(mirroredFrame);
				continue;
			}
				
			// Mirror support coefs
			double left = pieces.at(0).toDouble();
			double right = pieces.at(1).toDouble();
			
			std::ostringstream ss;
			ss << 1 - left;
			ss << " ";
			ss << 1 - right;
			
			mirroredFrame->support = ss.str();;
		}
		
		mirroredMotion.frames.push_back(mirroredFrame);
	}
	
	if(headerView->requestSave()) // Save only if everything in header is ok
	{
		mirroredMotion.motionName = headerView->setFileNameFromPath(path);
		mirroredMotion.save(path.toStdString());
	}
}

void  KeyframeModel::mirrorValues(KeyframePtr frame, KeyframePtr mirrored, std::string 
		 left, std::string right, bool swap, bool invert)
{
	int leftID;
	int rightID;
	
	double leftPos;
	double rightPos;
	
	if(swap == false) // Just invert the specified joint's position
	{
		leftID = m_jointManager->indexToName(left);
		mirrored->joints[leftID].position = -frame->joints[leftID].position; 
		return;
	}
	
	// Swap the positions of left and right
	leftID = m_jointManager->indexToName(left);
	leftPos = frame->joints[leftID].position; 
	
	rightID = m_jointManager->indexToName(right);
	rightPos = frame->joints[rightID].position; 
	
	if(invert == false) // Just swap
	{
		mirrored->joints[leftID].position = rightPos;
		mirrored->joints[rightID].position = leftPos;
	}
	else // Swap and invert
	{
		mirrored->joints[leftID].position = -rightPos;
		mirrored->joints[rightID].position = -leftPos;
	}
}

void KeyframeModel::createMotion()
{
	if (m_fileActive)
		return;
	
	// Get filename and path of new motion
	QString dir = QString::fromStdString(ros::package::getPath("launch") + "/motions/newMotion.yaml");
	
    QString path = QFileDialog::getSaveFileName(m_parent, tr("Create new motion. Please, type extension")
												, dir, tr("Motion files(*.yaml)"));
	
	if(path.isEmpty())
		return;
	
	filePath = path;
	
	// Set up new motion
	m_motion.motionName = headerView->setFileNameFromPath(filePath);
	
	m_motion.frames.clear();
	m_motion.jointList.clear();
	m_motion.jointList.resize(m_rbdl->mJoints.size());

	for (unsigned i = 0; i < m_rbdl->mJoints.size(); i++)
	{
		if (m_rbdl->jointName(i).empty())
			m_motion.jointList[i] = "";
		else
			m_motion.jointList[i] = m_rbdl->jointName(i);
	}

	m_motion.frames.push_back(createFrame());
	reset();
	
	// Update views
	m_jointManager->updateJointList(m_motion.jointList);
	abstractSpace->updateJointList(m_motion.jointList);
	inverseSpace->updateJointList(m_motion.jointList);
	setCurrentFrame(m_motion.frames[0]);
	
	headerView->enableEdit(true);
	headerView->setValues(headerView->setFileNameFromPath(filePath), "", "", "");
	m_motion.motionName = headerView->getName();
	
	frameView->updateJointList(m_motion.jointList);
	frameView->setPathAndName(path);
	
	QStringList pieces = filePath.split("/"); // Extract file name + extension for header
	m_parent->updateHeaderLabel(pieces.last());
    m_parent->updateStatusBar(QString("On file:  " + filePath));
}

KeyframeModel::KeyframePtr KeyframeModel::createFrame()
{
	KeyframePtr newFrame(new motionfile::Keyframe);
	newFrame->duration = 0;
	newFrame->id = this->findNextID();
	
	std::stringstream s;
	s << newFrame->id;
	newFrame->name = std::string("Frame ").append(s.str());
	
	for (unsigned i = 0; i < m_motion.jointList.size(); i++)
	{
		motionfile::FrameJoint newJoint;
		newJoint.effort = 1;
		newJoint.position = 0;
		newFrame->joints.push_back(newJoint);
	}

	setCurrentFrame(newFrame);
	return newFrame;
}

KeyframeModel::KeyframePtr KeyframeModel::createFrame(KeyframeModel::KeyframePtr oldFrame)
{
	if(!oldFrame)
		return createFrame();

	KeyframePtr newFrame(new motionfile::Keyframe);
	newFrame->duration = oldFrame->duration;
	newFrame->support = oldFrame->support;
	newFrame->id = this->findNextID();
	
	std::stringstream s;
	s << newFrame->id;
	newFrame->name = std::string("Frame ").append(s.str());
	
	for (unsigned i = 0; i < oldFrame->joints.size(); i++)
	{
		motionfile::FrameJoint newJoint(oldFrame->joints[i]);
		newFrame->joints.push_back(newJoint);
	}

	setCurrentFrame(newFrame);
	return newFrame;
}

int KeyframeModel::findNextID()
{
	int nextID = -1;

	for(size_t i = 0; i < m_motion.frames.size(); i++)
	{
		KeyframePtr temp = m_motion.frames[i];
		if(temp->id > nextID)
			nextID = temp->id;
	}
	
	return nextID+1;
}

int KeyframeModel::idToPosition(int id)
{
	for(unsigned i = 0; i < m_motion.frames.size(); i++)
	{
		KeyframePtr temp = m_motion.frames[i];
		if(temp->id == id)
			return i;
	}
	
	return 0;
}

void KeyframeModel::setCurrentFrame(KeyframePtr frame)
{
	if(!frame)
		return;
	
	m_currentFrame = frame;
	
	m_jointManager->setFrame(frame);
	abstractSpace->setFrame(frame);
	inverseSpace->setFrame(frame);
	
	frameView->setFrame(frame);
}

void KeyframeModel::updateHeaderData()
{
	m_motion.motionName = headerView->getName();
	m_motion.preState = headerView->getPreState();
	m_motion.playState = headerView->getPlayState();
	m_motion.postState = headerView->getPostState();
}

void KeyframeModel::moveFrame(unsigned position, int direction)
{
	if(direction != 1 && direction != -1)
		return;
	
	int nextPos = position + direction;
	std::iter_swap(m_motion.frames.begin() + position, m_motion.frames.begin() + nextPos);
}

void KeyframeModel::addFrame()
{
	if (m_motion.motionName.empty())
		return;

	beginInsertRows(QModelIndex(), 0, m_motion.frames.size()-1);
	m_motion.frames.push_back(createFrame(m_currentFrame));
	endInsertRows();
	
	setViewSelection(m_motion.frames.size()-1);
	this->setCurrentFrame(m_motion.frames.back());
}

void KeyframeModel::removeFrame(unsigned position)
{
	if (position > m_motion.frames.size() || position < 0)
		return;

	if (m_motion.frames.size() == 1)
	{
		m_currentFrame = KeyframePtr();
		m_jointManager->unsetFrame();
		
		removeRows(position, 1, QModelIndex());
		return;
	}

	int nextSelectedFramePosition = 0;
	
	if(position == 0)
		nextSelectedFramePosition = 0;
	else
		nextSelectedFramePosition = position - 1;

	
	removeRows(position, 1, QModelIndex());
	
	setCurrentFrame(m_motion.frames[nextSelectedFramePosition]);
	setViewSelection(nextSelectedFramePosition);
}

void KeyframeModel::handleIndexChange(const QModelIndex& index)
{
	int frameIndex = index.row();
	m_currentFrame = m_motion.frames[frameIndex];
	
	if (m_currentFrame)
		setCurrentFrame(m_currentFrame);
}

void KeyframeModel::handleUpdateFrame()
{
	setCurrentFrame(m_currentFrame);
	dataChanged(QModelIndex(),QModelIndex());
}

void KeyframeModel::handeFrameLoaded(KeyframePtr frame)
{
	int pos = this->idToPosition(m_currentFrame->id);
	m_motion.frames[pos] = frame;
	
	setCurrentFrame(frame);
}

void KeyframeModel::handleChangeEffortForAllFrames(std::vector<int> indexes)
{
	double newEffort = frameView->getEffort();
	
	for(unsigned j = 0; j < m_motion.frames.size(); j++)
	{
		KeyframePtr tempFrame = m_motion.frames[j];
	
		for(unsigned i = 0; i < indexes.size(); i++)
		{
			int id = indexes.at(i);
			if(id < 0)
				continue;

			tempFrame->joints[id].effort = newEffort;
		}
	}
	
	setCurrentFrame(m_currentFrame);
}

void KeyframeModel::handleTabChange(int id)
{
	ROS_INFO("Tab switched: %d", id);
	
	if(!m_currentFrame)
		return;
	
	if(id == 0) // Joint space
	{
		m_jointManager->setFrame(m_currentFrame);
	}
	else if(id == 1) // Abstract space
	{
		abstractSpace->setFrame(m_currentFrame);
	}
	else if(id == 2) // Inverse space
	{
		inverseSpace->setFrame(m_currentFrame);
	}
}

void KeyframeModel::updateRobotDisplay()
{
	if (!m_currentFrame)
		return;

	std::vector<double> positions(m_rbdl->dof_count, 0);
	for (unsigned i = 0; i < m_motion.jointList.size(); i++)
	{
		if (m_motion.jointList[i].empty())
			continue;

		int ind = m_rbdl->jointIndex(m_motion.jointList[i]) - 1;
		positions[ind] = m_currentFrame->joints[i].position;
	}
	m_robot->update(positions);
}

void KeyframeModel::requestUpdate(motionfile::Motion &motion, int type)
{
	motion_player::StreamMotion update;
	std::string sendM = motion.dump();

	update.request.name = motion.motionName;
	update.request.motion = sendM;
	update.request.type = type;

	m_cl_update.call(update);
}

void KeyframeModel::requestPlay(motionfile::Motion &motion, int type)
{
	motion_player::PlayMotion play;
	play.request.name = motion.motionName;
	play.request.type = type;

	m_cl_play.call(play);
}

void KeyframeModel::playMotion()
{
	if(m_motion.frames.size() < 1)
		return;

	// Play frame[0] first (init robot beginning position)
	KeyframePtr frame = m_motion.frames[0];
	requestPlayFrame(frame, 1.5);

	int waitTime = (int)frame->duration*1.5 + 1;
	sleep(waitTime); // Wait until first frame will finish playing

	requestUpdate(m_motion, TYPE_MOTION); // update & play motion
	requestPlay(m_motion, TYPE_MOTION);
}

void KeyframeModel::playMotionSLow()
{
	if(m_motion.frames.size() < 1)
		return;
	
	// Play frame[0] first (init robot beginning position)
	KeyframePtr frame = m_motion.frames[0];
	requestPlayFrame(frame, 1.5);
	
	int waitTime = frame->duration*1.5 + 1;
	sleep(waitTime); // Wait until first frame will finish playing
	
	// Construct motion with scaled duration
	double factor = headerView->getFactor();
	
	motionfile::Motion tempMotion;
	tempMotion.motionName = m_motion.motionName;
	tempMotion.jointList = m_motion.jointList;
	tempMotion.playState = m_motion.playState;
	tempMotion.preState =  m_motion.preState;
	tempMotion.postState = m_motion.postState;
	
	KeyframePtr currentFrame = m_currentFrame;
	
	for(unsigned i = 0; i < m_motion.frames.size(); i++)
	{
		KeyframePtr slowFrame = this->createFrame(m_motion.frames[i]);
		
		// Caclulate duration with factor
		double duration = slowFrame->duration * factor;
		if(duration < 1)
			duration = 1; // Minimum possible duration
		
		slowFrame->duration = duration;
		
		tempMotion.frames.push_back(slowFrame);
	}
	
	setCurrentFrame(currentFrame); // Reset current frame back as 'createFrame()' will set it to new frame
	
	// Update & play motion
	requestUpdate(tempMotion, TYPE_MOTION); 
	requestPlay(tempMotion, TYPE_MOTION);
}

void KeyframeModel::playFrame()
{
	if (!m_currentFrame)
		return;
	
	requestPlayFrame(m_currentFrame, 1);
}

void KeyframeModel::playFrameSlow()
{
	if (!m_currentFrame)
		return;
	
	requestPlayFrame(m_currentFrame, headerView->getFactor());
}

void KeyframeModel::requestPlayFrame(KeyframePtr frame, double factor)
{
	// Construct temp motion
	motionfile::Motion tempMotion;
	tempMotion.motionName = "FramePlay";
	tempMotion.jointList = m_motion.jointList;
	tempMotion.playState = "init";
	tempMotion.preState =  "init";
	tempMotion.postState = "init";
	
	// Construct temp frame with scaled duration
	KeyframePtr currentFrame = m_currentFrame;
	KeyframePtr frameToPlay = this->createFrame(frame);
	setCurrentFrame(currentFrame); // Reset current frame back as 'createFrame()' will set it to new frame
	
	// Caclulate duration with factor
	double duration = frameToPlay->duration * factor;
	if(duration < 1)
		duration = 1; // Minimum possible duration
		
	ROS_INFO("Request play frame slow with duration: %f", (float)duration);
	ROS_INFO("Support coefs: %s", frameToPlay->support.c_str());
	
	frameToPlay->duration = duration;
	
	tempMotion.frames.push_back(frameToPlay);
	tempMotion.frames.push_back(frameToPlay);
	
	 // Update & play frame
	requestUpdate(tempMotion, TYPE_FRAME);
	requestPlay(tempMotion, TYPE_FRAME);
}

KeyframeModel::~KeyframeModel()
{
	delete m_robot;
}
