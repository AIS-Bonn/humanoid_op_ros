// Handles operations on keyframes
// Authors: Max Schwarz <max.schwarz@uni-bonn.de>, Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <QMessageBox>
#include <QStringList>
#include <QMimeData>
#include <QDesktopServices>
#include <QUrl>

#include <algorithm>
#include <stdio.h>

#include <ros/package.h>
#include <ros/console.h>

#include <trajectory_editor/keyframemodel.h>

// Consts for drag and drop methods
const char* FRAME_MIME_TYPE = "application/vnd.nimbro.frame";
const int   DEFAULT_INSERT  = -1;

KeyframeModel::KeyframeModel(QObject* parent)
	: QAbstractListModel(parent)
{
	m_has_motion = false;
	m_perspective_manager = new joint_perspective::PerspectiveManager();
	
	ROS_INFO("initialized successfully");
}

bool KeyframeModel::hasMotion()
{
	return m_has_motion;
}

bool KeyframeModel::hasUnsavedChanges()
{
	return m_motion.dump() != m_canonic_dump;
}

void KeyframeModel::setSaved()
{
	m_canonic_dump = m_motion.dump();
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
    if(index.isValid())
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

// TODO implement it in a nicer way
bool smaller(int i,int j) { return (i<j); }
bool larger (int i,int j) { return (i>j); }

bool KeyframeModel::dropMimeData(const QMimeData *data, Qt::DropAction action, int row, int column, const QModelIndex &parent)
{
	if(action == Qt::IgnoreAction)
		return true;
	
	if(!data->hasFormat(FRAME_MIME_TYPE))
		return false;
	
	if(column > 0)
		return false;
	
	int positionToDrop;
	
	if(row != -1)
		positionToDrop = row;
	else if(parent.isValid())
		positionToDrop = parent.row();
	else
		positionToDrop = rowCount(QModelIndex());
	
	if(positionToDrop >= (int)m_motion.frames.size())
		positionToDrop = m_motion.frames.size() - 1;
	
	QByteArray encodedData = data->data(FRAME_MIME_TYPE);
	QDataStream stream(&encodedData, QIODevice::ReadOnly);
	std::vector<int> ids;
	std::vector<int> positions;
	
	QItemSelection selection;
	
	// Read ids which need to be dropped
	while(!stream.atEnd()) 
	{
		int id = 0;
		stream >> id;
		
		ids.push_back(id);
		positions.push_back(idToPosition(id));
	}
	
	// Sort the elements to be dropped
	int minPos = *std::min_element(positions.begin(), positions.end());
	
	if(minPos < positionToDrop)
		std::sort(positions.begin(), positions.end(), smaller);
	else
		std::sort(positions.begin(), positions.end(), larger);
	
	for(size_t j = 0; j < ids.size(); j++)
			ids.at(j) = m_motion.frames[positions.at(j)]->id;
	
	// Insert elements which need to be dropped at new position
	for(size_t j = 0; j < ids.size(); j++)
	{
		int position = idToPosition(ids.at(j));
		KeyframePtr frameToDrop = m_motion.frames[position];
		
		removeRows(position, 1, parent); // Remove, then insert
		insertRow(positionToDrop, frameToDrop, parent);
	}
	
	// Create selection for dropped items
	if(idToPosition(ids.at(0)) < positionToDrop)
		positionToDrop = positionToDrop - ids.size() + 1;
	
	for(size_t j = 0; j < ids.size(); j++) // Select
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

// Show current motion in folder
void KeyframeModel::showMotionInFolder()
{
	if(m_motion.motionName.empty()) // Open default location
	{
		QDesktopServices::openUrl((QUrl(
			QString::fromStdString(ros::package::getPath("launch") + "/motions"), QUrl::TolerantMode)));
		
		return;
	}
	
	// Get path to dir with current motion
	QStringList pieces = QString::fromStdString(m_motion.filePath).split("/");
	
	QString pathNoExtension;
	
	for(int i = 0; i < pieces.size()-1; i++)
		pathNoExtension.append(pieces.at(i) + "/");
	
	QDesktopServices::openUrl((QUrl(pathNoExtension, QUrl::TolerantMode)));
}

KeyframeModel::KeyframePtr KeyframeModel::createFrame()
{
	KeyframePtr newFrame(new motionfile::Keyframe);
	newFrame->duration = 0;
	newFrame->id = findNextID();
	
	newFrame->roll  = 0;
	newFrame->pitch = 0;
	newFrame->yaw   = 0;
	
	std::stringstream s;
	s << newFrame->id;
	newFrame->name = std::string("Frame ").append(s.str());
	
	for(unsigned i = 0; i < m_motion.jointList.size(); i++)
	{
		motionfile::FrameJoint newJoint;
		newJoint.effort = 1;
		newJoint.position = 0;
		newFrame->joints.push_back(newJoint);
	}

	return newFrame;
}

KeyframeModel::KeyframePtr KeyframeModel::createFrame(KeyframeModel::KeyframePtr oldFrame, bool copyNameAndID)
{
	if(!oldFrame)
		return createFrame();

	KeyframePtr newFrame(new motionfile::Keyframe);
	newFrame->duration = oldFrame->duration;
	newFrame->support = oldFrame->support;
	
	if(copyNameAndID)
		newFrame->id = oldFrame->id;
	else
		newFrame->id = findNextID();
	
	newFrame->roll  = oldFrame->roll;
	newFrame->pitch = oldFrame->pitch;
	newFrame->yaw   = oldFrame->yaw;
	
	if(copyNameAndID)
		newFrame->name = oldFrame->name;
	else
	{
		std::stringstream s;
		s << newFrame->id;
		newFrame->name = std::string("Frame ").append(s.str());
	}
	
	for (unsigned i = 0; i < oldFrame->joints.size(); i++)
	{
		motionfile::FrameJoint newJoint(oldFrame->joints[i]);
		newFrame->joints.push_back(newJoint);
	}
	
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
	
	m_current_frame = frame;
	currentFrameChanged(m_current_frame);
}

void KeyframeModel::updateHeaderData(HeaderData data)
{
	m_motion.motionName = data.motion_name;
	m_motion.preState   = data.pre_state;
	m_motion.playState  = data.play_state;
	m_motion.postState  = data.post_state;
	
	m_motion.pidEnabled = data.pid_enabled;
}

void KeyframeModel::moveFrame(unsigned position, int direction)
{
	if(direction != 1 && direction != -1)
		return;
	
	int nextPos = position + direction;
	std::iter_swap(m_motion.frames.begin() + position, m_motion.frames.begin() + nextPos);
}

void KeyframeModel::addFrame(int row)
{	
	if(m_motion.motionName.empty() || row < 0)
		return;
	
	beginInsertRows(QModelIndex(), row, row);
	m_motion.frames.insert(m_motion.frames.begin() + row, createFrame(m_current_frame));
	endInsertRows();
	
	setViewSelection(row);
	setCurrentFrame(m_motion.frames.at(row));
}

void KeyframeModel::insertCopiedFrames()
{
	if(m_motion.motionName.empty())
		return;
	
	for(unsigned k = 0; k < m_copied_frames.size(); k++)
	{
		// Reorganize joints in frame to match current jointList of opened motion
		KeyframePtr temp = createFrame(m_copied_frames.at(k));
		int index = 0;
		
		for(unsigned i = 0; i < m_copied_joint_list.size(); i++)
		{
			for(unsigned j = 0; j < m_motion.jointList.size(); j++)
			{
				if(m_copied_joint_list.at(i) == m_motion.jointList.at(j))
				{
					index = j;
					break;
				}
				index = i;
			}
			
			m_copied_frames.at(k)->joints[index] = temp->joints[i];
		}
		
		beginInsertRows(QModelIndex(), 0, m_motion.frames.size()-1);
		m_motion.frames.push_back(m_copied_frames.at(k));
		endInsertRows();
		
	}
	
	setViewSelection(m_motion.frames.size()-1);
	setCurrentFrame(m_motion.frames.back());
	
	m_copied_frames.clear();
}

void KeyframeModel::removeFrame(unsigned position)
{
	if(position > m_motion.frames.size() || position < 0)
		return;

	if(m_motion.frames.size() == 1)
	{
		m_current_frame = KeyframePtr();
		
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
	m_current_frame = m_motion.frames[frameIndex];
	
	if(m_current_frame)
		setCurrentFrame(m_current_frame);
}

void KeyframeModel::copySelectedFrames(QModelIndexList selectedIndexes)
{
	m_copied_frames.clear();
	
	for(int i = 0 ; i < selectedIndexes.size(); i++)
	{
		int frameIndex = selectedIndexes.at(i).row();
		
		KeyframePtr copiedFrame = createFrame(m_motion.frames[frameIndex]);
		copiedFrame->name = m_motion.frames[frameIndex]->name;
		
		m_copied_frames.push_back(copiedFrame);
	}
	
	m_copied_joint_list = m_motion.jointList;
}

void KeyframeModel::handleUpdateFrame()
{
	//setCurrentFrame(m_current_frame);
	dataChanged(QModelIndex(),QModelIndex());
}

void KeyframeModel::handeFrameLoaded(KeyframePtr frame)
{
	int pos = this->idToPosition(m_current_frame->id);
	m_motion.frames[pos] = frame;
	
	setCurrentFrame(frame);
}

void KeyframeModel::disableFrame(unsigned position)
{
	if(position > m_motion.frames.size() || position < 0)
		return;
	
	std::string disabled_sequence("##");
	std::size_t found = m_motion.frames[position]->name.find(disabled_sequence);
	
	if(found == std::string::npos || found != 0)
		m_motion.frames[position]->name.insert (0, disabled_sequence);
	else if(found == 0)
		m_motion.frames[position]->name.erase(0, disabled_sequence.length());
	
	handleUpdateFrame();
}

void KeyframeModel::handleChangeEffortForAllFrames(std::vector<int> indexes, double newEffort)
{
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
	
	setCurrentFrame(m_current_frame);
}

void KeyframeModel::handleTabChange(int id)
{
	if(!m_current_frame)
		return;
	
	currentFrameChanged(m_current_frame);
}

void KeyframeModel::updateRobotDisplay()
{
	if(!m_current_frame)
		return;
	
	std::vector<double> positions;
	positions.resize(m_current_frame->joints.size());
	
	for(unsigned i = 0; i < m_current_frame->joints.size(); i++)
		positions.at(i) = m_current_frame->joints[i].position;
	
	// HACK for dynaped   TODO: implement nicely
	if(m_perspective_manager->getCurrentPerspective().m_name == "dynaped_perspective")
	{
		int r_shank = motionfile::Motion::nameToIndex(m_motion.jointList, "right_shank_pitch");
		int r_thigh = motionfile::Motion::nameToIndex(m_motion.jointList, "right_thigh_pitch");
		int r_knee =  motionfile::Motion::nameToIndex(m_motion.jointList, "right_knee_pitch");
		int r_hip =   motionfile::Motion::nameToIndex(m_motion.jointList, "right_hip_pitch");
		
		int l_shank = motionfile::Motion::nameToIndex(m_motion.jointList, "left_shank_pitch");
		int l_thigh = motionfile::Motion::nameToIndex(m_motion.jointList, "left_thigh_pitch");
		int l_knee =  motionfile::Motion::nameToIndex(m_motion.jointList, "left_knee_pitch");
		int l_hip =   motionfile::Motion::nameToIndex(m_motion.jointList, "left_hip_pitch");
		
		positions.at(r_shank) = positions.at(r_knee);
		positions.at(r_thigh) = positions.at(r_hip);
		positions.at(l_shank) = positions.at(l_knee);
		positions.at(l_thigh) = positions.at(l_hip);
	}
	
	updateRobot(m_motion.jointList, positions);
}

void KeyframeModel::applyRule(double delta, int rule_id)
{
	bool limit_inverse;
	double epsilon;
	
	getInverseLimits(limit_inverse, epsilon);
	
	if(m_motion.canApplyRule(rule_id, delta, limit_inverse, epsilon))
	{
		m_motion.applyRule(rule_id, delta, limit_inverse, epsilon);
		
		if(m_current_frame)
			setCurrentFrame(m_current_frame);
	}
}

void KeyframeModel::estimateVelocity()
{
	if(!m_current_frame)
		return;
	
	// Find previous frame
	int prev_id = -1;
	
	for(size_t i = 0; i < m_motion.frames.size(); i++)
		if(m_motion.frames.at(i)->id == m_current_frame->id && i > 0)
			prev_id = i - 1;
		
	if(prev_id == -1)
		return;
	
	// Estimate velocity
	// HACK: hardcoded joints to estimate velocity for
	std::vector<std::string> joints;
	joints.push_back("right_hip_pitch");
	joints.push_back("right_knee_pitch");
	joints.push_back("right_ankle_pitch");
	
	double v1 = 0; // Velocity at the end of previous frame
	double v2 = 0; // To be estimated
	double p1 = 0; // Position at the end of previous frame
	double p2 = 0; // Position at the end of current  frame
	double s  = 0; // Distance
// 	double t  = m_current_frame->duration; // Duration of the movemnt
	double a  = 0.64; // Acceleration used during the movement
	
	for(size_t i = 0; i < joints.size(); i++)
	{
		int joint_id = m_motion.findJoint(joints.at(i));
		
		if(joint_id == -1)
			continue;
		
		v1 = m_motion.frames.at(prev_id)->joints.at(joint_id).velocity;
		p1 = m_motion.frames.at(prev_id)->joints.at(joint_id).position;
		p2 = m_current_frame->joints.at(joint_id).position;
		s = fabs(p2-p1);
		
		v2 = sqrt(v1*v1 + 2*a*s);
		m_current_frame->joints.at(joint_id).velocity = v2;
	}
	
	handleUpdateFrame();
}

void KeyframeModel::setMotion(motionfile::Motion motion)
{
	bool ok = false;
	bool new_perspective = m_perspective_manager->isNewPerspective(motion.perspective, ok);
	
	if(!ok)
	{
		QMessageBox::critical(0, "Error", "Perspective for requested motion is not found!");
		return;
	}
	
	beginResetModel();
	
	m_motion = motion;
	jointListChanged(m_motion.jointList);
	
	if(new_perspective)
	{
		perspectiveChanged(m_perspective_manager->getCurrentPerspective());
		modelChanged(m_perspective_manager->getCurrentPerspective().m_model);
	}
	
	if(m_motion.frames.size() > 0)
		setCurrentFrame(m_motion.frames[0]);
	
	m_has_motion = true;
	gotMotion(m_has_motion);
	gotRules(m_motion.rules);
	
	m_canonic_dump = motion.dump();
	
	// Update views
	HeaderData data;
	data.motion_name = m_motion.motionName;
	data.pre_state   = m_motion.preState;
	data.play_state  = m_motion.playState;
	data.post_state  = m_motion.postState;
	data.pid_enabled = m_motion.pidEnabled;
	
	headerDataChanged(data);
	updateCurrentFileLabel(QString::fromStdString(m_motion.motionName).append(".yaml"));
	
	endResetModel();
}

motionfile::Motion& KeyframeModel::getMotion()
{
	return m_motion;
}

motionfile::Motion KeyframeModel::getMotionCopy()
{
	motionfile::Motion motionCopy;
	motionCopy = m_motion;
	motionCopy.frames.clear();
	
	for(unsigned i = 0; i < m_motion.frames.size(); i++)
		motionCopy.frames.push_back(createFrame(m_motion.frames[i], true));
	
	return motionCopy;
}

KeyframeModel::KeyframePtr KeyframeModel::getCurrentFrameCopy()
{
	if(!m_current_frame)
		return createFrame();
	
	return createFrame(m_current_frame);
}

std::vector< std::string > KeyframeModel::getJointList()
{
	return m_motion.jointList;
}

joint_perspective::PerspectiveManager* KeyframeModel::getPerspectiveManager()
{
	return m_perspective_manager;
}

KeyframeModel::~KeyframeModel()
{
	delete m_perspective_manager;
}
