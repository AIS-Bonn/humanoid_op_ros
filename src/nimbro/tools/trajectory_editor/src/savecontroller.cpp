// Performs saving/loading motions
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <trajectory_editor/savecontroller.h>

SaveController::SaveController(HeaderView *view, joint_perspective::PerspectiveManager *manager)
	: QObject()
{
	m_header_view = view;
	m_perspective_manager = manager;
}

SaveController::~SaveController()
{
	
}

QString SaveController::newMotion(motionfile::Motion& motion)
{
	// Choose which robot model to use
	std::string model;
	std::string perspective;
	QStringList robots;
	const std::vector<joint_perspective::JointPerspective> perspectives = m_perspective_manager->getPerspectives();
	
	for(unsigned i = 0; i < perspectives.size(); i++)
		robots << perspectives.at(i).m_robot_name.c_str();

	bool ok;
	QString item = QInputDialog::getItem(0, tr("Choose a robot"), tr("Choose a robot:"), robots, 0, false, &ok);
	
	if(ok && !item.isEmpty())
	{
		for(unsigned i = 0; i < perspectives.size(); i++)
			if(item.toStdString() == perspectives.at(i).m_robot_name)
			{
				model = perspectives.at(i).m_model;
				perspective = perspectives.at(i).m_name;
			}
	}
	else
		return "";
	
	// Set robot model and obtain new joint list
	std::vector<std::string> new_joint_list;
	modelChanged(model, new_joint_list);
	
	// Get filename and path for new motion
	QString dir  = QString::fromStdString(ros::package::getPath("launch") + "/motions/newMotion.yaml");
    QString path = QFileDialog::getSaveFileName(0, tr("Create new motion"), dir, tr("Motion files(*.yaml)"));
	
	if(path.isEmpty())
		return "";
	
	// Set up new motion
	motion.motionName  = getFileNameFromPath(path);
	motion.preState    = "";
	motion.playState   = "";
	motion.postState   = "";
	motion.pidEnabled  = false;
	motion.filePath    = path.toStdString();
	motion.perspective = perspective;
	
	// Get joint list from the model
	motion.jointList.clear();
	motion.jointList.resize(new_joint_list.size());
	
	for (unsigned i = 0; i < new_joint_list.size(); i++)
	{
		std::cout << "Joint in model: " << new_joint_list.at(i).c_str() << "\n";
		motion.jointList[i] = new_joint_list.at(i);
	}
	
	motion.frames.clear();
	motion.frames.push_back(firstFrame(new_joint_list.size()));
	
	motion.save(path.toStdString());
	newPath(path);
	
	return path;
}

QString SaveController::open(motionfile::Motion &motion)
{
	// Get path of file to load
	QString dir  = QString::fromStdString(ros::package::getPath("launch") + "/motions/");
    QString path = QFileDialog::getOpenFileName(0, tr("Open motion"), dir, QString("*.yaml"));
	
	if(path.isEmpty())
		return "";

    return open(motion, path);
}

QString SaveController::open(motionfile::Motion& motion, QString path)
{
	if(motion.load(path.toStdString()) == false)
		return "";
	
	// Set up id and frame names if necessary
	for(int i = 0; i < (int)motion.frames.size(); i++)
	{
		KeyframePtr temp = motion.frames[i];
		temp->id = i+1;
		
		if(temp->name == "") // If there is no saved frame name - assign one
		{
			std::stringstream s;
			s << temp->id;
			temp->name = std::string("Frame ").append(s.str());
		}
	}
	
	// Set motion name
	motion.motionName = getFileNameFromPath(path);
	
	newPath(path);
	
	return path;
}

void SaveController::save(motionfile::Motion& motion)
{
	if (isMotionValid(motion) == false)
		return;

	if(m_header_view->requestSave()) // Save only if everything in header is ok
		motion.save(motion.filePath);
}

void SaveController::saveAs(motionfile::Motion& motion)
{
	if (isMotionValid(motion) == false)
		return;
	
	// Get filename to 'save as'
	QString dir = QString::fromStdString(ros::package::getPath("launch") + "/motions/savedMotion.yaml");
	QString path = QFileDialog::getSaveFileName(0, tr("Save motion as"), dir, tr("Motion files(*.yaml)"));
	
	if(m_header_view->requestSave()) // Save only if everything in header is ok
	{
		std::string currentName = motion.motionName;
		
		motion.motionName = m_header_view->getFileNameFromPath(path);
		motion.save(path.toStdString());
		
		motion.motionName = currentName;
	}
}

void SaveController::saveMirroredAs(motionfile::Motion& original_motion, motionfile::Motion& mirrored)
{
	if (isMotionValid(original_motion) == false)
		return;
	
	// Get filename to 'save as'
	QString dir = QString::fromStdString(ros::package::getPath("launch") + "/motions/"
			+ m_header_view->getNameForMirrored() + ".yaml");
	QString path = QFileDialog::getSaveFileName(0, tr("Save mirrored motion as"), dir, tr("Motion files(*.yaml)"));
	
	// Prepare mirrored frames
	for(unsigned i = 0; i < mirrored.frames.size(); i++)
	{
		KeyframePtr mirroredFrame = mirrored.frames[i];

		// Mirror per frame roll,pitch,yaw
		mirroredFrame->roll  = -original_motion.frames[i]->roll;
		mirroredFrame->pitch =  original_motion.frames[i]->pitch;
		mirroredFrame->yaw   = -original_motion.frames[i]->yaw;
		
		// Mirror joints
		mirrorValues(original_motion.frames[i], mirroredFrame, "neck_yaw", "neck_yaw", false, true, mirrored.jointList);
        
        mirrorValues(original_motion.frames[i], mirroredFrame, "left_elbow_pitch", "right_elbow_pitch", true, false, mirrored.jointList);
		mirrorValues(original_motion.frames[i], mirroredFrame, "left_shoulder_pitch", "right_shoulder_pitch", true, false, mirrored.jointList);
		mirrorValues(original_motion.frames[i], mirroredFrame, "left_hip_pitch", "right_hip_pitch", true, false, mirrored.jointList);
		mirrorValues(original_motion.frames[i], mirroredFrame, "left_ankle_pitch", "right_ankle_pitch", true, false, mirrored.jointList);
		mirrorValues(original_motion.frames[i], mirroredFrame, "left_knee_pitch", "right_knee_pitch", true, false, mirrored.jointList);
		
		mirrorValues(original_motion.frames[i], mirroredFrame, "left_hip_roll", "right_hip_roll", true, true, mirrored.jointList);
		mirrorValues(original_motion.frames[i], mirroredFrame, "left_shoulder_roll", "right_shoulder_roll", true, true, mirrored.jointList);
		mirrorValues(original_motion.frames[i], mirroredFrame, "left_ankle_roll", "right_ankle_roll", true, true, mirrored.jointList);
		mirrorValues(original_motion.frames[i], mirroredFrame, "left_hip_yaw", "right_hip_yaw", true, true, mirrored.jointList);
		
		// Mirror support coefs
		if(original_motion.frames[i]->support != "")
		{
			// Parse support coefs
			double left;
			double right;
			
			if(FrameView::parseSupport(QString::fromStdString(original_motion.frames[i]->support), left, right) == false)
			{
				continue;
			}
			
			std::ostringstream ss;
			ss << 1 - left;
			ss << " ";
			ss << 1 - right;
			
			mirroredFrame->support = ss.str();;
		}
	}
	
	if(m_header_view->requestSave()) // Save only if everything in header is ok
	{
		mirrored.motionName = getFileNameFromPath(path);
		mirrored.save(path.toStdString());
	}
}

void SaveController::saveBackup(motionfile::Motion& motion)
{
	if (isMotionValid(motion) == false)
		return;
	
	QString backUpPath = QString("/var/log/nimbro/trajectory_editor");
	if(QDir(backUpPath).exists() == false)
		QDir().mkdir(backUpPath);
	
	backUpPath.append("/");
	backUpPath.append(QString::fromStdString(motion.motionName));
	
	QString time = QString("_%1").arg(QTime::currentTime().toString());
	backUpPath.append(time);
	
	backUpPath.append(".yaml");
	
	motion.save(backUpPath.toStdString());
}

SaveController::KeyframePtr SaveController::firstFrame(unsigned joints_amount)
{
	KeyframePtr first_frame(new motionfile::Keyframe);
	first_frame->name = std::string("Frame 1");
	first_frame->duration = 1;
	first_frame->id = 1;
	
	first_frame->roll  = 0;
	first_frame->pitch = 0;
	first_frame->yaw   = 0;
	
	for (unsigned i = 0; i < joints_amount; i++)
	{
		motionfile::FrameJoint newJoint;
		newJoint.effort = 0;
		newJoint.position = 0;
		first_frame->joints.push_back(newJoint);
	}

	return first_frame;
}

std::string SaveController::getFileNameFromPath(QString path)
{
	QStringList pieces = path.split("/");
	QStringList pieces2 = pieces.last().split(".");
	
	return pieces2.first().toStdString();
}

void SaveController::showMessageBox(QString title, QString text)
{
	QMessageBox messageBox;
	messageBox.setWindowTitle(title);
	messageBox.setText(text);
	messageBox.exec();
}

bool SaveController::isMotionValid(motionfile::Motion& motion)
{
	if (motion.jointList.empty()) // If joint list is empty - dont save, show error
	{
		showMessageBox(tr("Fatal error!"), tr("Joint list is empty!\nPlease, report the bug: dm.mark999@gmail.com"));
		return false;
	}
	
	return true;
}

void SaveController::mirrorValues(KeyframePtr frame, KeyframePtr mirrored, std::string 
		 left, std::string right, bool swap, bool invert, std::vector<std::string> jointList)
{
	int leftID;
	int rightID;
	
	double leftPos;
	double rightPos;
	
	if(swap == false) // Just invert the specified joint's position
	{
		leftID = motionfile::Motion::nameToIndex(jointList, left);
		
		// Position
		mirrored->joints[leftID].position = -frame->joints[leftID].position;
		
		// PID
		mirrored->joints[leftID].pGain = -frame->joints[leftID].pGain; 
		mirrored->joints[leftID].iGain = -frame->joints[leftID].iGain; 
		mirrored->joints[leftID].dGain = -frame->joints[leftID].dGain; 
		
		return;
	}
	
	// Swap the positions of left and right
	leftID = motionfile::Motion::nameToIndex(jointList, left);
	leftPos = frame->joints[leftID].position; 
	
	rightID = motionfile::Motion::nameToIndex(jointList, right);
	rightPos = frame->joints[rightID].position; 
	
	if(invert == false) // Just swap
	{
		// Position
		mirrored->joints[leftID].position = rightPos;
		mirrored->joints[rightID].position = leftPos;
		
		// PID
		mirrored->joints[leftID].pGain = frame->joints[rightID].pGain; 
		mirrored->joints[leftID].iGain = frame->joints[rightID].iGain; 
		mirrored->joints[leftID].dGain = frame->joints[rightID].dGain; 
		
		mirrored->joints[rightID].pGain = frame->joints[leftID].pGain; 
		mirrored->joints[rightID].iGain = frame->joints[leftID].iGain; 
		mirrored->joints[rightID].dGain = frame->joints[leftID].dGain; 
	}
	else // Swap and invert
	{
		// Position
		mirrored->joints[leftID].position = -rightPos;
		mirrored->joints[rightID].position = -leftPos;
		
		// PID
		mirrored->joints[leftID].pGain = -frame->joints[rightID].pGain; 
		mirrored->joints[leftID].iGain = -frame->joints[rightID].iGain; 
		mirrored->joints[leftID].dGain = -frame->joints[rightID].dGain; 
		
		mirrored->joints[rightID].pGain = -frame->joints[leftID].pGain; 
		mirrored->joints[rightID].iGain = -frame->joints[leftID].iGain; 
		mirrored->joints[rightID].dGain = -frame->joints[leftID].dGain; 
	}
	
	// PID flags
	mirrored->joints[leftID].gainSelect  = frame->joints[rightID].gainSelect; 
	mirrored->joints[rightID].gainSelect = frame->joints[leftID].gainSelect; 
}

