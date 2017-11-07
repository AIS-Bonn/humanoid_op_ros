// Handles operations on keyframes
// Authors: Max Schwarz <max.schwarz@uni-bonn.de>, Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef KEYFRAME_MODEL_H
#define KEYFRAME_MODEL_H

#include <QAbstractListModel>
#include <QItemSelection>

#include <motion_file/motionfile.h>

#include <trajectory_editor/jointperspective.h>
#include <trajectory_editor/robotdisplay.h>
#include <trajectory_editor/headerview.h>

class KeyframeModel : public QAbstractListModel
{
Q_OBJECT
public:
	typedef boost::shared_ptr<motionfile::Keyframe> KeyframePtr;
	
	explicit KeyframeModel(QObject* parent = 0);
	virtual ~KeyframeModel();
	
	int rowCount(const QModelIndex &parent = QModelIndex()) const;
	
	void removeFrame(unsigned position);
	void moveFrame(unsigned position, int direction); // Move frame from position by +1/-1 within the vector
	
	void copySelectedFrames(QModelIndexList selectedIndexes);
	void insertCopiedFrames();
	
	void setSaved();
	void setMotion(motionfile::Motion motion);
	
	joint_perspective::PerspectiveManager* getPerspectiveManager();
	
	KeyframePtr getCurrentFrameCopy();
	
	motionfile::Motion& getMotion();
	motionfile::Motion  getMotionCopy();
	std::vector<std::string> getJointList();
	
	bool hasMotion();
	bool hasUnsavedChanges();
	
Q_SIGNALS:
	void setViewSelection(int row);
	void setViewSelection(QItemSelection selection);
	void updateCurrentFileLabel(QString newTitle);
	
	void currentFrameChanged(KeyframePtr frame);
	void jointListChanged(std::vector<std::string> list);
	void perspectiveChanged(const joint_perspective::JointPerspective& perspective);
	
	void headerDataChanged(HeaderData);
	
	void gotMotion(bool); // Emmited when model got motion
	void gotRules(const std::vector<motionfile::Rule> &rules);
	
	void updateRobot(std::vector<std::string> &joint_list, std::vector<double> &joint_positions);
	void modelChanged(std::string model_path);
	
	void getInverseLimits(bool &limit, double &epsilon);

public Q_SLOTS:
	void addFrame(int row);
	void showMotionInFolder(); // Show current motion file in folder
	
	void updateHeaderData(HeaderData); // Update name, states, etc
	void updateRobotDisplay();
	
	void handleIndexChange(const QModelIndex& index);
	void handleChangeEffortForAllFrames(std::vector<int> indexes, double newEffort);
	void handleUpdateFrame();
	void handeFrameLoaded(KeyframePtr frame);
	void disableFrame(unsigned position); // Disables specified frame if its enabled, wise wersa otherwise
	
	void handleTabChange(int id);
	
	void applyRule(double delta, int rule_id);
	
	void estimateVelocity();
	
private:
	int findNextID(); // Find valid ID which is not used yet
	int idToPosition(int id); // Find frame's position in vector by its ID
	void setCurrentFrame(KeyframePtr frame);
	
	KeyframePtr createFrame();
	KeyframePtr createFrame(KeyframePtr oldFrame, bool copyNameAndID = false);
	
	QVariant        data(const QModelIndex &index, int role = Qt::DisplayRole) const;
	QStringList     mimeTypes() const;
	QMimeData*      mimeData(const QModelIndexList &indexes) const;
	Qt::DropActions supportedDropActions() const;
	Qt::ItemFlags   flags(const QModelIndex &index) const;
	
	bool dropMimeData(const QMimeData *data, Qt::DropAction action, int row, int column, const QModelIndex &parent);
	bool removeRows(int row, int count, const QModelIndex &parent = QModelIndex());
	void insertRow(int row, KeyframePtr frame, const QModelIndex &parent = QModelIndex());

private:
	joint_perspective::PerspectiveManager *m_perspective_manager;

	motionfile::Motion m_motion;
	KeyframePtr m_current_frame;
	
	// Dump of motion at a time it is being loaded. Used to define if there are some changes of motion
	std::string m_canonic_dump; 
	
	bool m_has_motion;
	
	std::vector<KeyframePtr> m_copied_frames;
	std::vector<std::string> m_copied_joint_list;
};


#endif
