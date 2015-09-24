//Displays trajectories
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#ifndef KEYFRAME_MODEL_H
#define KEYFRAME_MODEL_H

#include <QtGui/QWidget>
#include <QtCore/QAbstractListModel>
#include <QTabWidget>
#include <QItemSelection>
#include <QScrollArea>

#include <urdf/model.h>
#include <rbdl/rbdl_parser.h>
#include <motion_file/motionfile.h>

#include <trajectory_editor_2/robotdisplay.h>
#include <trajectory_editor_2/jointdisplay.h>
#include <trajectory_editor_2/abstractspace.h>
#include <trajectory_editor_2/inversespace.h>

class InvKinBox;
class MainWindow;
class HeaderView;
class FrameView;

class KeyframeModel : public QAbstractListModel
{
Q_OBJECT
public:
	typedef boost::shared_ptr<motionfile::Keyframe> KeyframePtr;
	
	explicit KeyframeModel(QObject* parent = 0);
	virtual ~KeyframeModel();
	
	void initRobot(RobotDisplay* robot);
	void initJointDisplay(QTabWidget *jointsTabWidget, QTabWidget *headerTabWidget);

	QVariant        data(const QModelIndex &index, int role = Qt::DisplayRole) const;
	int             rowCount(const QModelIndex &parent = QModelIndex()) const;
	QStringList     mimeTypes() const;
	QMimeData*      mimeData(const QModelIndexList &indexes) const;
	Qt::DropActions supportedDropActions() const;
	Qt::ItemFlags   flags(const QModelIndex &index) const;
	
	bool dropMimeData(const QMimeData *data, Qt::DropAction action, int row, int column, const QModelIndex &parent);
	bool removeRows(int row, int count, const QModelIndex &parent = QModelIndex());
	//bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole);
	
	void insertRow(int row, KeyframePtr frame, const QModelIndex &parent = QModelIndex());
	
	void removeFrame(unsigned position);
	void moveFrame(unsigned position, int direction); // Move frame from position by +1/-1 within the vector
	
Q_SIGNALS:
	void setViewSelection(int row);
	void setViewSelection(QItemSelection selection);

public Q_SLOTS:
	void save();
	void saveAs();
	void saveMirroredAs();
	void load();
	void addFrame();
	void createMotion();
	
	void playMotion();
	void playMotionSLow();
	void playFrame();
	void playFrameSlow();
	
	void updateHeaderData(); // Update name, states, etc
	void updateRobotDisplay();
	void handleIndexChange(const QModelIndex& index);
	void handleChangeEffortForAllFrames(std::vector<int> indexes);
	void handleUpdateFrame();
	void handeFrameLoaded(KeyframePtr frame);
	
	void handleTabChange(int id);
	
private:
	int findNextID(); // Find valid ID which is not used yet
	int idToPosition(int id); // Find frame's position in vector by its ID
	void setCurrentFrame(KeyframePtr frame);
	QScrollArea* createAreaWithWidget(QWidget *widget);
	
	void requestPlayFrame(KeyframePtr frame, double factor); // Play frame with duration*factor
	void requestPlay(motionfile::Motion &motion, int type);
	void requestUpdate(motionfile::Motion &motion, int type);
	
	KeyframePtr createFrame();
	KeyframePtr createFrame(KeyframePtr oldFrame);
	
	// Mirror values of specified joint to create a mirrored motion
	void mirrorValues(KeyframePtr frame, KeyframePtr mirrored, std::string 
		 left, std::string right, bool swap, bool invert);

private:
    MainWindow   *m_parent;
	
	HeaderView   *headerView;
	FrameView    *frameView;
	
	JointManager  *m_jointManager;
	AbstractSpace *abstractSpace;
	InverseSpace  *inverseSpace;
	
	RobotDisplay* m_robot;
	boost::shared_ptr<rbdl_parser::URDF_RBDL_Model> m_rbdl;

	motionfile::Motion m_motion;
	KeyframePtr m_currentFrame;
	
	QString filePath;
	bool m_fileActive;

	ros::ServiceClient m_cl_update;
	ros::ServiceClient m_cl_play;
};


#endif
