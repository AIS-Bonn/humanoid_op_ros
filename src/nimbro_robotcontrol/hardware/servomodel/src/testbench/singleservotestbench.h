#ifndef SINGLESERVOTESTBENCH_H
#define SINGLESERVOTESTBENCH_H

#include <QtGui>
#include "ui_singleservotestbench.h"
#include "Keyframe.h"
#include "KeyframePlayer.h"
#include "Vec2f.h"
#include "StopWatch.h"
#include "valuelogger.h"
#include "plot.h"
#include <servomodel/torqueestimator.h>

#include <boost/shared_ptr.hpp>

class CommandFitter;
const double PROCESS_PERIOD = 0.02; // s
const double PROCESS_FREQ = 1.0 / PROCESS_PERIOD;

class ParametrizedObject;
class TunableCommandGenerator;
class IJointInterface;
class Trajectory;

class SingleServoTestbench : public QWidget
{
    Q_OBJECT

    Ui::SingleServoTestbench ui;

	int dragging;
	Vec2f drag;
	bool showLinear;
	double size;

	Keyframe motionState;
	double timeStamp;

	bool showAxes;
	double screenScale;
	QPointF screenOffset;
	QTransform screenTransform;

	StopWatch stopWatch;

	bool mousePresent; // Is the mouse on the widget or not.
	QPointF mouse; // current mouse location in pixel coordinates
	QPointF mappedMouse; // current mouse location in logical coordinates
	QPointF lastMouse; // last mouse location in pixel coordinates
	QPointF mouseDiff; // last mouse motion in pixel coordinates
	QPointF mappedMouseDiff; // last mouse motion in logical coordinates
	QPointF mouseVelocity; // estimated mouse velocity in pixels per second
	QPointF mappedMouseVelocity; // estimated mouse velocity in logical units per second
	QPointF mouseClick; // mouse click location in pixel coordinates
	double mouseClickTimeStamp;
	double lastMouseUpdateTimeStamp;

	double mouseCatchRadius;
	QPointF swipeFadeOutVelocity;
	double lastSwipeFadeOutTimeStamp;
	bool dragMoved;
	int draggedKeyframeId;
	int nearestKeyframeId;
	QTimer swipeFadeOutTimer;

	IJointInterface* m_jointInterface;

	Plot m_plot;
	ValueLogger *m_angleLogger;
	ValueLogger* m_cmdLogger;
	ValueLogger* m_modelCMDLogger;
	ValueLogger* m_trajVelLogger;
	ValueLogger* m_trajAccLogger;
	ValueLogger* m_velLogger;
	ValueLogger* m_accLogger;
	ValueLogger* m_loadLogger;
	ValueLogger* m_currentLogger;
	ValueLogger* m_deviationLogger;
	ValueLogger* m_trajPosLogger;
	ValueLogger* m_origCMDLogger;
	ValueLogger* m_outTorqueLogger;

	Trajectory* m_trajectory;
	TunableCommandGenerator* m_commandGenerator;
	CommandFitter* m_commandFitter;
	boost::shared_ptr<TorqueEstimator> m_torqueEstimator;

	QVector<double> m_discreteTrajectory;

	int m_writeCounter;

public:
	SingleServoTestbench(IJointInterface* iface, const boost::shared_ptr<TorqueEstimator>& torqueEstimator, QWidget *parent = 0);
	~SingleServoTestbench();

	inline const Trajectory& trajectory() const
	{ return *m_trajectory; }

	inline TunableCommandGenerator* generator()
	{ return m_commandGenerator; }

	inline CommandFitter* fitter()
	{ return m_commandFitter; }

	inline IJointInterface* jointInterface()
	{ return m_jointInterface; }

	void takeTrajectory(Trajectory* traj);
	void setModelTrajectory(const QVector<double>& cmd);

	void resetOutsideTorques();
	void logOutsideTorque(double time);

Q_SIGNALS:
	void trajectoryChanged();

public Q_SLOTS:
	void doStep(double time);
	void reset();
	void learn(bool checked);
	void read();
	void write();
	void saveSettings();
	void loadSettings();
	void updateTrajectory();
	void updateStatistics();

private Q_SLOTS:
	void swipeFadeOut();
	void initTrajectory();
	void setControlMode(int idx);
	void transferModelCMD();
	void fitModel();

protected:
	void paintEvent(QPaintEvent*);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseDoubleClickEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
	void leaveEvent(QEvent *event);
	void resizeEvent(QResizeEvent *event);

private:
	void updateMouse(QPointF mousePos);
	void updateScreenTransform();

	double targetValue(double time);
	double presentValue();

	void addSlidersForObject(ParametrizedObject* obj);
};

#endif // DYNAMICKEYFRAMEINTERPOLATION_H
