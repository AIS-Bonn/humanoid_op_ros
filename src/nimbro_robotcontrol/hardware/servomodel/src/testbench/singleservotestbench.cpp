#include "singleservotestbench.h"
#include "globals.h"
#include "ijointinterface.h"
#include "trajectory.h"
#include "sinustrajectory.h"
#include "compositetrajectory.h"
#include "tunablecommandgenerator.h"
#include "statisticswidget.h"
#include "commandfitter.h"

SingleServoTestbench::SingleServoTestbench(
	IJointInterface* iface,
	const boost::shared_ptr<TorqueEstimator>& torqueEstimator,
	QWidget *parent
) : QWidget(parent)
  , m_torqueEstimator(torqueEstimator)
{
	ui.setupUi(this);

	connect(ui.learnButton, SIGNAL(clicked(bool)), this, SLOT(learn(bool)));
	connect(ui.writeButton, SIGNAL(clicked()), this, SLOT(write()));
	connect(ui.readButton, SIGNAL(clicked()), this, SLOT(read()));
	connect(ui.transferButton, SIGNAL(clicked()), this, SLOT(transferModelCMD()));
	connect(ui.fitButton, SIGNAL(clicked()), SLOT(fitModel()));

	setFocus();
	setMouseTracking(true);

	m_jointInterface = iface;

	for(int i = 0; i < IJointInterface::CT_NUM_TYPES; ++i)
		ui.controlModeBox->addItem(IJointInterface::CONTROL_TYPE_NAMES[i]);

	connect(ui.controlModeBox, SIGNAL(activated(int)), SLOT(setControlMode(int)));
	m_jointInterface->setControlType(IJointInterface::CT_POSITION);

	size = 30;
	dragging = -1;
	showLinear = false;

	showAxes = true;
	screenScale = 0.01; // 1 px is worth 0.01 value
	screenOffset = QPointF(2,1);
	mouseCatchRadius = 6.0;
	mouseClickTimeStamp = 0;
	dragMoved = false;
	draggedKeyframeId = -1;
	nearestKeyframeId = -1;
	mousePresent = false;
	lastSwipeFadeOutTimeStamp = 0;
	lastMouseUpdateTimeStamp = 0;
	swipeFadeOutTimer.setInterval(20);
	connect(&swipeFadeOutTimer, SIGNAL(timeout()), this, SLOT(swipeFadeOut()));

	ui.plotLegend->setPlot(&m_plot);
	connect(ui.plotLegend, SIGNAL(plotSettingsChanged()), SLOT(update()));
	m_angleLogger = m_plot.addLogger("Measured angle");
	m_cmdLogger = m_plot.addLogger("Servo command");
	m_cmdLogger->setColor(Qt::green);

	m_modelCMDLogger = m_plot.addLogger("Model command");
	m_modelCMDLogger->setColor(qRgb(0x08,0x75,0x6E));

	m_origCMDLogger = m_plot.addLogger("Manual model command");
	m_origCMDLogger->setColor(Qt::black);

	m_trajPosLogger = m_plot.addLogger("Trajectory position");
	m_trajPosLogger->setColor(qRgb(0xFF, 0xFF, 0));

	m_trajVelLogger = m_plot.addLogger("Trajectory velocity");
	m_trajVelLogger->setColor(Qt::blue);

	m_trajAccLogger = m_plot.addLogger("Trajectory acceleration");
	m_trajAccLogger->setColor(qRgb(0xE0, 0x9B, 0x1B));

	m_velLogger = m_plot.addLogger("Velocity");
	m_velLogger->setColor(Qt::black);

	m_accLogger = m_plot.addLogger("Acceleration");
	m_accLogger->setColor(qRgb(0, 0xFF, 0xFF));

	m_outTorqueLogger = m_plot.addLogger("Outside torque");
	m_outTorqueLogger->setColor(qRgb(0x40, 0x40, 0x40));

	m_loadLogger = m_plot.addLogger("Load");

	m_currentLogger = m_plot.addLogger("Current");

	m_deviationLogger = m_plot.addLogger("Deviation");

// 	CompositeTrajectory* traj = new CompositeTrajectory();
	m_trajectory = new SinusTrajectory();
// 	Trajectory* sinus2 = new SinusTrajectory();
// 	traj->addTrajectory(sinus1);
// 	traj->addTrajectory(sinus2);

	m_commandGenerator = new TunableCommandGenerator(this);
	m_commandFitter = new CommandFitter(m_commandGenerator);
	m_commandFitter->setCommandLimits(
		m_jointInterface->minimumAngle(),
		m_jointInterface->maximumAngle()
	);

	addSlidersForObject(m_trajectory);
// 	addSlidersForObject(sinus2);
	addSlidersForObject(m_commandGenerator);

	connect(m_trajectory, SIGNAL(changed()), SLOT(initTrajectory()));
	connect(m_commandGenerator, SIGNAL(changed()), SLOT(initTrajectory()));

	loadSettings();

	ui.statistics->setPosLogger(m_angleLogger);
	ui.statistics->setCurrentLogger(m_currentLogger);
	ui.statistics->setTrajectory(m_trajectory);
	ui.statistics->update();

	connect(m_commandGenerator, SIGNAL(log(QString)), ui.log, SLOT(append(QString)));
	connect(ui.pSpinBox, SIGNAL(valueChanged(int)), m_commandGenerator, SLOT(setPValue(int)));
	connect(ui.pSpinBox, SIGNAL(valueChanged(int)), m_jointInterface, SLOT(setPValue(int)));

	initTrajectory();

	reset();

	m_writeCounter = 0;
}

void SingleServoTestbench::takeTrajectory(Trajectory* traj)
{
	if(m_trajectory)
		delete m_trajectory;
	m_trajectory = traj;
	ui.statistics->setTrajectory(traj);

	initTrajectory();
	trajectoryChanged();
}

void SingleServoTestbench::loadSettings()
{
	QSettings settings("de.uni-bonn", "servocontrol");
	settings.beginGroup(objectName());
	m_commandGenerator->deserialize(&settings);
	m_trajectory->deserialize(&settings);
	m_plot.deserialize(&settings);
	settings.endGroup();
}


void SingleServoTestbench::saveSettings()
{
	QSettings settings("de.uni-bonn", "servocontrol");
	settings.beginGroup(objectName());
	m_commandGenerator->serialize(&settings);
	m_trajectory->serialize(&settings);
	m_plot.serialize(&settings);
	settings.endGroup();
}

SingleServoTestbench::~SingleServoTestbench()
{
	delete m_jointInterface;
	delete m_commandFitter;
}

void SingleServoTestbench::addSlidersForObject(ParametrizedObject* obj)
{
	QGridLayout* layout = qobject_cast<QGridLayout*>(ui.trajConfigFrame->layout());
	int i = layout->rowCount();
	Q_FOREACH(DynamicParameter* p, obj->parameters())
	{
		QLabel* label = new QLabel(p->name, ui.trajConfigFrame);
		layout->addWidget(label, i, 0);

		QSlider* slider = new QSlider(Qt::Horizontal, ui.trajConfigFrame);
		slider->setMinimum(p->min * 10000);
		slider->setMaximum(p->max * 10000);
		slider->setSingleStep(p->step * 10000);
		slider->setValue(p->get() * 10000);
		layout->addWidget(slider, i, 1);
		connect(slider, SIGNAL(sliderMoved(int)), p, SLOT(setFromSlider(int)));
		connect(p, SIGNAL(valueChangedForSlider(int)), slider, SLOT(setValue(int)));

		QLabel* valueLabel = new QLabel(ui.trajConfigFrame);
		valueLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
		layout->addWidget(valueLabel, i, 2);
		connect(p, SIGNAL(valueChanged(QString)), valueLabel, SLOT(setText(QString)));

		p->notify();

		++i;
	}
}

double SingleServoTestbench::targetValue(double time)
{
	switch(m_jointInterface->controlType())
	{
		case IJointInterface::CT_POSITION:
			return m_trajectory->position(time);
		case IJointInterface::CT_VELOCITY:
			return m_trajectory->velocity(time);
		case IJointInterface::CT_TORQUE:
		{
			return m_trajectory->acceleration(time);
		}
		default:
			return 0;
	}
}

double SingleServoTestbench::presentValue()
{
	switch(m_jointInterface->controlType())
	{
		case IJointInterface::CT_POSITION:
			return m_jointInterface->currentPosition();
		case IJointInterface::CT_VELOCITY:
			return m_jointInterface->currentVelocity();
		case IJointInterface::CT_TORQUE:
			return m_jointInterface->currentAcceleration();
		default:
			return 0;
	}
}

void SingleServoTestbench::setModelTrajectory(const QVector< double >& cmd)
{
	m_modelCMDLogger->reset();
	for(int i = 0; i < cmd.size(); ++i)
	{
		double time = i * PROCESS_PERIOD;
		m_modelCMDLogger->log(time, cmd[i]);
	}
	update();
}

void SingleServoTestbench::transferModelCMD()
{
	m_cmdLogger->reset();
	m_discreteTrajectory.resize(m_trajectory->endTime()*PROCESS_FREQ + 1);
	int t = 0;
	while(t < m_trajectory->endTime() * PROCESS_FREQ)
	{
		double time = PROCESS_PERIOD * t;

		double cmd = m_modelCMDLogger->valueAtTime(time);

		m_discreteTrajectory[t] = cmd;
		m_cmdLogger->log(time, cmd);

		++t;
	}

	update();
}

void SingleServoTestbench::initTrajectory()
{
	int t = 0;
	m_modelCMDLogger->reset();
	m_origCMDLogger->reset();
	m_trajPosLogger->reset();
	m_trajVelLogger->reset();
	m_trajAccLogger->reset();

	while(t < m_trajectory->endTime() * PROCESS_FREQ)
	{
		double time = PROCESS_PERIOD * t;

		m_trajPosLogger->log(time, m_trajectory->position(time));
		m_trajVelLogger->log(time, m_trajectory->velocity(time));
		m_trajAccLogger->log(time, m_trajectory->acceleration(time));

		t += 1;
	}

	trajectoryChanged();

	update();
}

void SingleServoTestbench::updateTrajectory()
{
	write();

	m_cmdLogger->reset();
	for(int t = 0; t < m_trajectory->endTime() * PROCESS_FREQ; ++t)
	{
		double time = PROCESS_PERIOD * t;
		double dev = m_deviationLogger->valueAtTime(time + 8.0 * PROCESS_PERIOD);
		m_discreteTrajectory[t] += 0.8 * dev;
		m_cmdLogger->log(time, m_discreteTrajectory[t]);
	}
}

// Resets whatever needs to be reset when the user presses the reset button.
void SingleServoTestbench::reset()
{
	m_angleLogger->reset();
	m_currentLogger->reset();
	m_deviationLogger->reset();
	m_loadLogger->reset();
	m_velLogger->reset();
	m_accLogger->reset();
}

void SingleServoTestbench::doStep(double currentTime)
{
	if(currentTime / PROCESS_PERIOD >= m_discreteTrajectory.count())
		return;

	motionState.t = currentTime;
	motionState.x = m_discreteTrajectory[currentTime / PROCESS_PERIOD];
	motionState.v = m_trajectory->velocity(currentTime);
	motionState.a = m_trajectory->acceleration(currentTime);

	// Send the motion state as target to the servo here.
	m_jointInterface->setGoalPosition(motionState.x);
	m_jointInterface->setGoalSpeed(motionState.x);
	m_jointInterface->setGoalTorque(motionState.x);
	m_jointInterface->process();

	m_angleLogger->log(currentTime, m_jointInterface->currentPosition());
	m_velLogger->log(currentTime, m_jointInterface->currentVelocity());
	m_accLogger->log(currentTime, m_jointInterface->currentAcceleration());
	m_loadLogger->log(currentTime, m_jointInterface->currentLoad());
	m_currentLogger->log(currentTime, m_jointInterface->currentCurrent());

	double deviation = targetValue(currentTime) - presentValue();

	double dx = fabs(m_jointInterface->currentPosition() - m_trajectory->position(currentTime));
	double sigma = 10.0*M_PI/180.0;
	double devGain = exp(-(dx*dx)/(2.0*sigma*sigma));

	m_deviationLogger->log(currentTime, /*devGain * */deviation);

	update();
}

void SingleServoTestbench::mousePressEvent(QMouseEvent *event)
{
	mouseClick = event->posF();
	mouseClickTimeStamp = stopWatch.programTime();

	// If the mouseMoveEvent reports a keyframe closer than the mouse catch radius, then it's the start of a drag.
// 	if (nearestKeyframeId > -1)
// 		draggedKeyframeId = nearestKeyframeId;

	swipeFadeOutTimer.stop();
	swipeFadeOutVelocity *= 0;
	setCursor(Qt::ClosedHandCursor);
	dragMoved = false;
}

void SingleServoTestbench::mouseMoveEvent(QMouseEvent *event)
{
	updateMouse(event->posF());

	setCursor(Qt::ArrowCursor);

	// Detect drag motion and scale or translate accordingly.
	if (event->buttons() & (Qt::LeftButton | Qt::RightButton))
	{
		dragMoved = true;

		screenOffset -= mappedMouseDiff;
		updateScreenTransform();
	}

	update();
}

void SingleServoTestbench::mouseReleaseEvent(QMouseEvent *event)
{
	// Start swipe fade out if the screen was dragged.
	if (draggedKeyframeId == -1 && mouse != mouseClick)
	{
		updateMouse(event->posF());
		swipeFadeOutVelocity = mappedMouseVelocity;
		lastSwipeFadeOutTimeStamp = stopWatch.programTime();
		swipeFadeOutTimer.start();
	}

	dragMoved = false;
	draggedKeyframeId = -1;
	unsetCursor();
	update();
}

// Updates the mouse state (position and velocity).
void SingleServoTestbench::updateMouse(QPointF mousePos)
{
	mouse = mousePos;
	mousePresent = true;
	mappedMouse = screenTransform.inverted().map(mouse);
	mouseDiff = (mouse - lastMouse);
	mappedMouseDiff = mouseDiff*screenScale;
	mappedMouseDiff.ry() = -mappedMouseDiff.y();

	double timeDiff = (stopWatch.time()-lastMouseUpdateTimeStamp);
	if (timeDiff > 0.3)
	{
		mouseVelocity *= 0;
		mappedMouseVelocity = mouseVelocity*screenScale;
		mappedMouseVelocity.ry() = -mappedMouseVelocity.y();
		lastMouse = mouse;
		lastMouseUpdateTimeStamp = stopWatch.time();
	}
	else if (timeDiff >= 0.003)
	{
		QPointF measuredMouseVelocity = (mouse - lastMouse)/timeDiff;
		mouseVelocity = 0.5*mouseVelocity + 0.5*measuredMouseVelocity;
		mappedMouseVelocity = mouseVelocity*screenScale;
		mappedMouseVelocity.ry() = -mappedMouseVelocity.y();
		lastMouse = mouse;
		lastMouseUpdateTimeStamp = stopWatch.time();
	}

	//qDebug() << mouseDiff << mappedMouseDiff << lastMouse << timeDiff;
}

void SingleServoTestbench::mouseDoubleClickEvent(QMouseEvent *event)
{

	update();
}

void SingleServoTestbench::wheelEvent(QWheelEvent *event)
{
	if (event->delta() > 0)
	{
		screenScale /= 1.2;
	}
	else
	{
		screenScale *= 1.2;
	}

	updateScreenTransform();
	updateMouse(QPointF(event->pos()));
	update();
}

void SingleServoTestbench::leaveEvent(QEvent* event)
{
	mousePresent = false;
}

void SingleServoTestbench::resizeEvent(QResizeEvent* event)
{
	updateScreenTransform();
}

// Updates the transformation from screen to logical coordinates.
// This should be called each time when the scaling factor or the screen translation changes.
void SingleServoTestbench::updateScreenTransform()
{
	// The screen is first scaled and then translated by the screen offset, i.e. the screen
	// offset is scaled too, so that zooming with the mouse wheel only has to adjust the
	// scaling factor and automatically does the right thing.

	screenTransform = QTransform();
	screenTransform.translate(width()/2, height()/2);
	screenTransform.scale(1.0/screenScale, -1.0/screenScale);
	screenTransform.translate(-screenOffset.x(), -screenOffset.y());
	mappedMouse = screenTransform.inverted().map(mouse);
}


// Handles the swipe fade out (inertial movement after the mouse button was released).
void SingleServoTestbench::swipeFadeOut()
{
	if (swipeFadeOutVelocity.manhattanLength() < 0.8)
	{
		swipeFadeOutVelocity *= 0;
		swipeFadeOutTimer.stop();
	}
	else
	{
		double elapsedTime = (stopWatch.programTime() - lastSwipeFadeOutTimeStamp);
		screenOffset -= swipeFadeOutVelocity*elapsedTime;
		swipeFadeOutVelocity *= qMax(0.0, 1.0 - 4.0*elapsedTime);
		updateScreenTransform();
	}

	lastSwipeFadeOutTimeStamp = stopWatch.programTime();
	update();
}

void SingleServoTestbench::paintEvent(QPaintEvent*)
{
	Vec2f p1, p2;
	QPainter painter(this);

	painter.fillRect(rect(), Qt::white);

	// Prepare the basic painter colors and pen.
	QPen pen = QPen(QColor(0, 0, 0));
	pen.setCosmetic(true);
	painter.setPen(pen);
	painter.setFont(QFont("Arial", 8));
	QFontMetrics fm(painter.font());

	// Calculate the bounding box in the transformed coordinate system.
	QPointF topLeft = screenTransform.inverted().map(QPointF(0,0));
	QPointF bottomRight = screenTransform.inverted().map(QPointF(width(),height()));
	QRectF boundingBox = QRectF(topLeft, bottomRight);

	// Apply the coordinate transformation from device coordinates ((0,0) is in the top left corner)
	// to logical coordinates, where the origin of the coordinate system is in the middle and y grows
	// upwards. A scaling factor converts from pixel values to logical units (e.g. 100 px = 1 second).
	painter.setTransform(screenTransform);

	// Draw the axes.
	if (showAxes)
	{
		painter.drawLine(QPointF(boundingBox.left(), 0), QPointF(boundingBox.right(), 0));
		painter.drawLine(QPointF(0, boundingBox.bottom()), QPointF(0, boundingBox.top()));
		for (double i = floor(boundingBox.left()); i < boundingBox.right(); i=i+1.0)
			painter.drawLine(QPointF(i, 0), QPointF(i, -4.0*screenScale));
		for (double i = floor(boundingBox.bottom()); i < boundingBox.top(); i=i+1.0)
			painter.drawLine(QPointF(0, i), QPointF(4.0*screenScale, i));
	}

	painter.setRenderHint(QPainter::Antialiasing, true);

	// Draw the current motion state.
	pen.setWidth(1);
	pen.setColor(QColor("black"));
	painter.setPen(pen);
	painter.setBrush(QColor("yellow"));
	painter.setOpacity(0.5);
	painter.drawEllipse(QPointF(motionState.t, motionState.x), 7.0*screenScale, 7.0*screenScale);

	// Draw servo command bounding box
	double min = m_jointInterface->minimumAngle();
	double max = m_jointInterface->maximumAngle();
	painter.drawLine(QPointF(0, min), QPointF(1000.0, min));
	painter.drawLine(QPointF(0, max), QPointF(1000.0, max));

	// Draw log data
	painter.setOpacity(1);
	painter.setBrush(QBrush());
	m_plot.paint(&painter);
}

void SingleServoTestbench::learn(bool checked)
{
	if(checked)
	{
// 		play();
	}
}

void SingleServoTestbench::read()
{
	QFile file("out.traj");
	if(!file.open(QIODevice::ReadOnly))
	{
		QMessageBox::critical(this, "Error", "Could not read inpue file");
		return;
	}

	QTextStream in(&file);

	int i = 0;
	m_cmdLogger->reset();
	while(!in.atEnd())
	{
		double val;
		in >> val;
		in.skipWhiteSpace();

		m_discreteTrajectory[i] = val;
		m_cmdLogger->log(0.02 * i, val);
		i++;
		printf("%d\n", i);
	}
}

void SingleServoTestbench::write()
{
	QFile file(QString("out%1.traj").arg(m_writeCounter, 3, 10, QChar('0')));
	if(!file.open(QIODevice::WriteOnly))
	{
		QMessageBox::critical(this, "Error", "Could not write output file");
		return;
	}

	QTextStream out(&file);
	const QList<ValueLogger*>& loggers = m_plot.loggers();

	out << "Time ";
	Q_FOREACH(ValueLogger* logger, loggers)
		out << "'" << logger->name() << "' ";
	out << '\n';

	for(int i = 0; i < m_discreteTrajectory.count(); ++i)
	{
		out << 0.02 * i;
		Q_FOREACH(ValueLogger* logger, loggers)
			out << " " << logger->valueAtTime(PROCESS_PERIOD*i);
		out << '\n';
	}

	m_writeCounter++;
}

void SingleServoTestbench::setControlMode(int idx)
{
	m_jointInterface->setControlType((IJointInterface::ControlType)idx);
	trajectoryChanged();
}

void SingleServoTestbench::fitModel()
{
	m_commandFitter->fit(*m_cmdLogger, *m_outTorqueLogger, *m_trajectory);
}

void SingleServoTestbench::resetOutsideTorques()
{
	m_outTorqueLogger->reset();
}

void SingleServoTestbench::logOutsideTorque(double time)
{
	m_outTorqueLogger->log(time, m_torqueEstimator->torque(m_jointInterface->id()));
}

void SingleServoTestbench::updateStatistics()
{
	ui.statistics->setTemperature(m_jointInterface->currentTemperature());
	ui.statistics->update();
}

