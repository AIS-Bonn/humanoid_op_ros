// Walk and kick gaze behaviour state: Look around
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/beh_states/gaze_beh_look_around.h>
#include <rc_utils/math_vec_mat.h>
#include <rc_utils/math_funcs.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// GazeBehLookAround class
//

// Constructor
GazeBehLookAround::GazeBehLookAround(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID) : WAKBehState(config, SV, WBS, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void GazeBehLookAround::handleActivation(bool nowActive)
{
	// Reset variables
	resetGazeSpline();
}

// Reset function for the gaze spline variables
void GazeBehLookAround::resetGazeSpline()
{
	// Reset the gaze spline variables
	m_gazeInit << 0.0f, config.gazePitchNeutral();
	m_gazeMag = config.laGazeMagInitial();
	m_gazeTargetID = 0;
	m_gazeTargetDirn = 1;
	m_gazeSpline.reset();
}

// Execute function
void GazeBehLookAround::execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated)
{
	//
	// Gaze control
	//

	// Desired behaviour:
	// The look around process starts at the current last commanded gaze orientation and moves out in a
	// piecewise linear triangular spiral. The look starts at the corner of the triangle that is closest
	// (with some weighting), which was deemed to most often be the correct action in game situations.

	// Set the initial state of the gaze spline
	if(justActivated)
	{
		m_gazeInit << lastAV.gazeYaw, lastAV.gazePitch;
		m_gazeSpline.setState(m_gazeInit.x(), m_gazeInit.y());
	}

	// Calculate the gaze target options
	Vec2f gazeTargetDown(0.0f, config.laGazePitchLookDown());
	Vec2f gazeTargetPos(config.gazeYawAbsMax(), config.laGazePitchLookUp());
	Vec2f gazeTargetNeg(-config.gazeYawAbsMax(), config.laGazePitchLookUp());

	// Calculate the distances to the gaze target options
	Vec2f gazeDownDir = gazeTargetDown - m_gazeInit;
	Vec2f gazePosDir = gazeTargetPos - m_gazeInit;
	Vec2f gazeNegDir = gazeTargetNeg - m_gazeInit;
	float gazeDownDist = gazeDownDir.norm();
	float gazePosDist = gazePosDir.norm();
	float gazeNegDist = gazeNegDir.norm();

	// Initialise the gaze search pattern
	if(justActivated)
	{
		// Calculate the costs of starting by looking at a particular gaze target option
		float u = coerce(config.laGazeDownFirstFactor(), 0.0f, 1.0f);
		float gazeDownCost = (1.0f - u)*gazeDownDist;
		float gazePosCost = u*gazePosDist;
		float gazeNegCost = u*gazeNegDist;

		// Select an initial gaze target ID and search direction
		if(gazeDownCost <= gazePosCost && gazeDownCost <= gazeNegCost)
		{
			m_gazeTargetID = 0;
			m_gazeTargetDirn = sign(m_gazeInit.x());
		}
		else
		{
			m_gazeTargetID = (gazePosCost <= gazeNegCost ? +1 : -1);
			m_gazeTargetDirn = -m_gazeTargetID;
		}
	}

	// Recalculate the gaze spline if required
	if(m_gazeSpline.finished())
	{
		// Update the gaze magnitude
		if(!justActivated)
			m_gazeMag += config.laGazeMagInc();
		m_gazeMag = coerce<float>(m_gazeMag, 0.0f, M_2PI); // Note: Just to avoid the gaze magnitude from growing to infinity

		// Increment the gaze target ID
		if(!justActivated)
			m_gazeTargetID += m_gazeTargetDirn;
		if(m_gazeTargetID < -1) m_gazeTargetID = 1;
		if(m_gazeTargetID > 1) m_gazeTargetID = -1;

		// Retrieve the direction and distance to the desired gaze target
		Vec2f gazeDir;
		float gazeDist;
		if(m_gazeTargetID < 0)
		{
			gazeDir  = gazeNegDir;
			gazeDist = gazeNegDist;
		}
		else if(m_gazeTargetID > 0)
		{
			gazeDir  = gazePosDir;
			gazeDist = gazePosDist;
		}
		else
		{
			gazeDir  = gazeDownDir;
			gazeDist = gazeDownDist;
		}

		// Calculate an intermediate gaze spline target using the current gaze magnitude
		float mag = coerce(m_gazeMag, 0.0f, gazeDist);
		Vec2f splineTarget = m_gazeInit + mag*eigenNormalized(gazeDir);
		if(m_gazeTargetID == 0)
			splineTarget = gazeTargetDown; // Note: An override to make sure that when we want to look down, we really look down

		// Set the new target for the gaze spline
		m_gazeSpline.newTarget(splineTarget.x(), splineTarget.y(), config.laGazeSplineVelMax(), config.laGazeSplineAccMax());
	}

	// Evaluate the gaze spline
	m_gazeSpline.forward(TINC);
	AV.gazeYaw = m_gazeSpline.curX();
	AV.gazePitch = m_gazeSpline.curY();

	// Plotting
	if(config.plotData())
	{
		PM.plotScalar(m_gazeMag, PM_LA_GAZEMAG);
		PM.plotScalar(m_gazeTargetID, PM_LA_GAZETARGETID);
		PM.plotScalar(m_gazeSpline.targetX(), PM_LA_GAZETARGETX);
		PM.plotScalar(m_gazeSpline.targetY(), PM_LA_GAZETARGETY);
	}
}
// EOF