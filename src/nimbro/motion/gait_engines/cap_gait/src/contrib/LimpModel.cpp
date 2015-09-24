#include <cap_gait/contrib/LimpModel.h>
// #include "Framework/State.h"
// #include "Framework/Config.h"

using namespace margait_contrib;

LimpModel::LimpModel(const cap_gait::CapConfig* capConfig) : config(capConfig)
{
	systemIterationTime = 0;
	fusedAngleX = 0;
	fusedAngleY = 0;
	
	x = 0;
	vx = 0;
	ax = 0;
	y = 0;
	vy = 0;
	ay = 0;
	energyX = 0;
	energyY = 0;
	supportLegSign = 1;

	timeSinceStep = 0;
	timeToStep = 0;
	nominalTimeToStep = 0;
	crossing = false;
	stalling = false;
	nominalFootStepTHalf = 0;
}

// Sets the state of the limp model.
// It sets the current motion state, it computes the nominal state, the Z,T,F parameters, and the end of step state.
void LimpModel::setState(LimpState ms, Vec3f ggcv)
{
	double C = config->mgC();
	double sC = sqrt(C);
	double alpha = config->mgAlpha(); // Lateral step apex size.
	double delta = config->mgDelta(); // Minimum lateral support exchange location.
	double omega = config->mgOmega(); // Maximum lateral support exchange location.
	double sigma = config->mgSigma(); // Maximum sagittal apex velocity.
	double gamma = config->mgGamma(); // Maximum rotational step angle.

	gcv = ggcv; // Input

	// Set the motion state.
	x = ms.x;
	vx = ms.vx;
	ax = ms.ax;
	y = ms.y;
	vy = ms.vy;
	ay = ms.ay;
	supportLegSign = ms.supportLegSign;
	energyX = Limp::energy(x, vx, C);
	energyY = Limp::energy(y, vy, C);

	rxCapturePoint.x = vx/sC;
	rxCapturePoint.y = vy/sC;
	rxZmp.x = -ax/C;
	rxZmp.y = -ay/C;

	// Compute the nominal state.
	double nominalSex = delta; // The optimal abbreviation for "nominal support exchange location"!
	if (supportLegSign * gcv.y > 0)
		nominalSex = delta + qAbs(gcv.y)*(omega - delta); // long step case between delta and omega
	else
		nominalSex = delta;// - qAbs(gcv.y)*(delta - alpha); // short step case between alpha and delta
	limp.set(alpha, 0, C);
	nominalFootStepTHalf = limp.tLoc(nominalSex); // The time measured from alpha to sex...
	//nominalFootStepTHalf = limp.tLoc(delta); // The time measured from alpha to delta. Tau (6) in the RoboCup Symposium 2014 paper.

	nominalState.supportLegSign = supportLegSign;
	nominalState.y = supportLegSign * nominalSex;
	nominalState.vy = supportLegSign * sC * sqrt(nominalSex*nominalSex - alpha*alpha);

	limp.set(0, gcv.x*sigma, C);
	limp.update(nominalFootStepTHalf); // Tau is used to calculate the sagittal sex.
	nominalState.x = limp.x0;
	nominalState.vx = limp.v0;
	nominalState.energyX = limp.energy();

	nominalCapturePoint.x = nominalState.x + nominalState.vx/sC;
	nominalCapturePoint.y = nominalState.y + nominalState.vy/sC;

	nominalFootStep.x = nominalState.x; // Using symmetry here.
	limp.set(nominalState.y, nominalState.vy, C);
	nominalFootStep.y = -limp.origin(-supportLegSign*alpha, 0);
	nominalFootStep.z = gcv.z * gamma;
	nominalStepSize.x = nominalState.x + nominalFootStep.x;
	nominalStepSize.y = nominalState.y + nominalFootStep.y;
	nominalStepSize.z = nominalFootStep.z;


	// 1. Lateral ZMP.
	// The lateral ZMP offset is calculated such that the CoM should reach the nominal support exchange location
	// at the nominal time, which is computed from the fixed configured step frequency. But the zmp offset is
	// bounded to be inside the support polygon, so the nominal time cannot always be met and the step timing has
	// to be adjusted. The lateral ZMP cannot be estimated well towards the end of the step when the time to step
	// approaches zero, so it must be inhibited near the support exchange.
	limp.set(y, vy, C);
	double zy = limp.z(nominalState.y, nominalTimeToStep);
	if (supportLegSign == 1)
		zmp.y = qBound((double) config->mgZmpYMin(), zy, (double) config->mgZmpYMax());
	else
		zmp.y = qBound((double) -config->mgZmpYMax(), zy, (double) -config->mgZmpYMin());
	if(!config->cmdUseNonZeroZMP())
		zmp.y = 0;

	// The energy must be computed after the zmp.
	energyY = Limp::energy(y - zmp.y, vy, C); // with zmp!


	// Crossing means traveling towards the support leg with energy > 0 in lateral direction.
	// When crossing, torso balancing could be of great help.
	crossing = (supportLegSign*vy < 0 and energyY > 0) or
	           (supportLegSign*vy < 0 and supportLegSign*y < 0) or
	           (supportLegSign*y < 0 and energyY < 0);

	// Stalling is when the limp never crosses the sex, i.e. is "inside" the sex.
	stalling = (energyY < Limp::energy(nominalState.y - zmp.y, 0, C));


	// 2. Step timing.
	// Compute the time to step.
	// The best time T to step is when the lateral pendulum is going to reach the desired support exchange location (sex).
	// In the best case we are on a good kind of returning trajectory and the sex will be reached eventually.
	double tts = timeToSel();
	double tta = timeToApex();

	// On a crossing trajectory, the step time is infinity. We just wait and hope that the robot will return after all.
	// In this case we also return and don't modify the previously computed zmp and footstep coordinates. Nice hack dude.
	if (crossing)
	{
		timeToStep = 2.0;
		return;
	}

	// If the step time could be clearly determined, then there you go.
	else if (tts > 0)
		timeToStep = tts;

	// In stalling cases (the bad kind of returning trajectory), we accept a positive time to apex as step time approximation.
	// Better than nothing.
	else if (stalling and tta > 0)
		timeToStep = tta;

	// One problem with this is that if a push puts us on a stall trajectory after the apex, the step time would
	// (and should) be zero. In theory, we should step as quickly as we possibly can! In practice I found that this
	// can produce a nervous stepping behavior. It worked better to just decay the timeToStep and take an unhurried step
	// and then let the balance controller fix the problem on the other leg.

	// In other cases the step time cannot be properly determined and we just decay. The floor is always coming closer.
	else
		timeToStep -= systemIterationTime;


	// On the other hand, if the step time we computed based only on lateral considerations results in an extreme sagittal
	// position at the end of the step, we overwrite the step time with the time of reaching the sagittal boundary.
	limp.set(x, vx, C);
	double t1 = limp.tLoc(config->mgMaxComPositionX());
	double t2 = limp.tLoc(-config->mgMaxComPositionX());
	if (t2 > 0 and t2 < timeToStep)
		timeToStep = t2;
	if (t1 > 0 and t1 < timeToStep)
		timeToStep = t1;
	if (x > config->mgMaxComPositionX() or x < -config->mgMaxComPositionX())
		timeToStep = 0;


	// 3. Sagittal ZMP.
	// The gait control input defines a nominal capture point along with the nominal state. The sagittal zmp is calculated
	// using Englsberger's capture point based ZMP controller from the paper "Bipedal Walking Control Based on Capture Point Dynamics".
	// The controller computes a ZMP based on the capture point of the current CoM state and the desired capture point.
	// The computed ZMP will theoretically move the CoM such that its capture point matches the desired capture point
	// at a given time T, which is the end of the footstep in this case.
//	Vec2f capturePoint = getCapturePoint();
//	double b = exp(sqrt(C)*timeToStep);
//	zmp.x = qBound(config.zmpXMin, (nominalCapturePoint.x - b*capturePoint.x)/(1 - b), config.zmpXMax);

	// New zmp method. It tries to match the CoM location with the nominal state at the end of the step, instead of the capture point.
	limp.set(x, vx, C);
	double zx = limp.z(nominalState.x, timeToStep);
	zmp.x = qBound((double) config->mgZmpXMin(), zx, (double) config->mgZmpXMax());

	if(!config->cmdUseNonZeroZMP())
		zmp.x = 0;

	energyX = Limp::energy(x - zmp.x, vx, C); // with zmp!

	// The reachable end-of-step state prediction. c' in the RoboCup Symposium paper.
	// We forward the model by "timeToStep" time to the expected end of step state c' using the zmp that we just calculated.
	endOfStepState = forwarded(timeToStep).getMotionState();


	// 4. Lateral step size Y.
	// The lateral step size is the next pendulum pivot point relative to the future CoM location at support exchange.
	// The step size is chosen such that the CoM will pass the next apex at distance alpha.
	Limp limp(endOfStepState.y, endOfStepState.vy, C);
	footStep.y = limp.origin(supportLegSign*alpha, 0); // This works nicely on the real robot as well.
	//footStep.y = endOfStepState.y; // This didn't work very well.


	// 5. Sagittal step size X.
	// The sagittal step location is expressed relative to the CoM location and is calculated so. The combination of
	// the current CoM state and the current zmp will result in a CoM velocity at the end of the step. Now if that
	// velocity would have been the result of a step with nominal step time, which (nominal) step size would we have
	// used to reach that velocity? That's going to be our step size. To reproduce the equations: forward the current
	// CoM state by the step time t, calculate the velocity back at the apex using the tanh formula, forward the apex
	// state by the half step time tau and there is your step size.
	//footStep.x = endOfStepState.x; // Simply step size = 2 * com. Matches the end of step position. This means all steps are symmetrical and the CoM is in the middle. Worked best on the real robot, doesn't work with the limp model.
	footStep.x = endOfStepState.vx * tanh(sC*nominalFootStepTHalf) / sC; // Humanoids 2013 version. Matches the end of step velocities. Works with the limp model and the robot.
	// Match the end of step capture point?
	// Match the end of step energy?


	// 6. Rotational step size.
	footStep.z = nominalFootStep.z;

	stepSize.x = endOfStepState.x + footStep.x;
	stepSize.y = endOfStepState.y + footStep.y;
	stepSize.z = footStep.z;
}

// Forwards the pendulum model so that it triggers a step when the sel is reached.
bool LimpModel::forwardThroughStep(double dt)
{
	bool stepped = false;
	if (dt >= timeToStep)
	{
		double tts = qMax(timeToStep, 0.0);
		forward(tts);
		step();
		stepped = true;
		forward(dt-tts);
	}
	else
	{
		forward(dt);
	}

	return stepped;
}

// Simulates the pendulum model for dt amount of time. No step is induced.
void LimpModel::forward(double dt)
{
	double C = config->mgC();

	limp.set(x - zmp.x, vx, C); // update with zmp
	limp.update(dt);
	x = limp.x0 + zmp.x; // new location without zmp!
	vx = limp.v0;
	energyX = Limp::energy(x, vx, C); // energy without zmp!

	limp.set(y - zmp.y, vy, C);
	limp.update(dt);
	y = limp.x0 + zmp.y; // new location without zmp!
	vy = limp.v0;
	energyY = Limp::energy(y, vy, C); // energy without zmp!

	timeToStep -= dt;
	nominalTimeToStep -= dt;
	timeSinceStep += dt;

	ax = C*(x - zmp.x); // acceleration with zmp
	ay = C*(y - zmp.y);
//	rxZmp.x = -ax/C;
//	rxZmp.y = -ay/C;
//	rxCapturePoint.x = vx/sqrt(C);
//	rxCapturePoint.y = vy/sqrt(C);
}

// Returns a copy at time t relative to the current state. No step is induced.
LimpModel LimpModel::forwarded(double t)
{
	LimpModel lm = *this;
	lm.forward(t);
	return lm;
}

// Induces a step to the current footStep location.
void LimpModel::step()
{
	double C = config->mgC();

	// Post step prediction correction.
	// The larger the torso angle at support exchange, the less precise the limp step prediction becomes.
	// We try to fix it a little with this hack.
	double postStepCorrectionFactor = config->cmdUseRXFeedback()*qBound(0.0, qAbs(fusedAngleY)/config->mgPostStepStateCorrAng(), 1.0);
	if (not config->mgPostStepStateCorrAng() > 0)
		postStepCorrectionFactor = 0;

	LimpState ls = getMotionState();
	ls.x = postStepCorrectionFactor*ls.x + (1.0-postStepCorrectionFactor)*-footStep.x; // Danger! This explodes without adaptation.
	ls.y = -footStep.y;
	ls.ax = C*ls.x;
	ls.ay = C*ls.y;
	ls.supportLegSign = -supportLegSign;
	setState(ls, gcv);
	timeSinceStep = 0;
	nominalTimeToStep = 2.0*nominalFootStepTHalf;

	// Make a wild guess if the timeToStep could not be estimated.
	if(timeToStep <= 0)
		timeToStep = nominalTimeToStep;
}

// Returns the current motion state.
LimpState LimpModel::getMotionState()
{
	double C = config->mgC();

	LimpState ms;
	ms.x = x;
	ms.vx = vx;
	ms.ax = ax;
	ms.y = y;
	ms.vy = vy;
	ms.ay = ay;
	ms.supportLegSign = supportLegSign;
	ms.energyX = Limp::energy(x, vx, C);
	ms.energyY = Limp::energy(y, vy, C);
	return ms;
}


// Returns the estimated time to the support exchange location.
// Takes the currently set zmp into account. The sel is given in pendulum coordinates relative to the foot center.
double LimpModel::timeToLoc(double sel)
{
	double C = config->mgC();

	limp.set(y - zmp.y, vy, C);
	return limp.tLoc(sel - zmp.y);
}

// Returns the estimated time to the support exchange location.
double LimpModel::timeToSel()
{
	double C = config->mgC();

	limp.set(y - zmp.y, vy, C);
	return limp.tLoc(nominalState.y - zmp.y);
}

// Returns the estimated time to the apex.
// Takes the currently set zmp into account.
double LimpModel::timeToApex()
{
	double C = config->mgC();

	limp.set(y - zmp.y, vy, C);
	return limp.tVel(0);
}

// Returns the state equivalent that was mirrored to the right foot.
// If the state is already on the right foot, nothing is changed.
LimpState LimpModel::mirroredToRight()
{
	LimpState rightState = getMotionState();
	rightState.y = supportLegSign*y;
	rightState.vy = supportLegSign*vy;
	rightState.supportLegSign = 1;
	return rightState;
}

// Returns the nominal state.
LimpState LimpModel::getNominalState()
{
	return nominalState;
}

// Returns the estimated end of step state.
LimpState LimpModel::getEndOfStepState()
{
	return endOfStepState;
}

// Returns the capture point in pendulum coordinates (relative to support foot middle).
Vec2f LimpModel::getCapturePoint()
{
	double C = config->mgC();

	Vec2f capturePoint;
	capturePoint.x = x + Limp(x, vx, C).origin(0, 0);
	capturePoint.y = y + Limp(y, vy, C).origin(0, 0);
	return capturePoint;
}
