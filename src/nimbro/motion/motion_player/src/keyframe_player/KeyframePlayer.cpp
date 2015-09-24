#include "keyframe_player/KeyframePlayer.h"
#include "keyframe_player/globals.h"
#include <QDebug>

// This is a quadratic spline based non-linear keyframe interpolator implemented
// as a keyframe player that allows to step through a continuous motion trajectory
// generated from a given set of keyframes. To use the keyframe player, instantiate
// it, set the maximum acceleration A (default 100 m/ss) and maximum velocity V
// (default 100 m/s) parameters. You can also set a VX parameter (default 0.5) that
// determines how the keyframe player behaves when unreachable keyframes are provided.
// Then provide a set of keyframes using the addKeyframe(t, x, v) method. Then, call
// the calculateCommands() method after all keyframes are provided. Then you can use
// the step(dt) method to step forward on the time line with a time increment dt, or
// the evaluateAt(t) method to query the keyframe player at any point t in time.
// Please have a closer look at the method documentation for further information.
//
//	Example:
//
//	// Initialize and add keyframes.
//	KeyframePlayer kp;
//	kp.setA(10.0);
//	kp.setV(10.0);
//	kp.addKeyframe(0, 1.0, 2.0);
//	kp.addKeyframe(3.0, 4.0, 5.0);
// 	kp.calculateCommands();
//
//	// Step through with incremental steps (e.g. 10 ms).
//	kp.reset();
//	while (!kp.atEnd())
//	{
//		Keyframe state = kp.step(0.01);
//		doSomethingWith(State);
//	}
//
//	// Evaluate at certain points in time.
//	kp.reset();
//	for (double t = 0; t < kp.totalTime(); t = t+0.01)
//	{
//		Keyframe state = kp.evaluateAt(t);
//		doSomethingWith(State);
//	}


namespace kf_player
{


// Instantiates a keyframe player with A=100, V=100 and VX=0.5.
KeyframePlayer::KeyframePlayer()
{
	V = 100.0;
	A = 100.0;
	VX = 0.5;
}

// Sets the acceleration limit to A. It also resets the keyframe player.
void KeyframePlayer::setA(double A)
{
	this->A = A;
	reset();
	calculateCommands();
}

// Sets the velocity limit to V. It also resets the keyframe player.
void KeyframePlayer::setV(double V)
{
	this->V = V;
	for (int i = 0; i < keyframes.size(); i++)
		keyframes[i].v = qBound(-V, keyframes[i].v, V);
	reset();
	calculateCommands();
}

// Sets the target error parameter VX. It also resets the keyframe player.
// When a keyframe is unreachable due to acceleration and velocity limitations,
// an error at the target point (t, x1, v1) is unavoidable. The VX parameter
// is a blending parameter that defines if the keyframe interpolation attempts
// to comply with the target position x1 (set VX to 0), or the target velocity
// v1 (set VX to 1). Intermediate values of VX will try to comply with both, the
// position and velocity and thus accept an error in both dimensions. Please note
// that while for a certain range of unreachable keyframes at least either the
// desired position, or the desired velocity can be fully reached, in especially
// hard cases none of the desired targets can be achieved. Interpret the VX
// parameter as a bias on the position or the velocity, which the keyframe player
// primarily attempts to reach.
void KeyframePlayer::setVX(double VX)
{
	this->VX = qBound(0.0, VX, 1.0);
	reset();
	calculateCommands();
}

// Clears the keyframe player. It deletes all keyframes.
void KeyframePlayer::clear()
{
	commands.clear();
	keyframes.clear();
}

// Resets the keyframe player to the first keyframe.
void KeyframePlayer::reset()
{
	if (not keyframes.isEmpty())
		currentState = keyframes[0];
}

// Adds a keyframe to the motion sequence.
// It returns false if you added an unreachable keyframe.
bool KeyframePlayer::addKeyframe(Keyframe kf)
{
	if (qAbs(kf.v) > V)
	{
		kf.v = sgn(kf.v) * V;
		kf.type = Keyframe::TYPE_UNREACHABLE;
	}

	if (keyframes.isEmpty())
	{
		keyframes << kf;
		return true;
	}

	int index = 0;
	while (index < keyframes.size() and kf.t >= keyframes[index].t)
		index++;

	if (index == keyframes.size() or keyframes[index] != kf)
		keyframes.insert(index, kf);

	return (kf.type != Keyframe::TYPE_UNREACHABLE);
}

// Tells you if the keyframe player has reached the end of the motion sequence.
// Querying the atEnd() state only makes sense if the step(dt) method is used to
// iterate through the motion sequence.
bool KeyframePlayer::atEnd()
{
	if (currentState.t >= keyframes.last().t)
		return true;
	return false;
}

// Returns the time of the last keyframe.
double KeyframePlayer::totalTime()
{
	if (keyframes.isEmpty())
		return 0;

	return keyframes.last().t;
}

// Returns the minimum time needed to complete the currently loaded keyframes when traveling at the acceleration and velocity limit.
double KeyframePlayer::minimumTime()
{
	if (keyframes.length() < 2)
		return 0;

	Keyframe kf;
	Keyframe kf0;
	Keyframe kf1;

	double tt = 0;
	for (int i = 0; i < keyframes.length()-1; i++)
	{
		kf0 = keyframes[i];
		kf1 = keyframes[i+1];
		kf = kf0;

		// For nice notation.
		double x0 = kf0.x;
		double v0 = kf0.v;
		double x1 = kf1.x;
		double v1 = kf1.v;
		double dx = fabs(x1-x0);
		v0 = sgn(x1-x0)*v0;
		v1 = sgn(x1-x0)*v1;

		double epsilon = 0.001; // to make the sign decision a little more stable.
		int sign = (sqrt(v0*v0 + 2.0*A*dx)+epsilon < v1 or sqrt(v1*v1 + 2.0*A*dx)+epsilon < v0) ? -1 : 1;
		tt += (sqrt(2.0*(v0*v0 + v1*v1) + sign*4.0*A*dx) - sign*(v0+v1)) / A;


		// What velocity will we have at the t* point?
		double xstar = 0.5*dx + 0.25*(v1*v1 - v0*v0)/A;
		double tstar = (sqrt(v0*v0+v1*v1 + sign*A*dx) - sign*v0) / A;
		double vstar = v0 + tstar*sign*A;
		qDebug() << "stars:" << xstar << tstar << vstar;

//		if (fabs(vstar) > V)
//			qDebug() << "Velocity limit reached";
	}

	return tt;
}


// Steps the internal simulation forward by the time increment dt and returns the resulting
// state. Use reset() to reset the internal simulation to the beginning of the motion sequence
// and atEnd() to check if the end has been reached. Stepping beyond the end of the motion
// sequence will have no effect on the returned state. Stepping through the motion sequence
// takes the inertia of a simulated particle into account and will always produce a smooth
// trajectory, even if the keyframes are not reachable. The step(), atEnd(), reset()
// combination is the recommended interface to drive a keyframe interpolated motion sequence.
Keyframe KeyframePlayer::step(double dt)
{
	currentState = evaluateAt(currentState.t + dt);
	return currentState;
}

// Returns the current state. The current state only changes when the step() method is used
// to drive the keyframe player.
Keyframe KeyframePlayer::getCurrentState()
{
	return currentState;
}


// Evaluates the keyframe interpolation at a given time t and returns the expected state.
Keyframe KeyframePlayer::evaluateAt(double t)
{
	if (keyframes.isEmpty())
		return Keyframe();

	if (commands.isEmpty())
		return keyframes.first();

	if (t <= keyframes.first().t)
	{
		Keyframe state = keyframes.first();
		state.a = commands[0].a;
		state.t = t;
		return state;
	}

	if (t >= commands.last().t)
		return commands.last();

	int effortIndex = 0;
	while(effortIndex < keyframes.size() - 1 && t >= keyframes[effortIndex].t)
		effortIndex++;

	int index = 0;
	while (t >= commands[index].t)
		index++;

	Keyframe state = index == 0 ? keyframes[0] : commands[index-1];
	double dt = t - state.t;
	state.x += 0.5*commands[index].a*dt*dt + state.v*dt;
	state.v += commands[index].a*dt;
 	state.a = commands[index].a;
	state.type = commands[index].type;
	state.t = t;
	state.effort = keyframes[effortIndex].effort;
	state.suppLeftLeg = keyframes[effortIndex].suppLeftLeg;
	state.suppRightLeg = keyframes[effortIndex].suppRightLeg;

	return state;
}

void KeyframePlayer::transformState(double a, double t, Keyframe& kf)
{
	kf.a = a;
	kf.t += t;
	kf.x += 0.5*kf.a*t*t + kf.v*t;
	kf.v += kf.a*t;
	kf.type = (fabs(a) >= A) ? Keyframe::TYPE_AMAX : (kf.a == 0 and fabs(kf.v) >= V) ? Keyframe::TYPE_VMAX : Keyframe::TYPE_DEFAULT;
}

// The gory details of the keyframe interpolation. From the given keyframes, a set of commands
// is calculated that encode the velocity and acceleration limited quadratic spline.
void KeyframePlayer::calculateCommands()
{
	commands.clear();

	if (keyframes.length() < 2)
		return;

	Keyframe kf;
	Keyframe kf0;
	Keyframe kf1;

	for (int i = 0; i < keyframes.length()-1; i++)
	{
		kf0 = (i == 0) ? keyframes.first() : commands.last();
		kf1 = keyframes[i+1];
		kf = kf0;
		kf.effort = kf1.effort;
		kf.suppLeftLeg = kf1.suppLeftLeg;
		kf.suppRightLeg = kf1.suppRightLeg;

		// For nice notation.
		double x0 = kf0.x;
		double v0 = kf0.v;
		double x1 = kf1.x;
		double v1 = kf1.v;
		double t = kf1.t - kf0.t;

		// Handle the special cases explicitly, where no control is required (or a is bounded to 0).
		// Just coast to the next keyframe.
		if (A == 0 or (fabs(v0 - v1) < EPSILON and (fabs((x1-x0) - v0*t) < EPSILON)))
		{
			transformState(0, kf1.t-kf.t, kf);
			commands << kf;
			continue;
		}

		// Determine an initial unbounded acceleration and the sign of the approach (accelerate or brake).
		double sqrta = sqrt(2*(2*x1*x1+(-4*x0-2*t*v1-2*t*v0)*x1+2*x0*x0+(2*t*v1+2*t*v0)*x0+t*t*v1*v1+t*t*v0*v0));
		double a1 = (-sqrta+2*x1-2*x0-t*v1-t*v0)/(t*t); // "oben rum"
		double a2 =  (sqrta+2*x1-2*x0-t*v1-t*v0)/(t*t); // "unten rum"
		double a = fabs(a1) > fabs(a2) ? a1 : a2;
		int sign = fabs(a1) > fabs(a2) ? -1 : 1;

		// Apply the acceleration limit.
		a = qBound(-A, a, A);

		// Determine tstar.
		// The "tstar" time is the point where the acceleration flips sign.
		// We can calculate the tstar point in two ways: using only the desired velocity v1
		// or using only the desired position x1. For reachable keyframes the tstar point is
		// equal for both ways. For unreachable keyframes, we have to decide if
		// we are willing to accept an error in velocity or in position. VX is a blending
		// parameter [0,1] that determines if the position is achieved (VX=0) or the
		// velocity v1 is achieved (VX=1) or something in between.
		double tstarv = qBound(0.0, 0.5*(a*t + v1 - v0)/a, t);
		double tstarx = qMax(0.0, qMin(t - sign*sqrt(a*(x0-x1)+a*t*v0+0.5*a*a*t*t)/a, t));
		double tstar = VX*tstarv + (1.0-VX)*tstarx;

		// Now given a and tstar, are we going to hit the velocity limit?
		double vstar = v0 + a*tstar;
		if (fabs(vstar) < V)
		{
			// No velocity limit reached. Calculate the commands.

			if (tstar > 0) // The first command is not needed when tstar = 0.
			{
				transformState(a, tstar, kf);
				commands << kf;
			}

			if (tstar < t) // The second command is not needed when tstar = t.
			{
				transformState(-a, (kf1.t-kf.t), kf);
				commands << kf;
			}
		}
		else
		{
			// The velocity limit is reached on the way.

			// Calculate a new a that takes a time span at the velocity limit into account.
			// We map the (x0, v0, x1, v0) input to a (dx, v0, v1) situation where dx is
			// the positive distance between x0 and x1 and the signs of v0 and v1 are flipped
			// as needed.
			double dx = fabs(x1 - x0);
			double dxsign = sgn(x1 - x0);
			v0 = dxsign*v0;
			v1 = dxsign*v1;
			a = (t*V-dx) > 0 ? (V*V-V*(v0+v1)+0.5*(v1*v1+v0*v0))/fabs(t*V-dx) : A;

			// Apply the acceleration limit.
			a = qBound(-A, a, A);

			// To implement a smooth blending with the VX parameter, we need to determine a
			// VXstar where vstar equals V. For all VX < VXstar the velocity limit will be
			// reached and this velocity limit case is executed. Now the VX range < VXstar
			// is mapped to a [0, 1] range such that mappedVX = 0 when VX = 0 and
			// mappedVX = 1 when VX = VXstar.
			double VXstar = qMin(-(V-v0-fabs(a)*tstarx)/(fabs(a)*(tstarx-tstarv)), 1.0);
			double mappedVX = VX/VXstar;

			// Determine a new tstar.
			double t1 = qBound(0.0, (V-v0)/a, t);
			double t2v = qBound(0.0, (V-v1)/a, t-t1);
			double t2x = qBound(0.0, sqrt(2*(v0*V+fabs(a)*(t*V-dx))-v0*v0-V*V)/fabs(a), t-t1);
			double t2 = mappedVX*t2v + (1.0-mappedVX)*t2x;
//			double t2 = VX*t2v + (1.0-VX)*t2x;

			// Generate commands.
			a = dxsign*a;
			transformState(a, t1, kf);
			commands << kf;

			transformState(0, (t-t1-t2), kf);
			commands << kf;

			transformState(-a, t2, kf);
			commands << kf;
		}

//		qDebug() << "x0:" << x0 << "v0:" << v0 << "x1:" << x1 << "v1:" << v1 << "t:" << t << "a:" << a << "tstar:" << tstar << "sign:" << sign;

		// Mark the next keyframe with the appropriate reachability type.
		keyframes[i+1].type = (fabs(a) >= A || fabs(keyframes[i+1].v) >= V) ? Keyframe::TYPE_UNREACHABLE : Keyframe::TYPE_DEFAULT;
	}
}

}