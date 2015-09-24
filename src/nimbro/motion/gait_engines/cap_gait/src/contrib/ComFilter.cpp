#include <cap_gait/contrib/ComFilter.h>

using namespace margait_contrib;

ComFilter::ComFilter()
{
	smoothingX = 0.1;
	smoothingY = 0.1;
	sagittalKf.setSmoothing(smoothingX);
	lateralKf.setSmoothing(smoothingY);
}

// Sets the filter into the given state and doesn't touch the kalman gains.
void ComFilter::reset(double x, double vx, double ax, double y, double vy, double ay)
{
	sagittalKf.reset(x, vx, ax);
	lateralKf.reset(y, vy, ay);

	limpState.x = sagittalKf.x;
	limpState.vx = sagittalKf.v;
	limpState.ax = sagittalKf.a;
	limpState.y = lateralKf.x;
	limpState.vy = lateralKf.v;
	limpState.ay = lateralKf.a;
}

// Support vector in. LimpState out.
LimpState ComFilter::update(qglviewer::Vec z)
{
	sagittalKf.setSmoothing(smoothingX);
	lateralKf.setSmoothing(smoothingY);
	
	sagittalKf.update(z.x);
	lateralKf.update(z.y);

	limpState.x = sagittalKf.x;
	limpState.vx = sagittalKf.v;
	limpState.ax = sagittalKf.a;
	limpState.y = lateralKf.x;
	limpState.vy = lateralKf.v;
	limpState.ay = lateralKf.a;

	return limpState;
}

// Set the time step of the internal kalman filters
void ComFilter::setTimeStep(double dt)
{
	if(dt > 0)
	{
		lateralKf.setTimeStep(dt);
		sagittalKf.setTimeStep(dt);
	}
}
