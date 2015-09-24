#ifndef COMFILTER_H_
#define COMFILTER_H_

#include <cap_gait/contrib/KalmanFilter.h>
#include <cap_gait/contrib/LimpState.h>
#include <cap_gait/contrib/Limp.h>
#include <cap_gait/cap_gait_config.h>
#include <QGLViewer/vec.h>

namespace margait_contrib
{

class ComFilter
{
	KalmanFilter lateralKf;
	KalmanFilter sagittalKf;

public:
	LimpState limpState;

	double smoothingX;
	double smoothingY;

public:
	ComFilter();

	void reset(double x=0, double vx=0, double ax=0, double y=0, double vy=0, double ay=0);

	LimpState update(qglviewer::Vec z);

	void setTimeStep(double dt);
};

}

#endif // COMFILTER_H_
