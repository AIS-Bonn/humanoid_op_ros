#ifndef LIMPMODEL_H_
#define LIMPMODEL_H_

#include <cap_gait/contrib/Limp.h>
#include <cap_gait/contrib/LimpState.h>
#include <cap_gait/contrib/Vec2f.h>
#include <cap_gait/contrib/Vec3f.h>
#include <cap_gait/cap_gait_config.h>

namespace margait_contrib
{

struct LimpModel
{
	const cap_gait::CapConfig* config;
	double systemIterationTime;
	double fusedAngleX;
	double fusedAngleY;

// 	char name;
	Vec3f gcv; // input

	// current state
	double x;
	double vx;
	double ax;
	double y;
	double vy;
	double ay;
	double energyX;
	double energyY;
	int supportLegSign;
	bool crossing;
	bool stalling;

	LimpState nominalState;
	Vec2f nominalCapturePoint;
	Vec3f nominalFootStep;
	Vec3f nominalStepSize;
	double nominalFootStepTHalf;

	double timeSinceStep;
	double timeToStep;
	double nominalTimeToStep;

	Vec2f zmp;
	LimpState endOfStepState;
	Vec3f footStep;
	Vec3f stepSize;

	Vec2f rxZmp;
	Vec2f rxCapturePoint;

private:
	Limp limp;

public:
	explicit LimpModel(const cap_gait::CapConfig* capConfig);
	~LimpModel(){};

	void reset()
	{
		LimpState ms;
		ms.reset(); // Just to be safe
		setState(ms, Vec3f(0.0, 0.0, 0.0)); // TODO: This is probably not such a good idea as a zero LimpState is actually highly atypical and a situation that is in unstable equilibrium, leading to possibly weird timeToStep's and so on.
		updateInputData(systemIterationTime, 0.0, 0.0);
	}

	inline void updateInputData(double systemIterationTime, double fusedAngleX, double fusedAngleY)
	{
		this->systemIterationTime = systemIterationTime;
		this->fusedAngleX = fusedAngleX;
		this->fusedAngleY = fusedAngleY;
	}

	void setState(LimpState ms, Vec3f ggcv);
	bool forwardThroughStep(double dt);
	void forward(double dt);
	LimpModel forwarded(double t);
	void step(); // Make a footstep in the model

	double timeToLoc(double loc);
	double timeToSel();
	double timeToApex();

	LimpState mirroredToRight();
	LimpState getMotionState();
	LimpState getNominalState();
	LimpState getEndOfStepState();
	Vec2f getCapturePoint();

	inline bool operator==(const LimpModel& v) const
	{
		return (x==v.x) && (vx==v.vx) && (y==v.y) && (vy==v.vy);
	}
	inline bool operator!=(const LimpModel& v) const
	{
		return (x!=v.x) || (vx!=v.vx) || (y!=v.y) || (vy!=v.vy);
	}
};

}

#endif
