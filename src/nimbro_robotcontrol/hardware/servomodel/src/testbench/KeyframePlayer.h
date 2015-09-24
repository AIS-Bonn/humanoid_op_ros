#ifndef KEYFRAMEPLAYER_H_
#define KEYFRAMEPLAYER_H_
#include "globals.h"
#include "Keyframe.h"
#include <QList>

class KeyframePlayer
{
public:

	double V;
	double A;
	double VX;

	QList<Keyframe> keyframes;
	QList<Keyframe> commands;

	int currentCommandIndex;
	Keyframe currentState;

	KeyframePlayer();
	~KeyframePlayer(){};

	void setA(double A);
	void setV(double V);
	void setVX(double VX);

	void clear();
	bool addKeyframe(double t, double x=0, double v=0);
	bool calculateCommands();

	void reset();
	bool atEnd();
	Keyframe step(double t);
	Keyframe evaluateAt(double t);
	double totalTime();

private:
	void transformState(double a, double t, Keyframe& kf);
};

#endif // KEYFRAMEPLAYER_H_
