#ifndef STATE_H_
#define STATE_H_

#include <QList>
#include <QStringList>
#include <QMutex>
#include <typeinfo>
#include <QDebug>
#include "RobotControl/Action.h"
#include "Command.h"
#include "util/Vec2f.h"
#include "util/Vec3f.h"

namespace indep_cpg_gait
{
	// Represents the current state of the robot and its perception of the world.
	struct State
	{
		int frameId;
		int stepId;
		double time; // Current real time since program start.
		double lastIterationTime; // How long did the last iteration really take? (12 ms?)
		double lastExecutionTime; // The execution time of the last rc iteration. (<< 12 ms)

		double debug; // An all purpose debug value.

		Vec2f fusedAngle; // Estimated angle of the IMU fused from accelerometers and gyros.
		Vec2f DfusedAngle; // Estimated angular rate of the IMU fused from accelerometers and gyros.

		int supportExchange; // True if the support exchange has been detected in this iteration.
		int supportLegSign; // Sign of the leg the robot is currently standing on. 1:right, -1:left

		double gaitFrequency; // The currently used gait frequency.
		double gaitPhase; // The current gait phase.

		Vec3f gcvTarget; // Target gcv accepted once per step and faded towards during the step.
		Vec3f gcv; // Currently applied gait control vector. This determines the actual leg angle during gait.

		Command command;

		Action txAction; // Pose and abstract pose as commanded by the motion layer.
		Action rxAction; // Pose and abstract pose as received from the serial line.

		State();
		~State();
		void init();
		void buffer();
		int size();
		State& operator[](int i);
		double operator()(int i);
		double operator()(QString key);

	private:

		// Registers a member variable for index based access.
		template <typename T>
		void registerMember(QString name, T* member)
		{
			memberNames << name;
			memberOffsets << (char*)member - (char*)this;
			memberTypes << QString(typeid(*member).name());
		}

		// These members are static so that buffering into history does not create copies.
		static QList<off_t> memberOffsets;
		static QList<QString> memberTypes;
		static QMutex mutex;
		static QList<State> history;

	public:
		static QStringList memberNames; // Contains the names of the members in the right order.
	};

	extern State state;
}
#endif /* STATE_H_ */

