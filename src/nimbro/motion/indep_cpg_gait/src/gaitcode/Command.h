#ifndef COMMAND_H_
#define COMMAND_H_

#include "util/Vec3f.h"
#include <QList>

namespace indep_cpg_gait
{
	// The global command object contains user input from the GUI.
	struct Command
	{
		Vec3f GCV;
		bool motionTest;
		bool motionInterfaceTest;
		bool walk;
		bool kick;
		bool feedback;
		bool learnT;
		bool learnY;
		bool log;

		Command();
		void buffer();
		void reset();
		int size();
		Command operator[](int i);

	private:
		static QList<Command> history;
	};

	extern Command command;
}
#endif /* COMMAND_H_ */
