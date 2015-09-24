#include "Command.h"
#include "Globals.h"

namespace indep_cpg_gait
{
	// The global command object contains user input from the GUI.
	Command command;
	QList<Command> Command::history;

	Command::Command()
	{
		motionTest = false;
		motionInterfaceTest = false;
		walk = false;
		kick = false;
		feedback = false;
		learnT = false;
		learnY = false;
		log = true;
	}

	// Buffers the current command values into the history.
	void Command::buffer()
	{
		if (history.size() > 20000)
			history.removeFirst();
		history.prepend(*this);
	}

	// Clears the buffered history and resets the members to default values.
	void Command::reset()
	{
		history.clear();
	}

	// Returns the amount of buffered historical command objects.
	int Command::size()
	{
		return history.size();
	}

	// Returns a historical command object.
	// i = 0 returns the current command.
	// i = -1 (or 1) returns the command object from the iteration before and so on.
	// If i < max(-HISTORY_SIZE, history.size()), then the first known command object is returned.
	Command Command::operator[](int i)
	{
		if (i == 0 or history.isEmpty())
			return *this;

		i = qMin(qAbs(i), history.size()) - 1;
		return history[i];
	}
}