#include "Joystick.h"
#include <windows.h>
#include <mmsystem.h>
#include <QDebug>

Joystick::Joystick(QObject *parent) : QObject(parent)
{
	isConnected = false;
	connect(&timer, SIGNAL(timeout()), this, SLOT(update()));

	for (int i = 0; i < numOfButtons; i++)
	{
		buttonStateBefore << false;
		buttonState << false;
	}

	for (int i = 0; i < numOfAxes; i++)
	{
		axisState << 0;
		axisStateBefore << 0;
	}
}

Joystick::~Joystick()
{
}

// Tries to detect the first connected joystick.
// Returns true on success and false if no joystick was found.
// The connected() and disconnected() signals are also emitted as appropriate.
bool Joystick::init()
{
	JOYINFOEX joyInfoEx;
	ZeroMemory(&joyInfoEx, sizeof(joyInfoEx));
	joyInfoEx.dwSize = sizeof(joyInfoEx);
	bool joy1Present = (joyGetPosEx(JOYSTICKID1, &joyInfoEx) == JOYERR_NOERROR);
	if (!isConnected && joy1Present)
		emit connected();
	if (isConnected && !joy1Present)
		emit disconnected();
	isConnected = joy1Present;
	return joy1Present;
}

// Starts periodic polling of the joystick (active mode).
// This is required if the joystick is used in active mode.
void Joystick::startPolling(int ms)
{
	timer.start(ms);
}

// Polls the current joystick state. The return value is an indicator of the joystick connection state.
// Signals are automatically emitted for lost and found connections, detected button presses and releases
// and detected stick motion. Also the internal public data structures are updated (buttonState, axisState)
// for your convenience.
bool Joystick::update()
{
	JOYINFOEX joyInfoEx;
	joyInfoEx.dwSize = sizeof(joyInfoEx);
	joyInfoEx.dwFlags = JOY_RETURNALL;
	int ret = joyGetPosEx(JOYSTICKID1, &joyInfoEx);
	if (ret != JOYERR_NOERROR)
	{
		if (isConnected)
			emit disconnected();

		isConnected = false;
		return false;
	}

	if (!isConnected)
		emit connected();
	isConnected = true;

	bool buttonPressDetected = false;
	bool buttonReleaseDetected = false;
	for (int i = 0; i < numOfButtons; i++)
	{
		buttonStateBefore[i] = buttonState[i];
		buttonState[i] = joyInfoEx.dwButtons & (1 << i);

		if (!buttonStateBefore[i] && buttonState[i])
			buttonPressDetected = true;

		if (buttonStateBefore[i] && !buttonState[i])
			buttonReleaseDetected = true;
	}

	if (buttonPressDetected)
		emit buttonPressed(buttonState);
	if (buttonReleaseDetected)
		emit buttonReleased(buttonState);


	for (int i = 0; i < numOfAxes; i++)
		axisStateBefore[i] = axisState[i];

	axisState[0] = (double)joyInfoEx.dwXpos/32767.5 - 1.0;
	axisState[1] = (double)joyInfoEx.dwYpos/32767.5 - 1.0;
	axisState[2] = (double)joyInfoEx.dwZpos/32767.5 - 1.0;
	axisState[3] = (double)joyInfoEx.dwRpos/32767.5 - 1.0;

	float threshold = 0.05;
	axisState[0] = qAbs(axisState[0]) > threshold ? axisState[0] : 0.0;
	axisState[1] = qAbs(axisState[1]) > threshold ? axisState[1] : 0.0;
	axisState[2] = qAbs(axisState[2]) > threshold ? axisState[2] : 0.0;
	axisState[3] = qAbs(axisState[3]) > threshold ? axisState[3] : 0.0;

	for (int i = 0; i < numOfAxes; i++)
	{
		if (axisState[i] != axisStateBefore[i])
		{
			emit joystickMoved(axisState);
			break;
		}
	}

	return true;
}
