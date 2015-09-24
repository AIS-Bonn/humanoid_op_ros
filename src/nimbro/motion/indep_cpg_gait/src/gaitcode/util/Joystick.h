#ifndef Joystick_H
#define Joystick_H

#include <QTimer>

/**
 * A Qt oriented high level joystick interface class.
 *
 * The joystick can be used in two different modes. In passive mode (default) the update() function
 * has to be called to examine the state of the joystick. The update() function will emit signals to
 * inform your application whether the joystick connected or disconnected, a button press or a button
 * release was detected and if a stick motion was detected. The update() function also updates the
 * internal public data structures for buttonState and axisState that you can access to examine the
 * joystick state. The return value of the update() method indicates the connection status.
 *
 * In active mode you have to call startPolling(int ms) once and then the update() function will be
 * called periodically with the specified interval. Qt signals will be emitted accordingly.
 *
 * Following signals are emitted:
 * buttonPressed(QList<bool>) with a list of booleans reflecting the current state of all buttons.
 * The signal is only emitted once after a button press was detected.
 * buttonReleased(QList<bool>) with a list of booleans reflecting the current state of all buttons.
 * The signal is only emitted once after a button release was detected.
 * joystickMoved(QList<double>) is emitted each time the state of at least one axis has changed.
 *
 * The public data structures are:
 * The QList<bool> buttonState array contains booleans for each supported button (pressed or not).
 * The QList<double> axisState array contains a value for each axis in the range [-1:1].
 *
 *
 */
class Joystick : public QObject
{
	Q_OBJECT

	int jd;
	QList<bool> buttonStateBefore;
	QList<double> axisStateBefore;
	QTimer timer;

public:

	static const int numOfAxes = 4; // Number of supported axes.
	static const int numOfButtons = 32; // Number of supported buttons.

	bool isConnected;
	QList<bool> buttonState;
	QList<double> axisState;

	Joystick(QObject *parent = 0);
	~Joystick();

	bool init();

public slots:
	void startPolling(int ms = 10);
	bool update();

signals:
	void connected();
	void disconnected();
	void buttonPressed(QList<bool>);
	void buttonReleased(QList<bool>);
	void joystickMoved(QList<double>);
};

#endif //Joystick_H
