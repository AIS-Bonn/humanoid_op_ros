#ifndef __GUITHREAD_H__
#define __GUITHREAD_H__

// MS: Global QT gui thread

#include <qt4/QtCore/QThread>
#include <qt4/QtCore/QMutex>
#include <qt4/QtCore/QSemaphore>
#include <qt4/QtGui/QWidget>
#include <qt4/QtCore/QEvent>

// Usage:
//  1) Subclass WidgetProvider in your class.
//  2) Implement virtual QWidget *createWidget() with your
//     widget creation code
//  3) Call createGUI() when you want your widgets instantiated.
//     The system will call createWidget() inside the GUI thread.
//     createGUI() blocks until createWidget() returns.
//  4) Call shutdownGUI() when you want your widget deleted.
//     shutdownGUI() blocks until the widget is deleted.
// HINT: Don't forget to link to guithread!
// HINT: The Player-CMake-Framework sets BUILD_WITH_INSTALL_RPATH=TRUE,
//   which prevents cmake from setting the correct rpath during linking.
//   Set it again to FALSE using
//    set_target_properties(name_of_my_driver PROPERTIES BUILD_WITH_INSTALL_RPATH FALSE)

#include <ros/ros.h>

#include <vector>
#include <string>
#include <iostream>

class RosNodeThreadObserver {
public:
	RosNodeThreadObserver() {}
	virtual ~RosNodeThreadObserver() {}
	
	virtual void initialize() = 0;
	virtual void update() = 0;
};


class RosNodeThread : public QThread {

public:

	RosNodeThread( int argc, char **argv, const char* nodeName );
	~RosNodeThread() {}
	
	inline void addObserver( RosNodeThreadObserver* observer ) { QMutexLocker ml(&m_mutex); m_observers.push_back( observer ); }
	
	inline void setTimeToExit( bool v ) { m_timeToExit = v; }
	
	inline bool isRunning() { return !m_timeToExit; }
	
	inline void setLoopRate( float r ) { m_looprate = r; }
	
	static inline RosNodeThread* getInstance()
	{ return m_instance; }

protected:

	void run();

	int m_argc;
	char** m_argv;
	std::string m_nodeName;
	bool m_timeToExit;
	float m_looprate;
	
	std::vector< RosNodeThreadObserver* > m_observers;
	QMutex m_mutex;
	
	static RosNodeThread* m_instance;
};

class WidgetProvider
{
	public:
		WidgetProvider();
		
		virtual QWidget *createWidget() = 0;
		QWidget* doCreateWidget();
		void doDeleteWidget();
		void createGUI();
		void shutdownGUI();

	protected:
		bool m_guiInitialized;
		bool m_enableGui;
		
	private:
		QSemaphore m_waitSemaphore;
		QWidget *m_widget;
};

class GUIThread : public QObject
{
	public:
		static GUIThread *getInstance();
		
		void addWidgetProvider(WidgetProvider *provider);
		void deleteWidgetOfProvider(WidgetProvider *provider);
		void updateWidget(QWidget *w);
		void exec();
		void quit();
		
		static bool exists() { return mg_instance != NULL; }
		
	private:
		GUIThread();
		virtual bool event(QEvent *event);
		
		static GUIThread *mg_instance;
		static QMutex mg_instanceMutex;
};

#endif // __GUITHREAD_H__
