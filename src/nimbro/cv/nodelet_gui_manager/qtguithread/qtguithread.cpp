// MS: Global QT gui thread

#include "qtguithread.h"

#include <qt4/QtGui/QApplication>


GUIThread *GUIThread::mg_instance = 0;
RosNodeThread* RosNodeThread::m_instance = 0;
QMutex GUIThread::mg_instanceMutex;

int fake_argc = 1;
const char *fake_argv[] = {"dummy"};

class WidgetProviderEvent : public QEvent
{
	public:
		WidgetProviderEvent(Type type, WidgetProvider *provider)
		 : QEvent(type), m_provider(provider)
		{
		}
		
		inline WidgetProvider *getProvider()
		{ return m_provider; }
	protected:
		WidgetProvider *m_provider;
};

class NewWidgetProviderEvent : public WidgetProviderEvent
{
	public:
		static const Type TYPE_CODE;
		
		NewWidgetProviderEvent(WidgetProvider *provider)
		 : WidgetProviderEvent(TYPE_CODE, provider)
		{}
};

const QEvent::Type NewWidgetProviderEvent::TYPE_CODE = (QEvent::Type)QEvent::registerEventType();

class DeleteWidgetEvent : public WidgetProviderEvent
{
	public:
		static const Type TYPE_CODE;
		
		DeleteWidgetEvent(WidgetProvider *provider)
	        : WidgetProviderEvent(TYPE_CODE, provider)
		{}
};

const QEvent::Type DeleteWidgetEvent::TYPE_CODE = (QEvent::Type)QEvent::registerEventType();

class UpdateWidgetEvent : public QEvent
{
	public:
		static const Type TYPE_CODE;
		
		UpdateWidgetEvent(QWidget *widget)
		 : QEvent(TYPE_CODE), m_widget(widget)
		{}
		
		inline QWidget *getWidget()
		{ return m_widget; }
	protected:
		QWidget *m_widget;
};

const QEvent::Type UpdateWidgetEvent::TYPE_CODE = (QEvent::Type)QEvent::registerEventType();



RosNodeThread::RosNodeThread( int argc, char **argv, const char* nodeName )
{
	m_argc = argc;
	m_argv = argv;
	m_nodeName = std::string( nodeName );
	
	m_looprate = 10.f;
	
	m_timeToExit = true;

//	ros::NodeHandle n;
	m_instance = this;
}


void RosNodeThread::run() {

    
	ros::init(m_argc, m_argv, m_nodeName.c_str());
	ros::NodeHandle n;
	ros::Rate loop_rate( m_looprate );
	

	{
		QMutexLocker ml(&m_mutex);
		for( size_t i = 0; i < m_observers.size(); i++ )
			m_observers[i]->initialize();
	}
	
	m_timeToExit = false;
	
	usleep( 1000 );

	while ( n.ok() && !m_timeToExit ) {

		{
			QMutexLocker ml(&m_mutex);
			for( size_t i = 0; i < m_observers.size(); i++ )
				m_observers[i]->update();
		}

		ros::spinOnce();
		loop_rate.sleep();

	}
	
	m_timeToExit = true;

	GUIThread::getInstance()->quit();

	n.shutdown();

}



GUIThread* GUIThread::getInstance()
{
	QMutexLocker locker(&mg_instanceMutex);
	
	if(!mg_instance)
	{
		mg_instance = new GUIThread();
	}
	return mg_instance;
}

void GUIThread::addWidgetProvider(WidgetProvider *provider)
{
	QApplication::postEvent(this, new NewWidgetProviderEvent(provider));
}

void GUIThread::deleteWidgetOfProvider(WidgetProvider* provider)
{
	QApplication::postEvent(this, new DeleteWidgetEvent(provider));
}

void GUIThread::updateWidget(QWidget* w)
{
	QApplication::postEvent(this, new UpdateWidgetEvent(w));
}

GUIThread::GUIThread()
{
	new QApplication(fake_argc, (char**)fake_argv);
}

void GUIThread::exec() {
	
	qApp->exec();
	
}

void GUIThread::quit() {
	qApp->quit();
}

bool GUIThread::event(QEvent* event)
{

	if(event->type() == NewWidgetProviderEvent::TYPE_CODE)
	{
		NewWidgetProviderEvent *pevent = (NewWidgetProviderEvent*)event;
		
		pevent->getProvider()->doCreateWidget();
		
		return true;
	}
	
	if(event->type() == DeleteWidgetEvent::TYPE_CODE)
	{
		DeleteWidgetEvent *pevent = (DeleteWidgetEvent*)event;
		
		pevent->getProvider()->doDeleteWidget();
		
		return true;
	}
	
	if(event->type() == UpdateWidgetEvent::TYPE_CODE)
	{
		UpdateWidgetEvent *pevent = (UpdateWidgetEvent*)event;
		
		pevent->getWidget()->update();
		
		return true;
	}
	
	return QObject::event(event);
}


WidgetProvider::WidgetProvider() 
: m_guiInitialized( false )
, m_enableGui( false )
{
}

QWidget *WidgetProvider::doCreateWidget()
{
	m_widget = createWidget();
	return m_widget;
}

void WidgetProvider::doDeleteWidget()
{
	delete m_widget;
}

void WidgetProvider::createGUI()
{
	GUIThread::getInstance()->addWidgetProvider(this);
}

void WidgetProvider::shutdownGUI()
{
	GUIThread::getInstance()->deleteWidgetOfProvider(this);
}
