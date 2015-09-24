

#include "qtguithread/qtguithread.h"
#include <nodelet/loader.h>

class LoaderInitializer : public RosNodeThreadObserver
{
public:
        void initialize()
        {
                n = new nodelet::Loader;
        }
        
        void update()
        {
        }
private:
        nodelet::Loader* n;
};


int main(int argc, char** argv) {
        
        // QApplication must run in main thread
        // ros thread can run in a concurrent thread, but all the NodeHandle must only be called from within the thread context..
        GUIThread::getInstance();

        // create the rosnode thread class, nothing really happens in there
        RosNodeThread rosNodeThread( argc, argv, "manager" );
        rosNodeThread.setLoopRate( 100.f );
        

        // create the node implementation class, and add it as observer to the ros node thread class
	LoaderInitializer n;
        rosNodeThread.addObserver( &n );
 
        // now, the ros node thread will be spawned, the node handle will be initialized,
        // then, the observers will be initialized (to load parameters for example)
        rosNodeThread.start();
        
        // to initialize the gui, we must wait until the parameters are loaded
        // such that enable_gui parameters can be set!
        while( !rosNodeThread.isRunning() )
                usleep(10);
        
        // check if a GUI instance has been created (no need to check specific enable_gui variables)
        if( GUIThread::exists() ) {
                // calls the QApplication event loop execution
                GUIThread::getInstance()->exec();
                // trigger closing of the ros node
                rosNodeThread.setTimeToExit( true );
        }

        // wait until the ros node thread exits
        rosNodeThread.wait();
}
