// Thread for handling megapacket data
// Author: Grzegorz Ficht <ficht@ais.uni-bonn.de>

#ifndef MEGAPACKET_THREAD_H
#define MEGAPACKET_THREAD_H

#include <pthread.h>
#include <stdint.h>

namespace mp
{

class MegapacketThread
{
public:
	MegapacketThread();
	~MegapacketThread();

	void run();

	static void* start(void* ptr);

	boost::shared_ptr<CM740mp> transfer;
	std::vector<MPData>*       servoData;
	MPBoard*                   boardData;
private:
	pthread_mutex_t m_mutex;
	pthread_cond_t m_cond;
};

}





#endif
