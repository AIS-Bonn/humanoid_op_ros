// Thread for handling megapacket data
// Author: Grzegorz Ficht <ficht@ais.uni-bonn.de>

// Includes
#include "cm730/megapacket_thread.h"
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <string.h>

// I/O namespace
namespace mp
{

MegapacketThread::MegapacketThread()
{
	pthread_mutex_init(&m_mutex, 0);

	pthread_condattr_t attr;
	pthread_condattr_init(&attr);
	pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);

	pthread_cond_init(&m_cond, &attr);
}

void MegapacketThread::run()
{
	transfer->receiveMegapacket(*servoData,*boardData);
	transfer->writeMegapacket(*servoData,*boardData);
}

void* MegapacketThread::start(void* ptr)
{
	((MegapacketThread*)ptr)->run();
	return 0;
}

}









// EOF
