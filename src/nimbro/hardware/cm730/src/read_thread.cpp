// Thread for reading data from the serial connection
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

// Includes
#include "cm730/read_thread.h"
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <string.h>

// Defines
#define READ_THREAD_DEBUG  0 // Non-zero => Display incoming and outgoing bytes

// Constants
const size_t BUFSIZE = 1024;

// I/O namespace
namespace io
{

ReadThread::ReadThread()
 : m_writeIdx(0)
 , m_readIdx(0)
{
	m_buf = (uint8_t*)malloc(BUFSIZE);

	pthread_mutex_init(&m_mutex, 0);

	pthread_condattr_t attr;
	pthread_condattr_init(&attr);
	pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);

	pthread_cond_init(&m_cond, &attr);
}

ReadThread::~ReadThread()
{
	free(m_buf);
}

void ReadThread::setFile(int fd)
{
	m_fd = fd;
}

void ReadThread::run()
{
	size_t available_space = BUFSIZE-1;
	while(1)
	{
		int ret = ::read(m_fd, m_buf + m_writeIdx, available_space);
		if(ret < 0)
		{
			perror("Could not read()");
			return;
		}

		if(ret == 0)
			usleep(10);

		pthread_mutex_lock(&m_mutex);

#if READ_THREAD_DEBUG
		printf("ReadThread: got:");
		for(int i = 0; i < ret; ++i)
		{
			printf(" 0x%02X", m_buf[m_writeIdx + i]);
		}
		printf("\n");
#endif

		m_writeIdx += ret;
		if(m_writeIdx == BUFSIZE)
			m_writeIdx = 0;

		if(m_writeIdx >= m_readIdx)
		{
			available_space = BUFSIZE - m_writeIdx;
			if(m_readIdx == 0)
				available_space -= 1;
		}
		else
			available_space = m_readIdx - m_writeIdx - 1;
		pthread_cond_signal(&m_cond);
		pthread_mutex_unlock(&m_mutex);
	}
}

int ReadThread::read(uint8_t* data, size_t size, struct timespec* abstime)
{
	pthread_mutex_lock(&m_mutex);

	size_t bytes_read = 0;

	while(size)
	{
		while(m_readIdx == m_writeIdx)
		{
			int ret = pthread_cond_timedwait(&m_cond, &m_mutex, abstime);
			if(ret == ETIMEDOUT)
			{
				pthread_mutex_unlock(&m_mutex);
				return -ETIMEDOUT;
			}

			if(ret < 0)
			{
				fprintf(stderr, "pthread_cond_timedwait(): %s\n", strerror(ret));
				pthread_mutex_unlock(&m_mutex);
				return -1;
			}
		}

		if(m_readIdx > m_writeIdx)
		{
			size_t b = std::min(BUFSIZE - m_readIdx, size);
			memcpy(data + bytes_read, m_buf + m_readIdx, b);
			m_readIdx += b;
			bytes_read += b;
			size -= b;

			if(m_readIdx == BUFSIZE)
				m_readIdx = 0;
		}

		size_t b = std::min(m_writeIdx - m_readIdx, size);
		memcpy(data + bytes_read, m_buf + m_readIdx, b);
		m_readIdx += b;
		bytes_read += b;
		size -= b;
	}

#if READ_THREAD_DEBUG
	printf("ReadThread: read:");
	for(size_t i = 0; i < bytes_read; ++i)
	{
		printf(" 0x%02X", data[i]);
	}
	printf("\n");
#endif

	pthread_mutex_unlock(&m_mutex);

	return bytes_read;
}

void ReadThread::flush()
{
	// Lock the read thread mutex
	pthread_mutex_lock(&m_mutex);

	// Discard all data already in the local Rx buffer
	m_readIdx = m_writeIdx;

	// Discard all data in the operating system buffer
	tcflush(m_fd, TCIFLUSH);

	// Unlock the read thread mutex
	pthread_mutex_unlock(&m_mutex);
}

void* ReadThread::start(void* ptr)
{
	((ReadThread*)ptr)->run();
	return 0;
}

}
// EOF